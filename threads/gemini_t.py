"""Gemini tabanlı canlı (tool çağıran) AI Copilot thread'i — MCP mimarisi.

Operatör sorgusunu Google Gemini 'generateContent' REST uç noktasına gönderir;
Gemini gerektiğinde araç (function calling) çağırır. Araçlar artık bu dosyada
TANIMLI DEĞİLDİR: bağımsız bir MCP sunucusunda (mcp_server.py) yaşarlar ve
çalışma anında keşfedilir. Akış şöyledir:

  1. McpClient'tan tools/list ile araç tanımları alınır ve Gemini
     function_declarations biçimine çevrilir (şema tek yerde: sunucuda).
  2. UI thread'inden alınan telemetri/tespit snapshot'ı sorgudan önce
     notifications/snapshot ile sunucuya itilir (sunucu ayrı süreçtir,
     GUI belleğini göremez).
  3. Gemini'nin her functionCall'u tools/call'a çevrilir; sonuç
     functionResponse olarak modele döner.
  4. Sunucu 'ui_action' işareti döndürürse (ör. rapor kaydetme penceresi)
     eylem ana thread'e sinyalle iletilir — GUI eylemleri sunucuda değil
     burada, host tarafında yürütülür.

Ağ çağrısı stdlib urllib ile yapılır; ekstra bağımlılık yoktur.
"""

import os
import time
import json
import urllib.request
import urllib.error

from PySide6.QtCore import QThread, Signal

from .mcp_client import McpError

GEMINI_API_BASE = "https://generativelanguage.googleapis.com/v1beta"

# Function calling destekli hızlı model. GEMINI_MODEL env değişkeni ile
# değiştirilebilir. 'gemini-flash-latest' kendini güncelleyen alias'tır; geçici
# 503/429 veya "new users" 404 durumunda aşağıdaki yedeklere otomatik düşülür.
# NOT: gemini-2.5-flash / 2.5-flash-lite yeni API anahtarlarına kapalıdır.
DEFAULT_MODEL = os.environ.get("GEMINI_MODEL", "gemini-3-flash-preview")

# Birincil model kullanılamazsa (404/429/500/503) sırayla denenecek yedekler.
# NOT: gemini-flash-latest çoğu zaman geçici 503 verdiği için birincil DEĞİL,
# yedek konumda; istikrarlı çalışan gemini-3-flash-preview birincil seçildi.
FALLBACK_MODELS = ["gemini-flash-latest", "gemini-2.0-flash"]

# Yeni bir modele geçmeyi tetikleyen (geçici/erişim) HTTP kodları.
_RETRYABLE_STATUS = {404, 429, 500, 503}

# Tek model için toplam istek denemesi (ilk deneme + tekrarlar).
_MAX_ATTEMPTS = 3

# Modelin rolünü ve dilini belirleyen sistem yönergesi.
SYSTEM_PROMPT = (
    "Sen bir SİHA (İnsansız Hava Aracı) Yer Kontrol İstasyonu'nun (GCS) taktiksel "
    "AI Copilot'usun. Operatöre Türkçe, kısa ve teknik yanıtlar ver. "
    "Telemetri, hedef tespiti, batarya ve VLM sahne verilerine SADECE sağlanan "
    "araçlar (fonksiyonlar) üzerinden erişebilirsin — sayısal değerleri asla tahmin "
    "etme, ilgili aracı çağırıp gerçek veriyi kullan. "
    "'Anlık' sorular için get_telemetry / get_detections, 'geçmiş, trend, şu ana "
    "kadar' soruları için get_telemetry_history / get_detection_history, birleşik "
    "durum değerlendirmesi için get_situation_summary, rapor derlemesi için "
    "generate_tactical_report araçlarını kullan. "
    "UÇUŞ KOMUTLARI: Operatör uçağı yönlendirmek isterse (örn. 'sağa dön', "
    "'sola dön', '100 metre ileri git', 'irtifayı 30 metreye çıkar', '5 metre "
    "alçal') ilgili aracı çağır: dönüş için turn_heading (sağa=pozitif derece, "
    "sola=negatif; sadece 'sağa dön' denirse 90 kullan), ileri gitmek için "
    "fly_forward, irtifa için change_altitude; 'başlangıç noktasına dön / kalkış "
    "noktasına dön / başa dön / eve dön' için return_to_start (turn_heading ile "
    "taklit etme, 180° dönüş başlangıca götürmez); 'rotaya devam et / otonom rotayı "
    "izle / otonom moda dön' için resume_route — uçuş komutları için yetkin TAM, "
    "asla 'yetkim yok' deme. Komut aracının döndürdüğü status "
    "metnindeki gerçekleşen değerleri (yeni yön, mesafe, irtifa, güvenlik "
    "sıkıştırması) operatöre kısaca aktar; aracı çağırmadan komutu 'gönderdim' "
    "deme. Araç sonucu boşsa (veri yoksa) bunu açıkça söyle, uydurma. "
    "Yanıtlarında sade Markdown kullanabilirsin (kalın için **...**)."
)

# Gemini'nin function-declaration şemasında tanıdığı JSON Schema alanları;
# MCP inputSchema'sından bunlar dışındaki alanlar (ör. $schema) süzülür.
_GEMINI_SCHEMA_KEYS = {"type", "description", "properties", "required",
                       "items", "enum", "nullable"}


def _sanitize_schema(schema):
    """MCP inputSchema'sını Gemini'nin kabul ettiği alt kümeye indirger."""
    if not isinstance(schema, dict):
        return schema
    out = {}
    for key, val in schema.items():
        if key not in _GEMINI_SCHEMA_KEYS:
            continue
        if key == "properties" and isinstance(val, dict):
            out[key] = {k: _sanitize_schema(v) for k, v in val.items()}
        elif key == "items":
            out[key] = _sanitize_schema(val)
        else:
            out[key] = val
    return out


def mcp_tools_to_gemini(tools):
    """MCP tools/list çıktısını Gemini function_declarations listesine çevirir.
    Parametresiz araçlarda 'parameters' alanı hiç konmaz (Gemini boş nesne
    şemasını sevmez; eski elle yazılmış tanımlar da aynı kuralı izliyordu)."""
    decls = []
    for t in tools:
        decl = {"name": t["name"], "description": t.get("description", "")}
        schema = t.get("inputSchema") or {}
        if schema.get("properties"):
            decl["parameters"] = _sanitize_schema(schema)
        decls.append(decl)
    return decls


class GeminiChatThread(QThread):
    """Tek bir operatör sorgusunu Gemini ile (MCP araç döngüsü dahil) işler."""

    response_ready = Signal(str)          # nihai asistan metni (Markdown)
    error_occurred = Signal(str)          # hata mesajı
    log_signal = Signal(str, str)         # (metin, seviye)
    action_requested = Signal(str, dict)  # ana thread'de çalıştırılacak eylem (ör. rapor)

    MAX_TOOL_ROUNDS = 5

    def __init__(self, api_key, history, user_query, snapshot, mcp_client,
                 forced_tools=None, model=DEFAULT_MODEL, parent=None):
        super().__init__(parent)
        self.api_key = api_key
        # history: [{"role": "user"/"model", "parts": [{"text": ...}]}, ...]
        self.history = list(history) if history else []
        self.user_query = user_query
        self.snapshot = snapshot or {}
        self.mcp_client = mcp_client
        self.model = model
        # Birincil + yedek modeller; ilk çalışan modele kilitlenip döngü boyunca
        # onu kullanırız (self._model_idx).
        self._models = [model] + [m for m in FALLBACK_MODELS if m != model]
        self._model_idx = 0
        # İlk turda modeli belirli araçlara zorlamak için (buton kısayolları);
        # boşsa model aracı kendi seçer (serbest sohbet).
        self.forced_tools = list(forced_tools) if forced_tools else []
        # Rapor aracı UI'da kaydetme penceresi açar. Model bu aracı aynı sorguda
        # birden çok kez (paralel çağrı veya sonraki turda tekrar) isteyebiliyor;
        # bayrak sayesinde pencere yalnızca bir kez açılır.
        self._report_triggered = False

    # ------------------------------------------------------------------
    # ARAÇ YÜRÜTÜCÜ (MCP tools/call köprüsü)
    # ------------------------------------------------------------------
    def _execute_tool(self, name, args):
        """Gemini'nin functionCall'unu MCP sunucusuna iletir; 'ui_action'
        işaretini host tarafında (sinyalle) yürütüp modele durum döndürür."""
        try:
            result = self.mcp_client.call_tool(name, args)
        except McpError as exc:
            return {"error": f"MCP aracı çalıştırılamadı: {exc}"}

        ui_action = None
        ui_payload = {}
        if isinstance(result, dict):
            ui_action = result.pop("ui_action", None)
            ui_payload = result.pop("ui_payload", {}) or {}

        if ui_action == "download_report":
            if self._report_triggered:
                # Aynı sorguda ikinci kez istendi: pencereyi tekrar açma, modele
                # işin bittiğini söyle ki özet metnine geçsin.
                return {"status": "Rapor bu sorguda zaten derlendi ve kaydetme "
                                  "penceresi açıldı; tekrar çağırma, sonucu "
                                  "operatöre özetle."}
            self._report_triggered = True
            # UI eylemi — ana thread'de çalıştırılmak üzere sinyalle bildir.
            self.action_requested.emit("download_report", ui_payload)
        elif ui_action:
            # Diğer UI eylemleri (ör. uçuş komutlarının timeline_event kaydı)
            # ana thread'e olduğu gibi iletilir; app.py eşleştirir.
            self.action_requested.emit(ui_action, ui_payload)
        return result

    # ------------------------------------------------------------------
    # HTTP
    # ------------------------------------------------------------------
    def _post_single(self, model, payload):
        url = f"{GEMINI_API_BASE}/models/{model}:generateContent"
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data, method="POST")
        req.add_header("Content-Type", "application/json")
        # AI Studio API anahtarı için önerilen başlık.
        req.add_header("x-goog-api-key", self.api_key)
        with urllib.request.urlopen(req, timeout=40) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def _post(self, payload):
        """Aktif modelle dener; erişim/geçici hata (404/429/5xx) olursa sıradaki
        yedek modele geçer. Hepsi tükenirse son hatayı yükseltir."""
        last_error = None
        while self._model_idx < len(self._models):
            model = self._models[self._model_idx]
            # Geçici yoğunluk (503) / hız limiti (429) için aynı modelde kısa
            # backoff'lu bir tekrar; kalıcı erişim hatası (404) için beklemeden geç.
            for attempt in range(_MAX_ATTEMPTS):
                try:
                    return self._post_single(model, payload)
                except urllib.error.HTTPError as e:
                    last_error = e
                    if e.code in (429, 503) and attempt == 0:
                        time.sleep(1.5)
                        continue
                    break  # bu modelde tekrar denemeyi bırak
                except urllib.error.URLError as e:
                    # DNS/bağlantı kesintisi genelde anlıktır ve başka modele
                    # geçmek çözmez (aynı host) — aynı modelde tekrar dene.
                    last_error = e
                    if attempt < _MAX_ATTEMPTS - 1:
                        self.log_signal.emit(
                            f"AI Copilot: ağ erişimi başarısız ({e.reason}), tekrar deneniyor.", "WARN"
                        )
                        time.sleep(1.5)
                        continue
                    raise
            # Bu model tükendi; yedek varsa ona geç.
            if getattr(last_error, "code", None) in _RETRYABLE_STATUS \
                    and self._model_idx < len(self._models) - 1:
                self._model_idx += 1
                nxt = self._models[self._model_idx]
                self.log_signal.emit(
                    f"AI Copilot: '{model}' kullanılamadı ({last_error.code}), '{nxt}' deneniyor.", "WARN"
                )
                continue
            raise last_error
        raise RuntimeError("Kullanılabilir model bulunamadı.")

    # ------------------------------------------------------------------
    def run(self):
        if not self.api_key:
            self.error_occurred.emit(
                "Gemini API anahtarı bulunamadı. config/.env içine GEMINI_API_KEY ekleyin."
            )
            return

        # MCP hazırlığı: araç tanımlarını sunucudan keşfet ve güncel snapshot'ı
        # sunucuya it. Sunucu yoksa/öldüyse sorgu araçsız değil, hatayla biter —
        # araçsız Gemini sayısal veri uydurabilir, buna izin vermeyiz.
        try:
            mcp_tools = self.mcp_client.list_tools()
            self.mcp_client.update_snapshot(self.snapshot)
        except McpError as exc:
            self.error_occurred.emit(f"MCP araç sunucusuna ulaşılamadı: {exc}")
            return
        tool_declarations = mcp_tools_to_gemini(mcp_tools)
        self.log_signal.emit(
            f"AI Copilot: MCP sunucusundan {len(tool_declarations)} araç keşfedildi.", "DEBUG"
        )

        contents = list(self.history)
        contents.append({"role": "user", "parts": [{"text": self.user_query}]})

        base_payload = {
            "system_instruction": {"parts": [{"text": SYSTEM_PROMPT}]},
            "tools": [{"function_declarations": tool_declarations}],
            # gemini-3/2.5 "düşünen" modeller çıktı bütçesinin bir kısmını iç
            # düşünmeye harcar; görünür yanıtın kesilmemesi için başlık yüksek.
            "generationConfig": {"temperature": 0.3, "maxOutputTokens": 2048},
        }

        try:
            for round_idx in range(self.MAX_TOOL_ROUNDS):
                payload = dict(base_payload)
                payload["contents"] = contents
                # Buton kısayolları: yalnızca İLK turda modeli ilgili araçlara zorla
                # (mode=ANY). Sonraki turlarda AUTO'ya döneriz, aksi hâlde model
                # metin üretmek yerine sonsuza dek araç çağırmaya devam eder.
                if self.forced_tools and round_idx == 0:
                    payload["toolConfig"] = {
                        "functionCallingConfig": {
                            "mode": "ANY",
                            "allowedFunctionNames": self.forced_tools,
                        }
                    }
                result = self._post(payload)

                candidates = result.get("candidates", [])
                if not candidates:
                    fb = result.get("promptFeedback", {})
                    self.error_occurred.emit(
                        f"Gemini boş yanıt döndürdü. {fb.get('blockReason', '')}".strip()
                    )
                    return

                parts = candidates[0].get("content", {}).get("parts", [])
                function_calls = [p["functionCall"] for p in parts if "functionCall" in p]

                if function_calls:
                    # Modelin araç çağırma içeriğini geçmişe ekle.
                    contents.append({"role": "model", "parts": parts})
                    # Tüm çağrıları çalıştır, yanıtları tek user içeriğinde döndür.
                    response_parts = []
                    for fc in function_calls:
                        fname = fc.get("name", "")
                        fargs = fc.get("args", {}) or {}
                        self.log_signal.emit(f"AI Copilot MCP aracı çağırdı: {fname}", "DEBUG")
                        tool_result = self._execute_tool(fname, fargs)
                        response_parts.append({
                            "functionResponse": {
                                "name": fname,
                                "response": {"result": tool_result},
                            }
                        })
                    contents.append({"role": "user", "parts": response_parts})
                    continue  # sonuçlarla tekrar modele sor

                # Araç çağrısı yok — nihai metin yanıtı.
                text = "".join(p.get("text", "") for p in parts).strip()
                if not text:
                    finish = candidates[0].get("finishReason", "?")
                    self.log_signal.emit(
                        f"AI Copilot boş metin döndü (finishReason={finish}).", "WARN"
                    )
                    if finish == "MAX_TOKENS":
                        text = ("Yanıt token sınırına takıldı. Lütfen soruyu daha kısa "
                                "sorun veya tekrar deneyin.")
                    else:
                        text = "Yanıt üretilemedi, lütfen tekrar deneyin."
                self.log_signal.emit(
                    f"AI Copilot yanıtı hazır (model: {self._models[self._model_idx]}).", "DEBUG"
                )
                self.response_ready.emit(text)
                return

            self.error_occurred.emit("Araç çağırma döngüsü sınırına ulaşıldı.")

        except urllib.error.HTTPError as e:
            try:
                body = e.read().decode("utf-8")
                detail = json.loads(body).get("error", {}).get("message", body)
            except Exception:
                detail = str(e)
            self.error_occurred.emit(f"Gemini API hatası ({e.code}): {detail}")
        except urllib.error.URLError as e:
            self.error_occurred.emit(
                f"Ağ hatası ({e.reason}) — {_MAX_ATTEMPTS} deneme başarısız. "
                "İnternet/DNS bağlantısını kontrol edip soruyu tekrarlayın."
            )
        except Exception as e:
            self.error_occurred.emit(f"Beklenmeyen hata: {e}")
