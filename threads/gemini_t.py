"""Gemini tabanlı canlı (tool çağıran) AI Copilot thread'i.

Kural-bazlı yanıt motorunun yerine geçer. Operatör sorgusunu Google Gemini
'generateContent' REST uç noktasına gönderir; Gemini gerektiğinde tanımlı
araçları (function calling) çağırır. Araçlar UI thread'inden alınan bir
telemetri/tespit "snapshot"'ı üzerinde çalışır — böylece bu thread UI'ya
dokunmadan (bloklamadan) veri okuyabilir. Rapor indirme gibi UI eylemi
gerektiren araçlar ana thread'e sinyalle iletilir.

Ağ çağrısı stdlib urllib ile yapılır; ekstra bağımlılık yoktur.
"""

import os
import math
import time
import json
import urllib.request
import urllib.error

from PySide6.QtCore import QThread, Signal

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
    "generate_tactical_report araçlarını kullan. Araç sonucu boşsa (veri yoksa) "
    "bunu açıkça söyle, uydurma. Yanıtlarında sade Markdown kullanabilirsin "
    "(kalın için **...**)."
)

# Gemini'ye bildirilen araç (fonksiyon) tanımları — OpenAPI şema alt kümesi.
TOOL_DECLARATIONS = [
    {
        "name": "get_telemetry",
        "description": "SİHA'nın ANLIK telemetrisini döndürür: irtifa (m), hız (m/s), "
                       "otopilot modu, enlem/boylam, yön (heading) ve uydu sayısı.",
    },
    {
        "name": "get_telemetry_history",
        "description": "Uçuş başından bu yana kaydedilen GEÇMİŞ telemetriyi özetler: irtifa/hız/"
                       "batarya için min-maks-ortalama, batarya tüketim hızı, kat edilen mesafe, "
                       "otopilot mod değişimleri ve zaman serisinden örnek noktalar.",
        "parameters": {
            "type": "object",
            "properties": {
                "window_seconds": {
                    "type": "number",
                    "description": "Geriye dönük incelenecek süre (saniye). Verilmezse tüm "
                                   "kayıtlı geçmiş kullanılır.",
                },
            },
        },
    },
    {
        "name": "get_battery_status",
        "description": "Batarya seviyesi (%), voltaj (V) ve durum değerlendirmesini döndürür.",
    },
    {
        "name": "get_detections",
        "description": "YOLO'nun ŞU AN aktif (son saniyelerde görülen) hedeflerini ve canlı tespit "
                       "tablosunu döndürür: tür bazında adet (Araç, Kişi, Kutu vb.), toplam ve "
                       "son tespit koordinatı.",
    },
    {
        "name": "get_detection_history",
        "description": "Uçuş başından bu yana YOLO'nun tespit ettiği TÜM benzersiz hedeflerin "
                       "geçmişini döndürür: her hedef için tür, tepe güven skoru, ilk/son görülme "
                       "zamanı, kaç karede doğrulandığı ve GPS koordinatı; ayrıca tür bazında "
                       "toplamlar ve ilk görülme sırası (kronoloji).",
    },
    {
        "name": "get_vlm_scene",
        "description": "Görüntü-Dil Modeli (VLM) tarafından üretilen anlık sahne açıklamasını döndürür.",
    },
    {
        "name": "get_situation_summary",
        "description": "Genel durum özeti için tek çağrıda hem telemetrinin (anlık + geçmiş trend) "
                       "hem de YOLO tespitlerinin (aktif + geçmiş) birleşik verisini ve VLM sahne "
                       "açıklamasını döndürür.",
    },
    {
        "name": "generate_tactical_report",
        "description": "Mevcut telemetri, tespit ve VLM verilerini birleştirip taktik rapor "
                       "dosyası oluşturur ve operatörün ekranında kaydetme penceresi açar.",
    },
]


class GeminiChatThread(QThread):
    """Tek bir operatör sorgusunu Gemini ile (araç çağırma döngüsü dahil) işler."""

    response_ready = Signal(str)          # nihai asistan metni (Markdown)
    error_occurred = Signal(str)          # hata mesajı
    log_signal = Signal(str, str)         # (metin, seviye)
    action_requested = Signal(str, dict)  # ana thread'de çalıştırılacak eylem (ör. rapor)

    MAX_TOOL_ROUNDS = 5

    def __init__(self, api_key, history, user_query, snapshot,
                 forced_tools=None, model=DEFAULT_MODEL, parent=None):
        super().__init__(parent)
        self.api_key = api_key
        # history: [{"role": "user"/"model", "parts": [{"text": ...}]}, ...]
        self.history = list(history) if history else []
        self.user_query = user_query
        self.snapshot = snapshot or {}
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
    # ARAÇ YARDIMCILARI
    # ------------------------------------------------------------------
    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2):
        """İki koordinat arasındaki yaklaşık yer mesafesi (metre)."""
        r = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return 2 * r * math.asin(math.sqrt(a))

    @staticmethod
    def _minmaxavg(values):
        if not values:
            return None
        return {
            "min": round(min(values), 2),
            "max": round(max(values), 2),
            "avg": round(sum(values) / len(values), 2),
        }

    def _telemetry_now(self):
        tel = self.snapshot.get("telemetry", {})
        batt = tel.get("battery", 0)
        return {
            "altitude_m": round(tel.get("alt", 0.0), 2),
            "speed_ms": round(tel.get("speed", 0.0), 2),
            "mode": tel.get("mode", "?"),
            "lat": round(tel.get("lat", 0.0), 6),
            "lon": round(tel.get("lon", 0.0), 6),
            "heading_deg": tel.get("heading", 0),
            "satellites": tel.get("sats", 0),
            "battery_percent": int(batt),
            "voltage_v": tel.get("voltage", 0.0),
        }

    def _telemetry_history(self, window_seconds=None):
        """Kayıtlı telemetri örneklerinden trend/istatistik çıkarır."""
        samples = self.snapshot.get("telemetry_history", [])
        if not samples:
            return {"sample_count": 0,
                    "note": "Henüz telemetri geçmişi birikmedi (uçuş yeni başlamış olabilir)."}

        if window_seconds:
            cutoff = samples[-1]["t"] - float(window_seconds)
            windowed = [s for s in samples if s["t"] >= cutoff] or samples[-1:]
        else:
            windowed = samples

        alts = [s["alt"] for s in windowed]
        speeds = [s["speed"] for s in windowed]
        batts = [s["battery"] for s in windowed]

        # Mod değişimleri (kronolojik).
        mode_changes = []
        prev = None
        for s in windowed:
            if s["mode"] != prev:
                mode_changes.append({"time": s["time"], "mode": s["mode"]})
                prev = s["mode"]

        # Kat edilen yer mesafesi (ardışık örnekler arası toplam).
        distance = 0.0
        for a, b in zip(windowed, windowed[1:]):
            distance += self._haversine_m(a["lat"], a["lon"], b["lat"], b["lon"])

        span_s = max(0.0, windowed[-1]["t"] - windowed[0]["t"])
        batt_drop = batts[0] - batts[-1] if batts else 0.0
        drain_rate = round(batt_drop / (span_s / 60.0), 2) if span_s >= 30 else None

        # Zaman serisini modele taşımak için ~10 eşit aralıklı örneğe indir.
        step = max(1, len(windowed) // 10)
        trail = [
            {"time": s["time"], "alt_m": round(s["alt"], 1), "speed_ms": round(s["speed"], 1),
             "battery_percent": int(s["battery"]), "mode": s["mode"],
             "lat": round(s["lat"], 5), "lon": round(s["lon"], 5)}
            for s in windowed[::step]
        ][-10:]

        return {
            "window_seconds": round(span_s, 1),
            "sample_count": len(windowed),
            "altitude_m": self._minmaxavg(alts),
            "speed_ms": self._minmaxavg(speeds),
            "battery_percent": {"start": int(batts[0]), "current": int(batts[-1]),
                                "drop": round(batt_drop, 1)},
            "battery_drain_per_min": drain_rate,
            "distance_traveled_m": round(distance, 1),
            "mode_changes": mode_changes,
            "samples": trail,
        }

    @staticmethod
    def _det_row(d):
        return {
            "id": d.get("id"),
            "type": d.get("type"),
            "peak_conf": round(d.get("conf", 0.0), 2),
            "first_seen": d.get("first_time"),
            "last_seen": d.get("time"),
            "frames_confirmed": d.get("hits", 1),
            "lat": round(d.get("lat", 0.0), 5),
            "lon": round(d.get("lon", 0.0), 5),
            "alt_m": round(d["alt"], 1) if d.get("alt") is not None else None,
        }

    def _detections_live(self):
        dets = self.snapshot.get("detections", [])
        active = self.snapshot.get("active_detections", [])
        counts = {}
        for d in dets:
            counts[d.get("type", "?")] = counts.get(d.get("type", "?"), 0) + 1
        return {
            "active_now": [self._det_row(d) for d in active],
            "active_count": len(active),
            "table_total": len(dets),
            "counts_by_type": counts,
            "last_detection": self._det_row(dets[-1]) if dets else None,
            "targets": [self._det_row(d) for d in dets[-10:]],
        }

    def _detections_history(self):
        log = self.snapshot.get("detection_log", [])
        if not log:
            return {"total_unique_targets": 0,
                    "note": "Uçuş başından bu yana YOLO henüz hedef tespit etmedi."}
        counts = {}
        for d in log:
            counts[d.get("type", "?")] = counts.get(d.get("type", "?"), 0) + 1
        confs = [d.get("conf", 0.0) for d in log]
        return {
            "total_unique_targets": len(log),
            "counts_by_type": counts,
            "confidence": self._minmaxavg(confs),
            "first_detection_time": log[0].get("first_time"),
            "last_detection_time": log[-1].get("time"),
            # Kronolojik (ilk görülme sırasına göre) tüm hedefler.
            "targets": [self._det_row(d) for d in log],
        }

    # ------------------------------------------------------------------
    # ARAÇ YÜRÜTÜCÜ (snapshot üzerinde, thread-güvenli okuma)
    # ------------------------------------------------------------------
    def _execute_tool(self, name, args):
        tel = self.snapshot.get("telemetry", {})
        vlm = self.snapshot.get("vlm_summary", "")

        if name == "get_telemetry":
            return self._telemetry_now()

        if name == "get_telemetry_history":
            return self._telemetry_history(args.get("window_seconds"))

        if name == "get_battery_status":
            batt = tel.get("battery", 0)
            return {
                "battery_percent": int(batt),
                "voltage_v": tel.get("voltage", 0.0),
                "assessment": "kritik" if batt < 20 else ("düşük" if batt < 40 else "normal"),
            }

        if name == "get_detections":
            return self._detections_live()

        if name == "get_detection_history":
            return self._detections_history()

        if name == "get_vlm_scene":
            return {"scene_description": vlm or "Henüz sahne açıklaması üretilmedi."}

        if name == "get_situation_summary":
            return {
                "mission_time_s": round(self.snapshot.get("mission_time_s", 0.0), 1),
                "telemetry_now": self._telemetry_now(),
                "telemetry_history": self._telemetry_history(),
                "detections_live": self._detections_live(),
                "detections_history": self._detections_history(),
                "vlm_scene": vlm or "Henüz sahne açıklaması üretilmedi.",
            }

        if name == "generate_tactical_report":
            if self._report_triggered:
                # Aynı sorguda ikinci kez istendi: pencereyi tekrar açma, modele
                # işin bittiğini söyle ki özet metnine geçsin.
                return {"status": "Rapor bu sorguda zaten derlendi ve kaydetme penceresi "
                                  "açıldı; tekrar çağırma, sonucu operatöre özetle."}
            self._report_triggered = True
            # UI eylemi — ana thread'de çalıştırılmak üzere sinyalle bildir.
            self.action_requested.emit("download_report", {})
            return {"status": "Rapor derleyici tetiklendi; kaydetme penceresi açılıyor.",
                    "report_contents": {
                        "telemetry_now": self._telemetry_now(),
                        "telemetry_history": self._telemetry_history(),
                        "detections_history": self._detections_history(),
                        "vlm_scene": vlm or "Henüz sahne açıklaması üretilmedi.",
                    }}

        return {"error": f"Bilinmeyen araç: {name}"}

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

        contents = list(self.history)
        contents.append({"role": "user", "parts": [{"text": self.user_query}]})

        base_payload = {
            "system_instruction": {"parts": [{"text": SYSTEM_PROMPT}]},
            "tools": [{"function_declarations": TOOL_DECLARATIONS}],
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
                        self.log_signal.emit(f"AI Copilot aracı çağırdı: {fname}", "DEBUG")
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
