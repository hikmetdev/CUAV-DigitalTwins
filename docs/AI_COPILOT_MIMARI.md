# AI Copilot Mimarisi: Gömülü Tool'dan MCP'ye

Bu belge, SİHA GCS projesindeki AI Copilot'un araç (tool) mimarisinin evrimini
anlatır: kural-bazlı motor → Gemini function-calling (gömülü araçlar) →
MCP (Model Context Protocol) mimarisi.

## 1. Sistemin Genel Yapısı

Sistem bir SİHA Yer Kontrol İstasyonu (GCS) simülasyon ortamıdır:

- **Simülasyon katmanı:** Gazebo (dünya + alt kamera) + ArduPilot SITL +
  MAVROS + ros_gz_bridge — hepsini `app.py` alt süreç olarak başlatıp
  loglarını `logs/build_*/` altına yazar.
- **Otonom uçuş:** `siha_control.py` — rota üretimi, AUTO kalkış, GUIDED'de
  `MAV_CMD_DO_REPOSITION` ile hedef takibi, 900 m sonunda LOITER kuralı.
- **Algı:** `yolo.py` (YOLOE tespit + QR/renk sınıflandırma) ve VLM sahne
  özeti; sonuçlar ROS konularıyla GUI'ye akar.
- **GUI:** PySide6 — `interface/` (harita, kamera, telemetri, timeline) +
  `threads/` (telemetri, kamera, tespit alıcı thread'leri).
- **AI Copilot:** Gemini function-calling + MCP.

Copilot'un evrimi üç aşama geçirdi:

1. **Kural-bazlı yanıt motoru** — regex/anahtar kelime ile hazır cevap, LLM yok.
2. **Gemini function-calling, gömülü araçlar** — araçlar `threads/gemini_t.py`
   içine gömülü (commit `40490a4` ve öncesi).
3. **MCP mimarisi** — araçlar bağımsız sürece taşındı (commit `259577a`).

## 2. MCP ÖNCESİ: Gömülü Tool Mimarisi

Her şey tek dosyada (`threads/gemini_t.py`, ~500 satır), tek süreçte:

```
┌─────────────────── app.py (GUI süreci) ───────────────────┐
│                                                           │
│  UI thread ──snapshot (dict kopyası)──► GeminiChatThread  │
│                                          │                │
│                    ┌─────────────────────┤                │
│                    │ TOOL_DECLARATIONS   │  (elle yazılmış │
│                    │ (Gemini formatında) │   Gemini şeması)│
│                    │ _execute_tool()     │  (if/elif zinciri)
│                    │ _telemetry_now()... │  (araç gövdeleri)
│                    └─────────┬───────────┘                │
│                              ▼                            │
│                    Gemini REST API (urllib)               │
└───────────────────────────────────────────────────────────┘
```

### Çalışma şekli

1. **Araç tanımı:** `TOOL_DECLARATIONS` listesi doğrudan Gemini'nin
   function-declaration formatında elle yazılmıştı. 8 araç vardı:
   `get_telemetry`, `get_telemetry_history`, `get_battery_status`,
   `get_detections`, `get_detection_history`, `get_vlm_scene`,
   `get_situation_summary`, `generate_tactical_report`.
2. **Veri erişimi:** Sorgu başlarken UI thread'i telemetri/tespit geçmişinin
   bir **snapshot kopyasını** thread'e constructor'dan veriyordu; araçlar bu
   dict üzerinde çalıştığı için UI'yı kilitlemeden thread-güvenli okuma
   yapılıyordu.
3. **Yürütme:** Gemini `functionCall` döndürünce `_execute_tool(name, args)`
   içindeki if/elif zinciri ilgili yardımcı metodu çağırıp sonucu
   `functionResponse` olarak modele geri veriyordu; döngü `MAX_TOOL_ROUNDS=5`
   tura kadar sürüyordu.
4. **UI eylemleri:** Rapor gibi pencere açan işler Qt sinyaliyle
   (`action_requested`) ana thread'e devrediliyordu.

### Sorunları

- Araç **tanımı**, **yürütmesi** ve **LLM konuşma döngüsü** aynı sınıfta iç içeydi.
- Şema Gemini'ye özel formattaydı — başka LLM'e geçiş = yeniden yazım.
- Yeni araç eklemek hem tanım listesine hem if/elif zincirine dokunmak demekti.
- Araçlar GUI süreciyle aynı bellekte çalıştığından bir araç hatası GUI'yi
  etkileyebilirdi.

## 3. MCP SONRASI: Mevcut Mimari

Commit `259577a` ile araçlar bağımsız bir sürece taşındı:

```
┌────────── app.py (GUI süreci) ──────────┐      ┌── mcp_server.py (ayrı süreç) ──┐
│                                         │      │                                │
│  UI thread                              │      │  TOOLS[]  (JSON Schema,        │
│    │ snapshot                           │      │           LLM'den bağımsız)    │
│    ▼                                    │stdio │  execute_tool()                │
│  GeminiChatThread ◄──► McpClient ◄──────┼──────┼─►handle_message() JSON-RPC 2.0 │
│    │  tools/list → Gemini şemasına      │      │  SNAPSHOT (son bildirim)       │
│    │  çevir (_sanitize_schema)          │      │  FlightCommander (rclpy,       │
│    ▼                                    │      │   MAV_CMD_DO_REPOSITION)       │
│  Gemini REST API                        │      └────────────────────────────────┘
└─────────────────────────────────────────┘
```

### Parçalar

- **`mcp_server.py`** — araçların yeni evi. Stdio üzerinde satır-ayrımlı
  JSON-RPC 2.0 konuşan, sıfır bağımlılıklı bir MCP sunucusu:
  - `initialize` el sıkışması, `tools/list` (tanımlar standart **JSON Schema**
    ile), `tools/call` (sonuç `content[{type:"text"}]` + `isError` ile döner;
    araç hatası protokol hatası sayılmaz).
  - Veri araçlarına ek **uçuş komut araçları**: `turn_heading`, `fly_forward`,
    `change_altitude`, `change_speed`, `return_to_start`, `resume_route` —
    tembel başlatılan bir rclpy düğümü (`FlightCommander`) ile GUIDED modda
    `MAV_CMD_DO_REPOSITION` (hız için `MAV_CMD_DO_CHANGE_SPEED`) göndererek
    uçağı gerçekten yönlendirir.
  - İlk uçuş komutunda `/siha/operator_override` yayınlanır; otonom rota
    izleyicisi (`siha_control.py`) 900 m LOITER kuralını askıya alıp kontrolü
    operatöre bırakır. `resume_route` askıyı kaldırır.
- **`threads/mcp_client.py`** — host tarafındaki istemci. Sunucuyu uygulama
  ömrü boyunca **tek alt süreç** olarak tutar (her sorguda süreç başlatma
  maliyeti yok); kilitle korunan istek/yanıt eşleme, `select` ile 10 sn zaman
  aşımı, ölen sunucuyu bir sonraki sorguda yeniden başlatma.
- **`threads/gemini_t.py`** — artık araç içermez, sadece köprüdür:
  1. Her sorguda `tools/list` ile araçları **çalışma anında keşfeder** ve
     `mcp_tools_to_gemini()` ile Gemini formatına çevirir.
  2. Snapshot'ı `notifications/snapshot` özel bildirimiyle sunucuya iter
     (sunucu ayrı süreç olduğundan GUI belleğini göremez).
  3. Her `functionCall`'u `tools/call`'a çevirir.
  4. Sunucu `ui_action` işareti döndürürse (rapor penceresi, timeline kaydı)
     eylem Qt sinyaliyle **host tarafında** yürütülür — GUI eylemleri sunucuda
     değil host'ta kalır.

## 4. Karşılaştırma

| Konu | Gömülü (eski) | MCP (yeni) |
|---|---|---|
| Araç tanımı | `gemini_t.py` içinde, Gemini'ye özel formatta | `mcp_server.py`'de, standart JSON Schema; çalışma anında keşif |
| Araç yürütme | Aynı süreç, aynı sınıf, if/elif | Ayrı süreç, JSON-RPC `tools/call` |
| LLM bağımlılığı | Gemini'ye kilitli | LLM'den bağımsız; her MCP istemcisiyle çalışır |
| Veri akışı | Snapshot constructor parametresi (paylaşılan bellek) | Her sorguda `notifications/snapshot` ile serileştirilip itilir |
| Yeni araç ekleme | İki yeri değiştir (tanım + zincir), GUI koduna dokun | Sadece sunucuya ekle; istemci otomatik keşfeder |
| Hata izolasyonu | Araç istisnası thread'i/GUI'yi etkileyebilir | Sunucu çökse bile GUI yaşar; istemci yeniden başlatır |
| Yetenek | Sadece okuma + rapor | + Gerçek uçuş komutları (rclpy sunucu tarafında) |
| Maliyet | Doğrudan fonksiyon çağrısı (ns) | Süreçler arası JSON gidiş-dönüşü (ms) + snapshot serileştirme |

## 5. Takas ve Tasarım Kararları

- **Gömülü yaklaşım:** daha az parça, sıfır IPC maliyeti, paylaşılan bellek
  kolaylığı — ama araçlar tek LLM'e ve tek uygulamaya kaynaklanır.
- **MCP:** araçları "tek doğruluk kaynağı" olan bağımsız, keşfedilebilir,
  standart bir servise dönüştürür — bedeli taşıma katmanı yazmaktır (süreç
  yönetimi, el sıkışma, zaman aşımı, snapshot itme).
- Bu projede bedel bilinçli küçük tutuldu: SDK kullanılmadan ~720 satırlık
  saf-stdlib sunucu + ~160 satırlık istemciyle protokolün yalnızca gereken
  alt kümesi (`initialize`, `tools/list`, `tools/call`, bildirimler)
  uygulandı.
- `notifications/snapshot`, standart MCP'de olmayan projeye özel pragmatik
  bir eklentidir: canlı GUI verisini her sorgudan önce sunucuya taşır.
