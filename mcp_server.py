"""SİHA GCS araçları için MCP (Model Context Protocol) sunucusu.

AI Copilot'un araçları (telemetri, tespit, rapor...) eskiden GeminiChatThread
içine gömülüydü; artık bu bağımsız süreç MCP standardıyla sunar. Uygulama
(app.py) bu sunucuyu stdio üzerinden alt süreç olarak başlatır; istemci
(threads/mcp_client.py) MCP el sıkışmasını yapar, araç listesini keşfeder
(tools/list) ve Gemini'nin function-calling çağrılarını tools/call'a çevirir.
Böylece araç tanımları TEK yerde (burada) yaşar ve LLM'den bağımsızdır: aynı
sunucu Claude/Gemini fark etmeksizin her MCP istemcisiyle konuşabilir.

Taşıma: MCP stdio taşıması — satır sonuyla ayrılmış JSON-RPC 2.0 mesajları
(stdin'den istek, stdout'a yanıt; loglar stderr'e). Ek bağımlılık yoktur.

Canlı veri: araçlar GUI'nin biriktirdiği telemetri/tespit "snapshot"'ı üzerinde
çalışır. Sunucu ayrı süreç olduğu için GUI belleğine erişemez; istemci her
sorgudan önce güncel snapshot'ı 'notifications/snapshot' bildirimiyle gönderir,
sunucu son geleni saklar ve araçlar onu okur.

Uçuş komutları (2026-07-18): turn_heading / fly_forward / change_altitude
araçları uçağı GERÇEKTEN yönlendirir. siha_control.py ile aynı mekanizma
kullanılır: GUIDED modda MAVROS /mavros/cmd/command_int servisi üzerinden
MAV_CMD_DO_REPOSITION (192). Sunucu bunun için tembel başlatılan küçük bir
rclpy düğümü tutar (FlightCommander); ilk uçuş komutunda ayrıca
/siha/operator_override konusuna yayın yapılır ki otonom rota izleyicisi
(siha_control) 900 m LOITER kuralını uygulamayı bırakıp kontrolü operatöre
devretsin. rclpy importları tembeldir: ROS ortamı yoksa veri araçları
çalışmaya devam eder, yalnızca uçuş araçları hata döndürür.
"""

import sys
import json
import math
import time

PROTOCOL_VERSION = "2025-06-18"
SERVER_INFO = {"name": "siha-gcs-tools", "version": "1.0.0"}

# İstemcinin 'notifications/snapshot' ile gönderdiği son durum kopyası.
SNAPSHOT = {}

# ----------------------------------------------------------------------
# ARAÇ TANIMLARI (tools/list bunları döndürür; şema = JSON Schema)
# ----------------------------------------------------------------------
TOOLS = [
    {
        "name": "get_telemetry",
        "description": "SİHA'nın ANLIK telemetrisini döndürür: irtifa (m), hız (m/s), "
                       "otopilot modu, enlem/boylam, yön (heading) ve uydu sayısı.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_telemetry_history",
        "description": "Uçuş başından bu yana kaydedilen GEÇMİŞ telemetriyi özetler: irtifa/hız/"
                       "batarya için min-maks-ortalama, batarya tüketim hızı, kat edilen mesafe, "
                       "otopilot mod değişimleri ve zaman serisinden örnek noktalar.",
        "inputSchema": {
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
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_detections",
        "description": "YOLO'nun ŞU AN aktif (son saniyelerde görülen) hedeflerini ve canlı tespit "
                       "tablosunu döndürür: tür bazında adet (Kutu, Kırmızı/Mavi Kutu vb.), toplam ve "
                       "son tespit koordinatı.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_detection_history",
        "description": "Uçuş başından bu yana YOLO'nun tespit ettiği TÜM benzersiz hedeflerin "
                       "geçmişini döndürür: her hedef için tür, tepe güven skoru, ilk/son görülme "
                       "zamanı, kaç karede doğrulandığı ve GPS koordinatı; ayrıca tür bazında "
                       "toplamlar ve ilk görülme sırası (kronoloji).",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_vlm_scene",
        "description": "Görüntü-Dil Modeli (VLM) tarafından üretilen anlık sahne açıklamasını döndürür.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_situation_summary",
        "description": "Genel durum özeti için tek çağrıda hem telemetrinin (anlık + geçmiş trend) "
                       "hem de YOLO tespitlerinin (aktif + geçmiş) birleşik verisini ve VLM sahne "
                       "açıklamasını döndürür.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "generate_tactical_report",
        "description": "Mevcut telemetri, tespit ve VLM verilerini birleştirip taktik rapor "
                       "dosyası oluşturur ve operatörün ekranında kaydetme penceresi açar.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    # ---- UÇUŞ KOMUTLARI: uçağı gerçekten yönlendirir (GUIDED + DO_REPOSITION) ----
    {
        "name": "turn_heading",
        "description": "Uçağın burnunu MEVCUT yönüne göre döndürür ve yeni yönde düz uçuşa "
                       "devam ettirir. Pozitif derece = SAĞA, negatif = SOLA dönüş "
                       "(örn. 'sağa dön' = 90, 'sola dön' = -90, 'geri dön' = 180). "
                       "İrtifa korunur. Otonom rota izlemeyi devre dışı bırakır.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "degrees": {
                    "type": "number",
                    "description": "Dönüş miktarı (derece, -180..180). Pozitif sağa, negatif sola.",
                },
            },
            "required": ["degrees"],
        },
    },
    {
        "name": "fly_forward",
        "description": "Uçağı MEVCUT yönünde verilen mesafe kadar ileriye gönderir. Sabit "
                       "kanatlı uçak hedefe varınca o noktanın etrafında LOITER (daire) "
                       "çizerek bekler; bunu operatöre söyle. İrtifa korunur. Otonom rota "
                       "izlemeyi devre dışı bırakır.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "distance_m": {
                    "type": "number",
                    "description": "İleri gidilecek mesafe (metre, 50-2000 arasına sıkıştırılır).",
                },
            },
            "required": ["distance_m"],
        },
    },
    {
        "name": "return_to_start",
        "description": "Uçağı görevin BAŞLANGIÇ (kalkış/home) NOKTASINA geri döndürür ve "
                       "orada LOITER ile bekletir. Operatör 'başlangıç noktasına dön', "
                       "'kalkış noktasına dön', 'başa dön', 'eve dön' dediğinde bunu çağır; "
                       "turn_heading(180) ile TAKLİT ETME — 180° dönüş uçağı başlangıca "
                       "götürmez. İrtifa korunur. Otonom rota izlemeyi devre dışı bırakır.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "resume_route",
        "description": "Manuel uçuş komutlarından sonra OTONOM ROTA İZLEMEYE GERİ DÖNER: "
                       "uçak GUIDED modda orijinal düz rotaya (kuzey) yönelir ve 900 m "
                       "görev kuralı kaldığı yerden sürer. Operatör 'rotaya devam et', "
                       "'otonom rotayı izle', 'otonom moda dön' dediğinde bunu çağır.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "change_speed",
        "description": "Uçuş hızını değiştirir (hedef HAVA hızı, m/s). 'hızı 15 yap' için "
                       "target_speed_ms=15; '3 m/s hızlan' için change_ms=3, 'yavaşla' için "
                       "change_ms=-3 kullan (ikisinden birini ver). Güvenlik için 10-22 m/s "
                       "aralığına sıkıştırılır. Rotayı ve modu DEĞİŞTİRMEZ: uçak mevcut "
                       "hedefine/otonom rotasına yeni hızla devam eder.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "target_speed_ms": {
                    "type": "number",
                    "description": "Hedef hava hızı (m/s).",
                },
                "change_ms": {
                    "type": "number",
                    "description": "Mevcut hıza eklenecek fark (pozitif=hızlan, negatif=yavaşla).",
                },
            },
        },
    },
    {
        "name": "change_altitude",
        "description": "Uçuş irtifasını değiştirir; uçak mevcut yönünde düz uçmaya devam "
                       "ederken yeni irtifaya tırmanır/alçalır. 'irtifayı 30 metreye çıkar' "
                       "için target_altitude_m=30; '5 metre alçal' için change_m=-5 kullan "
                       "(ikisinden birini ver). Güvenlik için 10-120 m aralığına sıkıştırılır.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "target_altitude_m": {
                    "type": "number",
                    "description": "Hedef mutlak irtifa (metre, kalkış noktasına göre).",
                },
                "change_m": {
                    "type": "number",
                    "description": "Mevcut irtifaya eklenecek fark (pozitif=yüksel, negatif=alçal).",
                },
            },
        },
    },
    {
        "name": "start_flight",
        "description": "Uçağı pistten havalandırıp otonom görev uçuşuna başlatır. Operatör 'uçak uçmaya hazır', "
                       "'kalkışa geç', 'uçuşa başla', 'uçağı kaldır', 'havalan' vb. dediğinde bu aracı çağır.",
        "inputSchema": {"type": "object", "properties": {}},
    },
]


# ----------------------------------------------------------------------
# ARAÇ KATEGORİLERİ (Görev 3): araçları görev alanına göre gruplar.
# ----------------------------------------------------------------------
# tools/list bir 'categories' süzgeci alırsa yalnızca o kategori(ler)deki
# araçlar döner; böylece AI Copilot her sorguda 14 aracın tamamını değil,
# göreve uygun küçük bir alt küme görür (daha kısa istem = daha hızlı yanıt,
# daha az araç karışıklığı). İstemci (mcp_client) yerel yönlendiriciden
# (copilot_router) gelen kategorileri buraya iletir.
#   telemetry : anlık/geçmiş uçuş durumu (irtifa, hız, batarya, konum...)
#   yolo      : YOLO tespitleri + VLM sahne (hedefler, kutular, QR...)
#   analysis  : birleşik durum özeti (telemetri + tespit + VLM tek çağrıda)
#   report    : taktik rapor derleme (GUI kaydetme penceresi açar)
#   flight    : uçağı gerçekten yönlendiren komutlar (dön/ileri/irtifa/hız...)
TOOL_CATEGORIES = {
    "get_telemetry": "telemetry",
    "get_telemetry_history": "telemetry",
    "get_battery_status": "telemetry",
    "get_detections": "yolo",
    "get_detection_history": "yolo",
    "get_vlm_scene": "yolo",
    "get_situation_summary": "analysis",
    "generate_tactical_report": "report",
    "turn_heading": "flight",
    "fly_forward": "flight",
    "return_to_start": "flight",
    "resume_route": "flight",
    "change_speed": "flight",
    "change_altitude": "flight",
    "start_flight": "flight",
}


def _tools_for(categories):
    """İstenen kategori(ler)deki araç tanımlarını döndürür. categories boş/None
    ise tüm araçlar döner (geriye dönük uyum). Bilinmeyen kategori sessizce
    yok sayılır."""
    if not categories:
        return TOOLS
    wanted = set(categories)
    return [t for t in TOOLS if TOOL_CATEGORIES.get(t["name"]) in wanted]


# ----------------------------------------------------------------------
# UÇUŞ KOMUTLARI: MAVROS köprüsü (tembel rclpy düğümü)
# ----------------------------------------------------------------------
# Uçuş komutlarının güvenlik sınırları. Alt irtifa 10 m: rotadaki en yüksek
# cisim 0.6 m + sabit kanat manevra payı; üst 120 m: görev sahası tavanı.
FLIGHT_MIN_ALT_M = 10.0
FLIGHT_MAX_ALT_M = 120.0
FLIGHT_MIN_FWD_M = 50.0     # sabit kanat için daha yakın hedef anlamsız (loiter yarıçapı ~60 m)
FLIGHT_MAX_FWD_M = 2000.0
# Hız sınırları: SITL varsayılan ArduPlane bandı (AIRSPEED_MIN=9 / AIRSPEED_MAX=22);
# alt sınır stall payı için 10'a çekildi. Otopilot bu bandın dışını zaten kırpar.
FLIGHT_MIN_SPEED_MS = 10.0
FLIGHT_MAX_SPEED_MS = 22.0
# Dönüş/irtifa komutlarında uçağın "düz devam" hedefi bu kadar ileriye konur;
# siha_control'un aim-ahead mantığıyla aynı fikir (yakın hedef = erken loiter).
FLIGHT_AIM_AHEAD_M = 3000.0


class FlightCommander:
    """MAVROS üzerinden uçuş komutu gönderen tembel ROS düğümü.

    siha_control.py'nin send_guided_target'ıyla aynı yol: GUIDED moda geç +
    MAV_CMD_DO_REPOSITION (CommandInt). Ek olarak her komutta
    /siha/operator_override yayınlanır: otonom rota izleyicisi bunu görünce
    900 m LOITER kuralını bırakır (her komutta yayın, tek seferlik yayının
    DDS keşif gecikmesine takılıp kaybolmaması içindir)."""

    def __init__(self):
        import rclpy
        from mavros_msgs.srv import CommandInt, CommandLong, SetMode
        from std_msgs.msg import String as RosString
        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._CommandInt = CommandInt
        self._CommandLong = CommandLong
        self._SetMode = SetMode
        self._RosString = RosString
        self.node = rclpy.create_node("mcp_flight_commander")
        self.cmd_int_cli = self.node.create_client(CommandInt, "/mavros/cmd/command_int")
        self.cmd_long_cli = self.node.create_client(CommandLong, "/mavros/cmd/command")
        self.set_mode_cli = self.node.create_client(SetMode, "/mavros/set_mode")
        self.override_pub = self.node.create_publisher(RosString, "/siha/operator_override", 10)

    def _call(self, client, req, timeout_s=5.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"MAVROS servisi bulunamadı: {client.srv_name} "
                               "(simülasyon/MAVROS çalışıyor mu?)")
        future = client.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        if not future.done():
            raise RuntimeError(f"MAVROS servis çağrısı zaman aşımı: {client.srv_name}")
        return future.result()

    def announce_override(self, reason):
        msg = self._RosString()
        msg.data = reason
        self.override_pub.publish(msg)

    def goto(self, lat, lon, alt_m, reason):
        """GUIDED moda geçirip uçağı verilen küresel hedefe yönlendirir."""
        self.announce_override(reason)
        mode_req = self._SetMode.Request()
        mode_req.custom_mode = "GUIDED"
        self._call(self.set_mode_cli, mode_req)
        req = self._CommandInt.Request()
        req.frame = 3        # GLOBAL_RELATIVE_ALT
        req.command = 192    # MAV_CMD_DO_REPOSITION
        req.current = 2      # Guided "goto" hedefi
        req.param1 = -1.0    # hız: varsayılan
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = float("nan")  # yönü ArduPlane hedefe göre hizalasın
        req.x = int(lat * 1e7)
        req.y = int(lon * 1e7)
        req.z = float(alt_m)
        res = self._call(self.cmd_int_cli, req)
        if not res.success:
            raise RuntimeError(f"DO_REPOSITION reddedildi (MAV_RESULT={res.result}).")

    def set_speed(self, speed_ms):
        """Hedef hava hızını değiştirir (MAV_CMD_DO_CHANGE_SPEED). Modu ve
        hedefi değiştirmediği için operator_override yayınlanmaz: otonom rota
        izleme (AUTO/GUIDED fark etmez) yeni hızla sürer."""
        req = self._CommandLong.Request()
        req.command = 178    # MAV_CMD_DO_CHANGE_SPEED
        req.param1 = 0.0     # hız tipi: 0 = hava hızı (airspeed)
        req.param2 = float(speed_ms)
        req.param3 = -1.0    # gaz: otopilot ayarlasın (değişiklik yok)
        res = self._call(self.cmd_long_cli, req)
        if not res.success:
            raise RuntimeError(f"DO_CHANGE_SPEED reddedildi (MAV_RESULT={res.result}).")


_flight_commander = None


def _get_flight_commander():
    global _flight_commander
    if _flight_commander is None:
        _flight_commander = FlightCommander()
    return _flight_commander


def _destination_point(lat_deg, lon_deg, bearing_deg, distance_m):
    """Verilen noktadan bearing yönünde distance kadar ilerideki koordinat
    (siha_control.py ile aynı formül)."""
    radius_m = 6371000.0
    ang = distance_m / radius_m
    bearing = math.radians(bearing_deg)
    lat1 = math.radians(lat_deg)
    lon1 = math.radians(lon_deg)
    lat2 = math.asin(
        math.sin(lat1) * math.cos(ang)
        + math.cos(lat1) * math.sin(ang) * math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(ang) * math.cos(lat1),
        math.cos(ang) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


def _flight_state():
    """Uçuş komutları için snapshot'tan konum/yön/irtifa okur. Snapshot her
    sorgudan hemen önce güncellendiği için saniyelik tazeliktedir."""
    tel = SNAPSHOT.get("telemetry", {})
    lat = tel.get("lat", 0.0)
    lon = tel.get("lon", 0.0)
    if not lat or not lon:
        raise RuntimeError("Uçağın konumu henüz bilinmiyor (GPS/telemetri yok); "
                           "uçuş komutu gönderilemez.")
    heading = float(tel.get("heading", 0.0)) % 360.0
    alt = float(tel.get("alt", 0.0))
    mode = tel.get("mode", "?")
    return lat, lon, heading, alt, mode


def _clamp(value, lo, hi):
    return max(lo, min(hi, float(value)))


# ----------------------------------------------------------------------
# ARAÇ MANTIĞI (snapshot üzerinde; gemini_t.py'den taşındı)
# ----------------------------------------------------------------------
def _haversine_m(lat1, lon1, lat2, lon2):
    """İki koordinat arasındaki yaklaşık yer mesafesi (metre)."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def _minmaxavg(values):
    if not values:
        return None
    return {
        "min": round(min(values), 2),
        "max": round(max(values), 2),
        "avg": round(sum(values) / len(values), 2),
    }


def _telemetry_now():
    tel = SNAPSHOT.get("telemetry", {})
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


def _telemetry_history(window_seconds=None):
    """Kayıtlı telemetri örneklerinden trend/istatistik çıkarır."""
    samples = SNAPSHOT.get("telemetry_history", [])
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
        distance += _haversine_m(a["lat"], a["lon"], b["lat"], b["lon"])

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
        "altitude_m": _minmaxavg(alts),
        "speed_ms": _minmaxavg(speeds),
        "battery_percent": {"start": int(batts[0]), "current": int(batts[-1]),
                            "drop": round(batt_drop, 1)},
        "battery_drain_per_min": drain_rate,
        "distance_traveled_m": round(distance, 1),
        "mode_changes": mode_changes,
        "samples": trail,
    }


def _det_row(d):
    # Video tespitlerinin GPS'i yoktur (lat/lon None) → round None'a çökmesin;
    # koordinat alanları None kalır ve 'source' etiketi kaynağı belli eder.
    lat = d.get("lat")
    lon = d.get("lon")
    return {
        "id": d.get("id"),
        "type": d.get("type"),
        "peak_conf": round(d.get("conf", 0.0), 2),
        "first_seen": d.get("first_time"),
        "last_seen": d.get("time"),
        "frames_confirmed": d.get("hits", 1),
        "lat": round(lat, 5) if lat is not None else None,
        "lon": round(lon, 5) if lon is not None else None,
        "alt_m": round(d["alt"], 1) if d.get("alt") is not None else None,
        "qr_text": d.get("qr_text"),
        # "video" = drone.mp4 üzerinde tespit (GPS yok); yoksa uçuş tespiti.
        "source": d.get("source", "flight"),
    }


def _detections_live():
    dets = SNAPSHOT.get("detections", [])
    active = SNAPSHOT.get("active_detections", [])
    counts = {}
    for d in dets:
        counts[d.get("type", "?")] = counts.get(d.get("type", "?"), 0) + 1
    return {
        "active_now": [_det_row(d) for d in active],
        "active_count": len(active),
        "table_total": len(dets),
        "counts_by_type": counts,
        "last_detection": _det_row(dets[-1]) if dets else None,
        "targets": [_det_row(d) for d in dets[-10:]],
    }


def _detections_history():
    log = SNAPSHOT.get("detection_log", [])
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
        "confidence": _minmaxavg(confs),
        "first_detection_time": log[0].get("first_time"),
        "last_detection_time": log[-1].get("time"),
        # Kronolojik (ilk görülme sırasına göre) tüm hedefler.
        "targets": [_det_row(d) for d in log],
    }


def execute_tool(name, args):
    tel = SNAPSHOT.get("telemetry", {})
    vlm = SNAPSHOT.get("vlm_summary", "")

    if name == "get_telemetry":
        return _telemetry_now()

    if name == "get_telemetry_history":
        return _telemetry_history(args.get("window_seconds"))

    if name == "get_battery_status":
        batt = tel.get("battery", 0)
        return {
            "battery_percent": int(batt),
            "voltage_v": tel.get("voltage", 0.0),
            "assessment": "kritik" if batt < 20 else ("düşük" if batt < 40 else "normal"),
        }

    if name == "get_detections":
        return _detections_live()

    if name == "get_detection_history":
        return _detections_history()

    if name == "get_vlm_scene":
        return {"scene_description": vlm or "Henüz sahne açıklaması üretilmedi."}

    if name == "get_situation_summary":
        return {
            "mission_time_s": round(SNAPSHOT.get("mission_time_s", 0.0), 1),
            "telemetry_now": _telemetry_now(),
            "telemetry_history": _telemetry_history(),
            "detections_live": _detections_live(),
            "detections_history": _detections_history(),
            "vlm_scene": vlm or "Henüz sahne açıklaması üretilmedi.",
        }

    if name == "turn_heading":
        degrees = float(args.get("degrees", 0.0))
        # -180..180'e normalle (540 -> 180 gibi girişler için).
        degrees = ((degrees + 180.0) % 360.0) - 180.0
        if abs(degrees) < 1.0:
            return {"error": "Dönüş açısı verilmedi (degrees ~0). Sağa dönüş için "
                             "pozitif, sola için negatif derece gönder."}
        lat, lon, heading, alt, mode = _flight_state()
        alt = _clamp(alt, FLIGHT_MIN_ALT_M, FLIGHT_MAX_ALT_M)
        new_heading = (heading + degrees) % 360.0
        tgt_lat, tgt_lon = _destination_point(lat, lon, new_heading, FLIGHT_AIM_AHEAD_M)
        yon = "sağa" if degrees > 0 else "sola"
        _get_flight_commander().goto(tgt_lat, tgt_lon, alt,
                                     f"turn_heading {degrees:+.0f}")
        return {
            "status": f"Komut gönderildi: {abs(degrees):.0f}° {yon} dönüş. Uçak "
                      f"{new_heading:.0f}° yönünde düz uçuşa devam edecek "
                      f"(irtifa {alt:.0f} m korunuyor). Otonom rota izleme "
                      "devre dışı bırakıldı.",
            "previous_heading_deg": round(heading, 1),
            "new_heading_deg": round(new_heading, 1),
            "altitude_m": round(alt, 1),
            "previous_mode": mode,
            "ui_action": "timeline_event",
            "ui_payload": {"label": f"AI Uçuş Komutu: {abs(degrees):.0f}° {yon} dönüş"},
        }

    if name == "fly_forward":
        distance = _clamp(args.get("distance_m", 0.0), FLIGHT_MIN_FWD_M, FLIGHT_MAX_FWD_M)
        lat, lon, heading, alt, mode = _flight_state()
        alt = _clamp(alt, FLIGHT_MIN_ALT_M, FLIGHT_MAX_ALT_M)
        tgt_lat, tgt_lon = _destination_point(lat, lon, heading, distance)
        _get_flight_commander().goto(tgt_lat, tgt_lon, alt,
                                     f"fly_forward {distance:.0f}m")
        note = ""
        if float(args.get("distance_m", distance)) != distance:
            note = (f" (istenen mesafe güvenlik sınırına sıkıştırıldı: "
                    f"{FLIGHT_MIN_FWD_M:.0f}-{FLIGHT_MAX_FWD_M:.0f} m)")
        return {
            "status": f"Komut gönderildi: mevcut yönde ({heading:.0f}°) {distance:.0f} m "
                      f"ileri.{note} Sabit kanatlı uçak hedefe varınca o noktada LOITER "
                      "(daire) çizerek bekleyecek. Otonom rota izleme devre dışı.",
            "heading_deg": round(heading, 1),
            "distance_m": round(distance, 1),
            "target_lat": round(tgt_lat, 6),
            "target_lon": round(tgt_lon, 6),
            "altitude_m": round(alt, 1),
            "previous_mode": mode,
            "ui_action": "timeline_event",
            "ui_payload": {"label": f"AI Uçuş Komutu: {distance:.0f} m ileri"},
        }

    if name == "change_speed":
        current = float(tel.get("speed", 0.0))
        if args.get("target_speed_ms") is not None:
            requested = float(args["target_speed_ms"])
        elif args.get("change_ms") is not None:
            # Taban telemetri yer hızıdır (hava hızı sensörü yok); rüzgârsız
            # SITL'de ikisi ~eşittir, fark operatöre yansıtılmaz.
            requested = current + float(args["change_ms"])
        else:
            return {"error": "target_speed_ms veya change_ms parametrelerinden biri gerekli."}
        new_speed = _clamp(requested, FLIGHT_MIN_SPEED_MS, FLIGHT_MAX_SPEED_MS)
        _get_flight_commander().set_speed(new_speed)
        clamp_note = ""
        if requested != new_speed:
            clamp_note = (f" (istenen {requested:.0f} m/s güvenlik aralığına sıkıştırıldı: "
                          f"{FLIGHT_MIN_SPEED_MS:.0f}-{FLIGHT_MAX_SPEED_MS:.0f} m/s)")
        return {
            "status": f"Komut gönderildi: hedef hava hızı {new_speed:.0f} m/s "
                      f"(mevcut ~{current:.0f} m/s).{clamp_note} Rota ve mod "
                      "değişmedi; uçak mevcut hedefine yeni hızla devam ediyor.",
            "previous_speed_ms": round(current, 1),
            "new_speed_ms": round(new_speed, 1),
            "ui_action": "timeline_event",
            "ui_payload": {"label": f"AI Uçuş Komutu: hız {new_speed:.0f} m/s"},
        }

    if name == "change_altitude":
        lat, lon, heading, alt, mode = _flight_state()
        if args.get("target_altitude_m") is not None:
            requested = float(args["target_altitude_m"])
        elif args.get("change_m") is not None:
            requested = alt + float(args["change_m"])
        else:
            return {"error": "target_altitude_m veya change_m parametrelerinden biri gerekli."}
        new_alt = _clamp(requested, FLIGHT_MIN_ALT_M, FLIGHT_MAX_ALT_M)
        # Uçak mevcut yönünde düz uçmaya devam ederken tırmansın/alçalsın.
        tgt_lat, tgt_lon = _destination_point(lat, lon, heading, FLIGHT_AIM_AHEAD_M)
        _get_flight_commander().goto(tgt_lat, tgt_lon, new_alt,
                                     f"change_altitude {new_alt:.0f}m")
        clamp_note = ""
        if requested != new_alt:
            clamp_note = (f" (istenen {requested:.0f} m güvenlik aralığına sıkıştırıldı: "
                          f"{FLIGHT_MIN_ALT_M:.0f}-{FLIGHT_MAX_ALT_M:.0f} m)")
        verb = "tırmanacak" if new_alt > alt else ("alçalacak" if new_alt < alt else "korunacak")
        return {
            "status": f"Komut gönderildi: irtifa {alt:.0f} m -> {new_alt:.0f} m, uçak "
                      f"{heading:.0f}° yönünde düz uçarken {verb}.{clamp_note} "
                      "Otonom rota izleme devre dışı.",
            "previous_altitude_m": round(alt, 1),
            "new_altitude_m": round(new_alt, 1),
            "heading_deg": round(heading, 1),
            "previous_mode": mode,
            "ui_action": "timeline_event",
            "ui_payload": {"label": f"AI Uçuş Komutu: irtifa {new_alt:.0f} m"},
        }

    if name == "return_to_start":
        lat, lon, heading, alt, mode = _flight_state()
        # Başlangıç noktası: GUI snapshot'ı sabit kalkış konumunu 'home' olarak
        # gönderir; yoksa telemetri geçmişindeki ilk geçerli GPS örneğine düşülür
        # (uçak kalkışta home üzerindedir, ilk örnek ~home'dur).
        home = SNAPSHOT.get("home") or {}
        h_lat, h_lon = home.get("lat"), home.get("lon")
        if not h_lat or not h_lon:
            for s in SNAPSHOT.get("telemetry_history", []):
                if abs(s.get("lat", 0.0)) > 0.01 and abs(s.get("lon", 0.0)) > 0.01:
                    h_lat, h_lon = s["lat"], s["lon"]
                    break
        if not h_lat or not h_lon:
            return {"error": "Başlangıç (kalkış) noktası bilinmiyor: snapshot'ta home "
                             "yok ve telemetri geçmişinde geçerli GPS örneği bulunamadı."}
        alt = _clamp(alt, FLIGHT_MIN_ALT_M, FLIGHT_MAX_ALT_M)
        dist = _haversine_m(lat, lon, h_lat, h_lon)
        _get_flight_commander().goto(h_lat, h_lon, alt, "return_to_start")
        return {
            "status": f"Komut gönderildi: uçak ~{dist:.0f} m uzaklıktaki başlangıç "
                      f"(kalkış) noktasına dönüyor (irtifa {alt:.0f} m korunuyor). "
                      "Sabit kanatlı uçak varınca o noktanın üzerinde LOITER (daire) "
                      "çizerek bekleyecek. Otonom rota izleme askıda.",
            "home_lat": round(h_lat, 6),
            "home_lon": round(h_lon, 6),
            "distance_to_home_m": round(dist, 1),
            "altitude_m": round(alt, 1),
            "previous_mode": mode,
            "ui_action": "timeline_event",
            "ui_payload": {"label": f"AI Uçuş Komutu: başlangıç noktasına dönüş (~{dist:.0f} m)"},
        }

    if name == "resume_route":
        # Devam sinyali otonom rota izleyicisine (siha_control) gider; hedefi
        # ve mod geçişini o hesaplar (home konumunu yalnızca o bilir). İki kez
        # yayın: ilk yayın DDS keşif gecikmesine denk gelirse kaybolmasın.
        fc = _get_flight_commander()
        fc.announce_override("resume_route")
        time.sleep(0.5)
        fc.announce_override("resume_route")
        return {
            "status": "Otonom rotaya dönüş sinyali gönderildi. Rota izleyici uçağı "
                      "GUIDED modda orijinal kuzey rotasına yöneltecek ve 900 m görev "
                      "kuralı kaldığı yerden devam edecek. (Not: görev 900 m'yi zaten "
                      "tamamladıysa uçak LOITER'da bekler, rota yeniden başlamaz.)",
            "ui_action": "timeline_event",
            "ui_payload": {"label": "AI Uçuş Komutu: otonom rotaya dönüş"},
        }

    if name == "generate_tactical_report":
        # Rapor penceresi bir GUI eylemidir; sunucu GUI'ye dokunamaz. 'ui_action'
        # işareti istemci tarafında (GeminiChatThread) yakalanır ve ana thread'e
        # sinyalle iletilir. Aynı sorguda tekrar tetiklenmeme kontrolü de
        # istemcidedir (sunucu sorgu sınırlarını bilmez).
        return {
            "ui_action": "download_report",
            "status": "Rapor derleyici tetiklendi; kaydetme penceresi açılıyor.",
            "report_contents": {
                "telemetry_now": _telemetry_now(),
                "telemetry_history": _telemetry_history(),
                "detections_history": _detections_history(),
                "vlm_scene": vlm or "Henüz sahne açıklaması üretilmedi.",
            },
        }

    if name == "start_flight":
        return {
            "status": "Otonom uçuş ve otomatik pist kalkış komutu tetiklendi.",
            "ui_action": "start_flight",
            "ui_payload": {"label": "AI Uçuş Komutu: Uçuşa Geç / Kalkış"},
        }

    raise KeyError(f"Bilinmeyen araç: {name}")


# ----------------------------------------------------------------------
# JSON-RPC 2.0 / MCP protokol katmanı
# ----------------------------------------------------------------------
def _response(req_id, result):
    return {"jsonrpc": "2.0", "id": req_id, "result": result}


def _error(req_id, code, message):
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": message}}


def handle_message(msg):
    """Tek JSON-RPC mesajını işler; yanıt sözlüğü veya None (bildirim) döndürür."""
    method = msg.get("method", "")
    params = msg.get("params", {}) or {}
    req_id = msg.get("id")

    # --- Bildirimler (id yok, yanıt yazılmaz) ---
    if req_id is None:
        if method == "notifications/snapshot":
            # İstemci her sorgudan önce güncel GUI durumunu gönderir.
            SNAPSHOT.clear()
            SNAPSHOT.update(params.get("snapshot", {}) or {})
        # notifications/initialized dahil diğer bildirimler sessizce yutulur.
        return None

    # --- İstekler ---
    if method == "initialize":
        return _response(req_id, {
            "protocolVersion": PROTOCOL_VERSION,
            "capabilities": {"tools": {}},
            "serverInfo": SERVER_INFO,
        })

    if method == "ping":
        return _response(req_id, {})

    if method == "tools/list":
        # İstemci 'categories' verirse araçları o alana daralt (Görev 3);
        # vermezse tüm araçlar döner.
        return _response(req_id, {"tools": _tools_for(params.get("categories"))})

    if method == "tools/call":
        name = params.get("name", "")
        args = params.get("arguments", {}) or {}
        try:
            result = execute_tool(name, args)
        except KeyError as exc:
            return _error(req_id, -32602, str(exc))
        except Exception as exc:
            # Araç hatası MCP'de protokol hatası DEĞİLDİR; isError ile taşınır.
            return _response(req_id, {
                "content": [{"type": "text", "text": f"Araç hatası: {exc}"}],
                "isError": True,
            })
        return _response(req_id, {
            "content": [{"type": "text",
                         "text": json.dumps(result, ensure_ascii=False)}],
            "isError": False,
        })

    return _error(req_id, -32601, f"Bilinmeyen metot: {method}")


def main():
    print("MCP sunucusu hazır (siha-gcs-tools, stdio).", file=sys.stderr, flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as exc:
            print(json.dumps(_error(None, -32700, f"Parse hatası: {exc}")),
                  flush=True)
            continue
        try:
            reply = handle_message(msg)
        except Exception as exc:
            reply = _error(msg.get("id"), -32603, f"İç hata: {exc}")
        if reply is not None:
            print(json.dumps(reply, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
