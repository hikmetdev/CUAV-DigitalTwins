import cv2
import numpy as np
import os
import time
import json
import math
from datetime import datetime, timezone, timedelta
from typing import List, Literal, Optional, Tuple

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_LOG_DIR = os.environ.get("ROS_LOG_DIR", os.path.join(BASE_DIR, "logs", "ros"))
os.makedirs(ROS_LOG_DIR, exist_ok=True)
os.environ.setdefault("ROS_LOG_DIR", ROS_LOG_DIR)
os.environ.setdefault("MPLCONFIGDIR", os.path.join(BASE_DIR, "logs", "matplotlib"))
os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ROSImage, NavSatFix
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from std_msgs.msg import String
from ultralytics import YOLOE
import torch
from pydantic import BaseModel, Field, computed_field

# Güven eşiği: Gazebo'daki küçük/sentetik cisimlerde YOLOE güveni tipik olarak
# 0.10-0.40 arası kaldığı için yüksek eşik (0.50) hiçbir tespit geçirmiyordu →
# kamera üzerine kutu çizilmiyordu. Bu yüzden varsayılan düşük tutulur; yanlış
# pozitif fazlaysa ortam değişkeniyle yükseltilebilir (ör. YOLO_CONF_THRESHOLD=0.35).
# Gösterim/log eşiği 0.10: log verisi (build_141626) traktörün 'truck:0.12',
# turuncu kutunun 'box:0.06' ürettiğini gösterdi. 0.15 traktörü 0.03 farkla
# eliyordu; 0.10 traktörü geçirir, QR kutu (0.50) ve insan (~0.33) zaten üstünde.
# Bu ortamda 0.10 üstü sahte pozitif yok (sadece başlangıçtaki car:0.27, o da
# 150m bastırma bölgesinde). Gerçek uçuşta YOLO_CONF_THRESHOLD ile yükseltilebilir.
CONFIDENCE_THRESHOLD = float(os.environ.get("YOLO_CONF_THRESHOLD", "0.10"))
# Kullanılacak tracker konfigürasyonu (Ultralytics ByteTrack varsayılan).
TRACKER_NAME = os.environ.get("YOLO_TRACKER", "bytetrack.yaml")
# Türkiye saati (UTC+3) — log zaman damgaları için.
LOCAL_TZ = timezone(timedelta(hours=3))

# ---- CPU/GPU iş paylaşımı (8 GB RAM / GTX 1650 gibi sınırlı sistemler için) ----
# GPU varsa çıkarım GTX 1650'ye alınır: CPU Gazebo fiziği + ArduPilot + GUI'ye
# kalır, model ağırlıkları sistem RAM'i yerine VRAM'de tutulur (RAM baskısı azalır).
# GPU yoksa CPU'da çalışılır ama tüm çekirdekler kaplanmaz; aksi halde Gazebo ve
# MAVROS/EKF init'i aç kalıp uygulama takılır veya OOM ile çöker.
if torch.cuda.is_available():
    YOLO_DEVICE = os.environ.get("YOLO_DEVICE", "0")
    YOLO_HALF = os.environ.get("YOLO_HALF", "1") == "1"
else:
    YOLO_DEVICE = "cpu"
    YOLO_HALF = False
    torch.set_num_threads(int(os.environ.get("YOLO_CPU_THREADS", "4")))

# Çıkarım kare hızını sınırla: her kameraya gelen kareyi işlemek CPU'yu boğar ve
# annotated yayını gereksiz sıklıkta üretir. Varsayılan ~4 FPS operatör için yeterli.
YOLO_MAX_FPS = float(os.environ.get("YOLO_MAX_FPS", "4"))
# Çıkarım çözünürlüğü (düşük = daha az CPU/VRAM). Kamera 320x240 native; imgsz'i
# 416'ya büyütmek stride grid'ine her cisim üzerinde daha çok hücre verir →
# ~10-25 px'lik havadan cisimlerde recall/güven belirgin artar (CPU ~1.7x).
YOLO_IMGSZ = int(os.environ.get("YOLO_IMGSZ", "416"))
# Tanılama tabanı: çıkarım bu düşük eşikle çalışır ki eşik-altı adaylar da
# görülebilsin (ör. traktör 'truck:0.08'). Gösterim/log için yine dinamik eşik
# (effective_conf) uygulanır. Bir cismin neden hiç ateşlemediğini veriyle anlamak
# için "bu aralıkta tespit yok" satırına en iyi eşik-altı adaylar eklenir.
YOLO_RAW_CONF = float(os.environ.get("YOLO_RAW_CONF", "0.05"))
# Sahte pozitif filtresi: 15 m irtifada gerçek cisim karenin küçük bir kısmını
# kaplar (4 m kutu ~%3-22). Zemini/gölgeyi/pist metnini 'car'/'box' sanan
# tespitlerin bbox'ı ise kareyi doldurur (~%99). Kare alanının bu oranından
# büyük kutuları at → tam-kare sahte pozitifler elenir, gerçek cisimler kalır.
YOLO_MAX_BBOX_FRAC = float(os.environ.get("YOLO_MAX_BBOX_FRAC", "0.40"))
# NMS IoU eşiği (çakışan kutuları birleştirir). claude2.md 0.50 önerdi; küçük/bitişik
# cisimlerin ayrı kalması için ortam değişkeniyle ayarlanabilir tutulur.
YOLO_IOU_THRESHOLD = float(os.environ.get("YOLO_IOU_THRESHOLD", "0.50"))
# Görüntü ön işleme modu: none | clahe | gamma | sharpen | clahe_gamma.
# Geriye dönük uyumluluk: eski YOLO_CLAHE=1 varsayılanını korumak için varsayılan
# "clahe". YOLO_PREPROCESS açıkça verilirse o kazanır.
YOLO_PREPROCESS = os.environ.get("YOLO_PREPROCESS", "clahe").lower()
YOLO_GAMMA = float(os.environ.get("YOLO_GAMMA", "1.2"))
# Tiling (multi-scale): tam kare az/hiç tespit verirse kareyi NxN parçaya bölüp
# her parçada ayrı çıkarım yapar; küçük/uzak cisimlerde recall'ı artırır. CPU/VRAM
# maliyeti grid^2 katına çıktığı için (8GB RAM/GTX1650'de OOM riski) varsayılan KAPALI.
YOLO_ENABLE_TILING = os.environ.get("YOLO_ENABLE_TILING", "0") == "1"
YOLO_TILE_GRID = max(1, int(os.environ.get("YOLO_TILE_GRID", "2")))
# Tam karede tespit sayısı bunun altındaysa tiling devreye girer.
YOLO_TILE_MIN_DETS = int(os.environ.get("YOLO_TILE_MIN_DETS", "1"))
# Temporal smoothing: bir hedef en az bu kadar kare görülmeden tabloya/loga
# geçmez (tek karelik sahte pozitifleri eler). Varsayılan 1 = mevcut davranış
# (bastırma yok); bu ortamda sorun kaçırılan tespitler olduğu için düşük tutulur.
YOLO_MIN_HITS = max(1, int(os.environ.get("YOLO_MIN_HITS", "1")))


# Benzer sınıfları tek hedef kategorisine normalize eder: aynı cisim bir karede
# "box", diğerinde "crate" gelirse iki farklı hedef gibi sayılmasın. Ham etiket
# (raw label) log ve payload'da ayrıca korunur.
CLASS_NORMALIZATION = {
    "person": "person",
    "human": "person",
    "pedestrian": "person",
    "walking person": "person",
    "car": "vehicle",
    "truck": "vehicle",
    "semi-truck": "vehicle",
    "pickup truck": "vehicle",
    "van": "vehicle",
    "vehicle": "vehicle",
    "tractor": "vehicle",
    "box": "cargo_box",
    "crate": "cargo_box",
    "cargo box": "cargo_box",
    "orange box": "cargo_box",
    "qr box": "cargo_box",
    "qr code box": "cargo_box",
    "blue cargo box": "cargo_box",
    "package": "cargo_box",
    "drone": "drone",
    "quadcopter": "drone",
    "quadrotor": "drone",
    "uav": "drone",
    "backpack": "backpack",
    "rucksack": "backpack",
    "knapsack": "backpack",
}


def normalize_category(label: str) -> str:
    """Ham sınıf etiketini normalize kategoriye çevirir; bilinmeyen etiket kendisi kalır."""
    return CLASS_NORMALIZATION.get(str(label).lower(), str(label).lower())


# ----------------------------------------------------------------------
# Pydantic tespit log modelleri (yolo.log içine JSONL olarak yazılır)
# ----------------------------------------------------------------------
class DetectionObjectLog(BaseModel):
    label: str = Field(..., description="YOLO tarafindan bulunan ham nesne sinifi")
    normalized_category: str = Field(..., description="Normalize edilmis hedef kategorisi")
    count: int = Field(default=1, ge=1)
    confidence: float = Field(..., ge=0.0, le=1.0)


class YoloDetectionLog(BaseModel):
    schema_version: str = "1.2"
    event_type: Literal["object_detection"] = "object_detection"
    timestamp: datetime
    local_time: str
    uav_altitude_m: Optional[float] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    object: DetectionObjectLog
    track_id: Optional[int] = None
    bbox_xyxy: Optional[List[float]] = None
    hits: int = Field(default=1, ge=1)
    tracker_name: str = "bytetrack"
    source_topic: str
    frame_id: int
    confidence_threshold: Optional[float] = None
    inference_imgsz: Optional[int] = None
    preprocess_mode: Optional[str] = None
    tiling_enabled: Optional[bool] = None

    @computed_field
    @property
    def message(self) -> str:
        confidence_percent = round(self.object.confidence * 100)
        if self.uav_altitude_m is not None:
            alt_part = f"İHA {self.uav_altitude_m:.0f}m irtifadayken, "
        else:
            alt_part = "İHA irtifası bilinmezken, "
        if self.latitude is not None and self.longitude is not None:
            coord_part = f"[{self.latitude:.6f}, {self.longitude:.6f}] koordinatında"
        else:
            coord_part = "koordinatı henüz alınamadan"
        cat_part = f" (kategori: {self.object.normalized_category})"
        if self.track_id is not None:
            obj_part = (
                f"ID: {self.track_id} '{self.object.label}'{cat_part} "
                f"%{confidence_percent} güvenle tespit edildi."
            )
        else:
            obj_part = (
                f"bir adet '{self.object.label}'{cat_part} "
                f"%{confidence_percent} güvenle tespit edildi."
            )
        hits_part = f" Hedef {self.hits} karede doğrulandı." if self.hits > 1 else ""
        return f"Saat {self.local_time}'te, {alt_part}{coord_part} {obj_part}{hits_part}"


class GazeboYoloNode(Node):
    def __init__(self, model_name="yoloe-26s-seg.pt", topic="/bottom_camera/image",
                 annotated_topic="/yolo/image_annotated", classes=["human", "car", "box"]):
        super().__init__("gazebo_yolo_detector")
        self.get_logger().info(f"Model yükleniyor: {model_name}...")
        self.model = YOLOE(model_name)
        self.get_logger().info(f"Hedef sınıflar ayarlanıyor: {classes}")
        self.model.set_classes(classes)
        self.get_logger().info(
            f"Güven eşiği (CONFIDENCE_THRESHOLD) = {CONFIDENCE_THRESHOLD}; "
            f"tracker = {TRACKER_NAME}"
        )
        self.bridge = CvBridge()
        self.topic = topic
        self.last_log_time = 0.0
        self.last_heartbeat = 0.0
        self.last_subthresh_log = 0.0
        self.last_infer_time = 0.0
        self.min_infer_interval = (1.0 / YOLO_MAX_FPS) if YOLO_MAX_FPS > 0 else 0.0
        self.frame_count = 0
        # Tracking açık başlar; ByteTrack bu ortamda çalışmazsa detection'a düşer.
        self.use_tracking = True
        self.get_logger().info(
            f"Çıkarım cihazı: {YOLO_DEVICE} (half={YOLO_HALF}), "
            f"maks {YOLO_MAX_FPS:.0f} FPS, imgsz={YOLO_IMGSZ}, "
            f"CPU thread={torch.get_num_threads()}"
        )

        # Görüntü ön işleme (preprocess) kurulumu.
        # Modlar: none | clahe | gamma | sharpen | clahe_gamma.
        # Geriye dönük uyumluluk: eski YOLO_CLAHE=0 clahe içeren modları kapatır.
        self.preprocess_mode = YOLO_PREPROCESS
        if os.environ.get("YOLO_CLAHE", "1") == "0" and "clahe" in self.preprocess_mode:
            self.preprocess_mode = "none"
        self.use_clahe = "clahe" in self.preprocess_mode  # geriye dönük referanslar için
        clahe_clip = float(os.environ.get("YOLO_CLAHE_CLIP", "2.0"))
        clahe_tile = int(os.environ.get("YOLO_CLAHE_TILE", "8"))
        self.clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))
        # Gamma LUT'unu önceden hesapla (her karede yeniden üretmemek için).
        inv_gamma = 1.0 / max(0.01, YOLO_GAMMA)
        self._gamma_lut = np.array(
            [((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype=np.uint8
        )
        self.get_logger().info(
            f"Ön işleme modu: {self.preprocess_mode} "
            f"(clipLimit={clahe_clip}, tile=({clahe_tile},{clahe_tile}), gamma={YOLO_GAMMA})"
        )

        # Tespit kalitesi metrikleri (periyodik YOLO_METRICS satırı için).
        self.metrics = {"raw": 0, "filtered": 0, "tracked": 0, "missed_frames": 0}
        self.last_metrics_log = 0.0
        self.metrics_frames = 0
        # track_id -> art arda görülme sayısı (hits). Temporal smoothing / min_hits.
        self.track_hits = {}

        # Dinamik Eşik parametreleri.
        # NOT: IHA bu görevde ~15 m'de seyrediyor (alt_low'un altında), yani tüm
        # uçuş "alçak irtifa" sayılıp conf_low_alt uygulanıyor. Gazebo'daki sentetik
        # cisimlerde YOLOE güveni ~0.25-0.45 arası kaldığı için önceki 0.65/0.35
        # değerleri HER tespiti eliyordu (bkz. build_20260716_105737: 246 kare, 0
        # tespit). Bu yüzden Gazebo için eşikler düşük tutulur; gerçek yüksek-irtifa
        # uçuşunda YOLO_CONF_LOW_ALT/HIGH_ALT env ile 0.65/0.35'e yükseltilebilir.
        self.use_dynamic_conf = int(os.environ.get("YOLO_DYNAMIC_CONF", "1")) == 1
        self.alt_high = float(os.environ.get("YOLO_ALT_HIGH", "25.0"))
        self.alt_low = float(os.environ.get("YOLO_ALT_LOW", "12.0"))
        self.conf_high_alt = float(os.environ.get("YOLO_CONF_HIGH_ALT", "0.15"))
        self.conf_low_alt = float(os.environ.get("YOLO_CONF_LOW_ALT", "0.10"))
        self.person_conf_min = float(os.environ.get("YOLO_PERSON_CONF_MIN", "0.05"))
        self.last_status_log_time = 0.0
        
        # Her nesneye benzersiz ID atamak için takipçi ve fallback ID sayacı
        self.next_fallback_id = 1000
        self.active_fallback_tracks = []

        if self.use_dynamic_conf:
            self.get_logger().info(
                f"Dinamik eşik aktif: AltHigh={self.alt_high}m (Conf={self.conf_high_alt}), "
                f"AltLow={self.alt_low}m (Conf={self.conf_low_alt})"
            )
        else:
            self.get_logger().info("Dinamik eşik devre dışı, statik eşik kullanılacak.")

        # Telemetri (yolo.log kayıtlarını zenginleştirmek için MAVROS'tan okunur).
        self.latitude = None
        self.longitude = None
        self.altitude = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5
        )
        # Görüntü aboneliği için ayrı QoS: depth=1. Çıkarım ~250ms sürerken
        # depth=5 ile 5 kare (0.5s) kuyrukta birikip bayat kare işleniyordu →
        # cisim kadraj ortasındayken değil, arka kenardan çıkarken tespit
        # ediliyordu (geç tespit). depth=1 her zaman EN TAZE kareyi işletir.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        self.subscription = self.create_subscription(
            ROSImage,
            self.topic,
            self.image_callback,
            image_qos
        )
        # Telemetri abonelikleri: tespit kaydına koordinat/irtifa eklemek için.
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_cb, qos_profile)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos_profile)
        # Tespit kutuları çizilmiş görüntüyü GUI'nin izleyebilmesi için yayınla.
        self.annotated_pub = self.create_publisher(ROSImage, annotated_topic, qos_profile)
        # Tespit sonuçlarını (sınıf + güven + track_id) GUI tablo/timeline için JSON yayınla.
        self.detections_pub = self.create_publisher(String, "/yolo/detections", qos_profile)
        self.get_logger().info(
            f"{self.topic} dinleniyor; kutulanmış akış {annotated_topic}, "
            f"tespitler /yolo/detections adresine yayınlanacak."
        )
        # Eşik değerini yolo.log başlangıcında JSONL kaydıyla görünür kıl.
        print(json.dumps({
            "event_type": "startup",
            "confidence_threshold": CONFIDENCE_THRESHOLD,
            "iou_threshold": YOLO_IOU_THRESHOLD,
            "imgsz": YOLO_IMGSZ,
            "tracker_name": TRACKER_NAME,
            "classes": classes,
            "preprocess_mode": self.preprocess_mode,
            "use_dynamic_conf": self.use_dynamic_conf,
            "tiling_enabled": YOLO_ENABLE_TILING,
            "tile_grid": YOLO_TILE_GRID,
            "min_hits": YOLO_MIN_HITS,
        }), flush=True)

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
            return 0.0
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        return 6371000.0 * 2.0 * math.asin(math.sqrt(a))

    def gps_cb(self, msg):
        if (msg.latitude != 0.0 and msg.longitude != 0.0
                and not math.isnan(msg.latitude) and not math.isnan(msg.longitude)):
            self.latitude = float(msg.latitude)
            self.longitude = float(msg.longitude)

    def pose_cb(self, msg):
        self.altitude = float(msg.pose.position.z)

    def image_callback(self, msg):
        # Kapanış sırasında (SIGTERM) ROS bağlamı geçersizken callback'e girme.
        if not rclpy.ok():
            return
        # Çıkarımı hedef kare hızına kilitle: fazla kareleri işlemeden düşür.
        if self.min_infer_interval > 0.0:
            t = time.time()
            if t - self.last_infer_time < self.min_infer_interval:
                return
            self.last_infer_time = t

        effective_conf = self.dynamic_conf()

        # Saniyede bir irtifa ve eşik değerini logla (hem stdout/yolo.log hem ROS logu)
        now = time.time()
        if now - self.last_status_log_time > 1.0:
            self.get_logger().info(
                f"İrtifa: {f'{self.altitude:.2f}m' if self.altitude is not None else 'Bilinmiyor'}, "
                f"Eşik: {effective_conf:.3f} (Dinamik: {self.use_dynamic_conf})"
            )
            try:
                status_log = {
                    "event_type": "status",
                    "timestamp": datetime.now(LOCAL_TZ).isoformat(),
                    "uav_altitude_m": self.altitude,
                    "confidence_threshold": effective_conf,
                }
                print(json.dumps(status_log), flush=True)
            except Exception as e:
                self.get_logger().warn(f"Eşik loglama hatası: {e}")
            self.last_status_log_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Ön işleme (none/clahe/gamma/sharpen/clahe_gamma). İnference işlenmiş
            # kare üzerinden; gösterim için de aynı kareyi kullanırız (annotated bunun
            # üstüne çizilir), böylece kutular gösterilen görüntüyle hizalı kalır.
            frame = self.preprocess_frame(frame)

            # Çıkarımı düşük tanılama tabanında (raw_conf) yap: zayıf adaylar da
            # gelsin, gösterim/log için aşağıda effective_conf uygulanır.
            raw_conf = min(effective_conf, YOLO_RAW_CONF)
            results = self.run_inference(frame, raw_conf)
        except Exception as exc:
            self.get_logger().warn(f"YOLO görüntü işleme hatası: {exc}")
            return
        if results is None:
            return

        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info("İlk kamera karesi alındı; tespit çalışıyor.")

        # Eşik-altı adayları tanılama için topla (filtrelemeden önce). Bir cisim
        # gösterim eşiğini geçemese bile modelin ona ne güven verdiğini görürüz.
        raw_candidates = []
        if len(results) > 0 and results[0].boxes is not None:
            try:
                names = results[0].names
                for box in results[0].boxes:
                    c = float(box.conf[0])
                    lbl = names.get(int(box.cls[0]), str(int(box.cls[0])))
                    raw_candidates.append((lbl, c))
                raw_candidates.sort(key=lambda x: x[1], reverse=True)
            except Exception:
                raw_candidates = []

        # Filter detections in the Results object before plotting
        frame_area = float(frame.shape[0] * frame.shape[1])
        if len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            try:
                keep_indices = []
                for idx, box in enumerate(results[0].boxes):
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = names.get(class_id, str(class_id))
                    norm_cat = normalize_category(class_name)

                    # Tam-kare sahte pozitif filtresi
                    try:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        bbox_area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
                        if frame_area > 0 and bbox_area > YOLO_MAX_BBOX_FRAC * frame_area:
                            continue
                    except Exception:
                        pass
                    if self.latitude is not None and self.longitude is not None:
                        dist_to_start = self.distance_m(39.920782, 32.854115, self.latitude, self.longitude)
                        if dist_to_start < 150.0:
                            continue

                    # Truck vs Cargo Box Çakışma Düzeltmesi (Uzun gövdeli araçların kutu olarak sanılmasını önler)
                    try:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        bw, bh = abs(x2 - x1), abs(y2 - y1)
                        aspect_ratio = max(bw, bh) / (min(bw, bh) + 1e-5)
                        if norm_cat == "cargo_box" and aspect_ratio > 1.25 and max(bw, bh) > 50:
                            for raw_lbl, raw_conf in raw_candidates:
                                if normalize_category(raw_lbl) == "vehicle" and raw_conf >= 0.05:
                                    class_name = raw_lbl
                                    box.cls[0] = next((k for k, v in names.items() if v == raw_lbl), box.cls[0])
                                    norm_cat = "vehicle"
                                    break
                    except Exception:
                        pass

                    # İnsanlar tepe perspektifinde ~0.05-0.08 güven ürettiği için özel düşük eşik
                    is_person = (norm_cat == "person" or class_name.lower() in ["person", "human", "pedestrian", "walking person"])
                    required_conf = self.person_conf_min if is_person else effective_conf

                    if confidence >= required_conf:
                        keep_indices.append(idx)
                results[0] = results[0][keep_indices]
            except Exception as e:
                self.get_logger().warn(f"Tespit filtreleme sirasinda hata: {e}")

        # Tespit kutularını + etiketlerini (ve segmentasyon maskelerini) kare üzerine çiz.
        annotated = results[0].plot()

        det_list = []  # {"cls", "category", "conf", "track_id", "bbox", "hits"}
        for result in results:
            names = result.names
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = names.get(class_id, str(class_id))
                raw_track_id = None
                if getattr(box, "id", None) is not None:
                    try:
                        raw_track_id = int(box.id[0])
                    except Exception:
                        raw_track_id = None
                bbox = None
                if getattr(box, "xyxy", None) is not None:
                    try:
                        bbox = [round(float(v), 1) for v in box.xyxy[0].tolist()]
                    except Exception:
                        bbox = None

                norm_cat = normalize_category(class_name)
                # BÜTÜN tespitlere benzersiz integer ID ataması (ByteTrack atayamadıysa fallback takipçi çalışır)
                final_id, hits = self.assign_object_id(raw_track_id, norm_cat, bbox)

                det_list.append({
                    "cls": class_name,
                    "category": norm_cat,
                    "conf": confidence,
                    "track_id": final_id,
                    "bbox": bbox,
                    "hits": hits,
                })

        # Tiling: tam kare az/hiç tespit verdiyse kareyi parçalayıp küçük cisimleri
        # kurtarmayı dene. Yeni tespitleri det_list'e ekle ve annotated'a manuel çiz.
        if YOLO_ENABLE_TILING and len(det_list) < YOLO_TILE_MIN_DETS:
            annotated = self._merge_tile_detections(
                frame, annotated, det_list, effective_conf, frame_area
            )

        # min_hits: track_id olan hedeflerden yeterince tutarlı görülmeyenleri
        # yayın/log dışında tut (tek karelik sahte pozitifleri eler). track_id yoksa
        # (predict/tile) filtre uygulanmaz — bu ortamda cisimler kısa süre görünür.
        if YOLO_MIN_HITS > 1:
            det_list = [
                d for d in det_list
                if d["track_id"] is None or d["hits"] >= YOLO_MIN_HITS
            ]

        # Araçların (Kamyon/Treyler/Traktör) parçalı tespiti yerine TEK VE BÜTÜN bir kutu üretmek için birleştirme:
        det_list = self.merge_vehicle_boxes(det_list)

        # Metrikleri güncelle (periyodik YOLO_METRICS satırı için).
        self.metrics["raw"] += len(raw_candidates)
        self.metrics["filtered"] += len(det_list)
        self.metrics["tracked"] += sum(1 for d in det_list if d["track_id"] is not None)
        if not det_list:
            self.metrics["missed_frames"] += 1
        self.metrics_frames += 1

        # Kutulanmış görüntüyü yayınla (GUI kamera penceresi bunu gösterir).
        if rclpy.ok():
            try:
                out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                out_msg.header = msg.header
                self.annotated_pub.publish(out_msg)
            except Exception:
                pass

        now = time.time()
        if det_list:
            # Tespit varsa saniyede en fazla bir kez logla + GUI'ye yayınla.
            if now - self.last_log_time > 1.0:
                self.get_logger().info(
                    "Tespitler: " + ", ".join(
                        f"{d['cls']}:{d['conf']:.2f}"
                        + (f"#{d['track_id']}" if d['track_id'] is not None else "")
                        for d in det_list
                    )
                )
                self.publish_detections(det_list)
                self.log_detections(det_list, effective_conf)
                self.last_log_time = now
        else:
            # Cisim gösterim eşiğini geçemedi ama model zayıf bir aday buldu:
            # 1 sn'de bir bunu logla. Böylece traktör/turuncu kutu gibi hiç
            # görünmeyen hedeflerin modelde ne güven ürettiğini (ör. truck:0.08)
            # veya hiç ateşlemediğini kaçırmadan görürüz.
            if raw_candidates and now - self.last_subthresh_log > 1.0:
                top = ", ".join(f"{l}:{c:.2f}" for l, c in raw_candidates[:3])
                self.get_logger().info(
                    f"Eşik-altı aday(lar): {top} (gösterim eşiği {effective_conf:.2f}, kare {self.frame_count})"
                )
                self.last_subthresh_log = now
            # Ayrıca 5 sn'de bir "canlıyım" mesajı (kare akıyor mu görmek için).
            if now - self.last_heartbeat > 5.0:
                self.get_logger().info(
                    f"İşlenen kare: {self.frame_count} (bu aralıkta tespit yok)"
                )
                self.last_heartbeat = now

        # Periyodik tuning metriği: hangi ayarın (conf/iou/imgsz/preprocess/tiling)
        # daha iyi tespit ürettiğini karşılaştırmak için 10 sn'de bir özetle.
        if now - self.last_metrics_log > 10.0:
            m = self.metrics
            self.get_logger().info(
                f"YOLO_METRICS frame={self.frame_count} raw={m['raw']} "
                f"filtered={m['filtered']} tracked={m['tracked']} "
                f"missed_frames={m['missed_frames']} conf={effective_conf:.2f} "
                f"iou={YOLO_IOU_THRESHOLD:.2f} imgsz={YOLO_IMGSZ} "
                f"preprocess={self.preprocess_mode} "
                f"tiling={int(YOLO_ENABLE_TILING)}"
            )
            try:
                print(json.dumps({
                    "event_type": "metrics",
                    "timestamp": datetime.now(LOCAL_TZ).isoformat(),
                    "window_frames": self.metrics_frames,
                    **m,
                    "conf": round(effective_conf, 3),
                    "iou": YOLO_IOU_THRESHOLD,
                    "imgsz": YOLO_IMGSZ,
                    "preprocess": self.preprocess_mode,
                    "tiling": YOLO_ENABLE_TILING,
                }), flush=True)
            except Exception as e:
                self.get_logger().warn(f"Metrik loglama hatası: {e}")
            # Pencereyi sıfırla + track_hits'i sınırla (uzun uçuşta şişmesin).
            self.metrics = {"raw": 0, "filtered": 0, "tracked": 0, "missed_frames": 0}
            self.metrics_frames = 0
            self.last_metrics_log = now
            if len(self.track_hits) > 2000:
                self.track_hits.clear()

    def _merge_tile_detections(self, frame, annotated, det_list, effective_conf, frame_area):
        """Tile tespitlerini filtreleyip (bbox-frac, başlangıç bastırma, eşik)
        det_list'e ekler ve annotated kareye manuel kutu çizer. Güncellenmiş
        annotated kareyi döndürür."""
        tile_dets = self.run_tiles(frame, min(effective_conf, YOLO_RAW_CONF))
        suppress_start = False
        if self.latitude is not None and self.longitude is not None:
            if self.distance_m(39.920782, 32.854115, self.latitude, self.longitude) < 150.0:
                suppress_start = True
        for td in tile_dets:
            if td["conf"] < effective_conf:
                continue
            if suppress_start:
                continue
            bbox = td["bbox"]
            if bbox is not None and frame_area > 0:
                area = max(0.0, bbox[2] - bbox[0]) * max(0.0, bbox[3] - bbox[1])
                if area > YOLO_MAX_BBOX_FRAC * frame_area:
                    continue
            # Tam-kare tespitleriyle çakışıyorsa (aynı cisim) atla.
            if any(self._iou(bbox, d.get("bbox")) >= 0.4 for d in det_list):
                continue
            td["category"] = normalize_category(td["cls"])
            td["hits"] = 1
            det_list.append(td)
            # Annotated'a manuel çiz (tile tespitleri plot()'a girmez).
            if bbox is not None:
                try:
                    x1, y1, x2, y2 = [int(v) for v in bbox]
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 200, 255), 1)
                    cv2.putText(annotated, f"{td['cls']} {td['conf']:.2f}", (x1, max(0, y1 - 3)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
                except Exception:
                    pass
        return annotated

    def dynamic_conf(self) -> float:
        if not self.use_dynamic_conf:
            return CONFIDENCE_THRESHOLD
        
        alt = self.altitude
        if alt is None:
            return CONFIDENCE_THRESHOLD
        
        if alt >= self.alt_high:
            return self.conf_high_alt
        if alt <= self.alt_low:
            return self.conf_low_alt
        
        # Ara irtifada doğrusal interpolasyon
        t = (alt - self.alt_low) / (self.alt_high - self.alt_low)
        return self.conf_low_alt + t * (self.conf_high_alt - self.conf_low_alt)

    def preprocess_frame(self, frame):
        """Ön işleme modunu uygular (none/clahe/gamma/sharpen/clahe_gamma). Hata
        olursa orijinal kareyi döndürür; inference'ı asla durdurmaz."""
        mode = self.preprocess_mode
        if mode == "none":
            return frame
        try:
            out = frame
            if "clahe" in mode:
                lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
                l, a, b = cv2.split(lab)
                l = self.clahe.apply(l)
                out = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)
            if "gamma" in mode:
                out = cv2.LUT(out, self._gamma_lut)
            if "sharpen" in mode:
                kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
                out = cv2.filter2D(out, -1, kernel)
            return out
        except Exception as e:
            self.get_logger().warn(f"Ön işleme hatası ({mode}): {e}")
            return frame

    def run_inference(self, frame, conf):
        """Önce ByteTrack ile takip dener; tracker bu ortamda çalışmazsa uyarı
        yazıp düz detection moduna kalıcı olarak düşer (sistem durmaz)."""
        if self.use_tracking:
            try:
                return self.model.track(
                    frame,
                    verbose=False,
                    conf=conf,
                    iou=YOLO_IOU_THRESHOLD,
                    imgsz=YOLO_IMGSZ,
                    device=YOLO_DEVICE,
                    half=YOLO_HALF,
                    tracker=TRACKER_NAME,
                    persist=True,
                )
            except Exception as exc:
                self.use_tracking = False
                self.get_logger().warn(
                    f"Tracking başarısız, düz tespit moduna geçiliyor: {exc}"
                )
        return self.model.predict(
            frame,
            verbose=False,
            conf=conf,
            iou=YOLO_IOU_THRESHOLD,
            imgsz=YOLO_IMGSZ,
            device=YOLO_DEVICE,
            half=YOLO_HALF,
        )

    def run_tiles(self, frame, conf):
        """Kareyi YOLO_TILE_GRID x YOLO_TILE_GRID parçaya bölüp her parçada
        predict çalıştırır; kutuları tam-kare koordinatına geri çevirir. Küçük/uzak
        cisimlerin recall'ını artırmak için (track yok; ham tespit). CPU/VRAM
        maliyeti yüksek olduğu için yalnızca tiling açıkken çağrılır."""
        dets = []
        h, w = frame.shape[:2]
        g = YOLO_TILE_GRID
        th, tw = h // g, w // g
        if th <= 0 or tw <= 0:
            return dets
        for gy in range(g):
            for gx in range(g):
                x0, y0 = gx * tw, gy * th
                x1 = w if gx == g - 1 else x0 + tw
                y1 = h if gy == g - 1 else y0 + th
                tile = frame[y0:y1, x0:x1]
                try:
                    res = self.model.predict(
                        tile, verbose=False, conf=conf, iou=YOLO_IOU_THRESHOLD,
                        imgsz=YOLO_IMGSZ, device=YOLO_DEVICE, half=YOLO_HALF,
                    )
                except Exception as e:
                    self.get_logger().warn(f"Tile inference hatası: {e}")
                    continue
                if not res or res[0].boxes is None:
                    continue
                names = res[0].names
                for box in res[0].boxes:
                    try:
                        bx1, by1, bx2, by2 = box.xyxy[0].tolist()
                    except Exception:
                        continue
                    dets.append({
                        "cls": names.get(int(box.cls[0]), str(int(box.cls[0]))),
                        "conf": float(box.conf[0]),
                        "track_id": None,
                        "bbox": [round(bx1 + x0, 1), round(by1 + y0, 1),
                                 round(bx2 + x0, 1), round(by2 + y0, 1)],
                    })
        return dets

    def assign_object_id(self, track_id: Optional[int], category: str, bbox: Optional[List[float]]) -> Tuple[int, int]:
        """ByteTrack ID atayamadıysa bile centroid/IoU bazlı fallback takipçi ile HER tespitedilmiş nesneye benzersiz bir integer ID ve hits atar. ID asla None kalmaz."""
        if track_id is not None:
            hits = self.track_hits.get(track_id, 0) + 1
            self.track_hits[track_id] = hits
            return track_id, hits

        now = time.time()
        matched_track = None
        if bbox is not None:
            best_iou = 0.2
            for trk in self.active_fallback_tracks:
                if now - trk["last_seen"] > 2.0:
                    continue
                iou = self._iou(bbox, trk["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    matched_track = trk

        if matched_track is not None:
            matched_track["last_seen"] = now
            matched_track["bbox"] = bbox
            matched_track["hits"] += 1
            return matched_track["id"], matched_track["hits"]
        else:
            new_id = self.next_fallback_id
            self.next_fallback_id += 1
            if bbox is not None:
                self.active_fallback_tracks.append({
                    "id": new_id,
                    "category": category,
                    "bbox": bbox,
                    "last_seen": now,
                    "hits": 1
                })
                if len(self.active_fallback_tracks) > 50:
                    self.active_fallback_tracks = [
                        t for t in self.active_fallback_tracks if now - t["last_seen"] <= 3.0
                    ]
            return new_id, 1

    @staticmethod
    def _iou(a, b):
        if not a or not b:
            return 0.0
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0.0:
            return 0.0
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0.0

    @staticmethod
    def _boxes_close(a, b, max_dist=50.0):
        if not a or not b:
            return False
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        dx = max(0.0, max(ax1, bx1) - min(ax2, bx2))
        dy = max(0.0, max(ay1, by1) - min(ay2, by2))
        return dx <= max_dist and dy <= max_dist

    def merge_vehicle_boxes(self, det_list):
        """Aynı araca (kamyon/treyler/traktör) ait parçalı bounding box'ları birleştirerek tüm aracı kapsayan TEK bir bütünsel kutu üretir."""
        vehicles = [d for d in det_list if d["category"] == "vehicle"]
        others = [d for d in det_list if d["category"] != "vehicle"]

        if len(vehicles) <= 1:
            return det_list

        merged_vehicles = []
        visited = [False] * len(vehicles)

        for i in range(len(vehicles)):
            if visited[i]:
                continue
            visited[i] = True
            current = dict(vehicles[i])
            box_a = current["bbox"]

            for j in range(i + 1, len(vehicles)):
                if visited[j]:
                    continue
                box_b = vehicles[j]["bbox"]
                if box_a and box_b and (self._iou(box_a, box_b) > 0.05 or self._boxes_close(box_a, box_b, max_dist=50.0)):
                    visited[j] = True
                    box_a = [
                        min(box_a[0], box_b[0]),
                        min(box_a[1], box_b[1]),
                        max(box_a[2], box_b[2]),
                        max(box_a[3], box_b[3]),
                    ]
                    current["conf"] = max(current["conf"], vehicles[j]["conf"])

            current["bbox"] = box_a
            merged_vehicles.append(current)

        return others + merged_vehicles

    def publish_detections(self, det_list):
        # Tespitleri JSON olarak /yolo/detections'a yayınlar (GUI tablo/timeline için).
        if not rclpy.ok():
            return
        payload = {
            "time": time.strftime("%H:%M:%S"),
            "detections": [
                {
                    "track_id": d["track_id"],
                    "cls": d["cls"],
                    "category": d.get("category", normalize_category(d["cls"])),
                    "conf": round(float(d["conf"]), 3),
                    "bbox": d["bbox"],
                    "hits": d.get("hits", 1),
                }
                for d in det_list
            ],
        }
        try:
            det_msg = String()
            det_msg.data = json.dumps(payload)
            self.detections_pub.publish(det_msg)
        except Exception:
            pass

    def log_detections(self, det_list, effective_conf):
        """Her tespit edilen cisim için Pydantic ile doğrulanmış tek satır JSONL
        kaydı yazar. stdout, app.py tarafından yolo.log dosyasına yönlendirilir."""
        now = datetime.now(LOCAL_TZ)
        tracker = "bytetrack" if self.use_tracking else "none"
        for d in det_list:
            try:
                record = YoloDetectionLog(
                    timestamp=now,
                    local_time=now.strftime("%H:%M"),
                    uav_altitude_m=self.altitude,
                    latitude=self.latitude,
                    longitude=self.longitude,
                    object=DetectionObjectLog(
                        label=d["cls"],
                        normalized_category=d.get("category", normalize_category(d["cls"])),
                        count=1,
                        confidence=d["conf"],
                    ),
                    track_id=d["track_id"],
                    bbox_xyxy=d["bbox"],
                    hits=d.get("hits", 1),
                    tracker_name=tracker,
                    source_topic=self.topic,
                    frame_id=self.frame_count,
                    confidence_threshold=effective_conf,
                    inference_imgsz=YOLO_IMGSZ,
                    preprocess_mode=self.preprocess_mode,
                    tiling_enabled=YOLO_ENABLE_TILING,
                )
            except Exception as exc:
                self.get_logger().warn(f"Tespit log kaydı doğrulanamadı: {exc}")
                continue
            print(record.model_dump_json(), flush=True)


def main(args=None):
    rclpy.init(args=args)
    # Hedef sınıf listesi ortam değişkeniyle ayarlanabilir (YOLO_TARGET_CLASSES,
    # virgülle ayrık). Varsayılan liste bu ortamda VERİYLE seçildi:
    # 'car' KASITLI dışarıda — gerçek araçlar (traktör/trailer) 'truck' olarak
    # geliyor; 'car' ise neredeyse yalnızca zemini/gölgeyi tam-kare sanan sahte
    # pozitif üretiyordu (bkz. build_144152: car:0.57 = %99 kare). Daha geniş
    # prompt denemek için: YOLO_TARGET_CLASSES="person,human,truck,box,crate,cargo box".
    # 'dumpster' ve 'trailer' de aynı gerekçeyle çıkarıldı: karşılık gelen cisimler
    # rotadan kaldırıldı (yerlerine drone ve sırt çantası kondu), kalan prompt'lar
    # yalnızca sahte pozitif yüzeyi olurdu — traktörler zaten 'semi-truck' geliyor.
    default_classes = ("person,human,pedestrian,truck,semi-truck,tractor,"
                       "box,crate,cargo box,orange box,qr code box,"
                       "drone,quadcopter,uav,backpack,rucksack")
    target_classes = [
        c.strip() for c in os.environ.get("YOLO_TARGET_CLASSES", default_classes).split(",")
        if c.strip()
    ]
    selected_topic = "/bottom_camera/image"
    node = GazeboYoloNode(model_name="yoloe-26s-seg.pt", topic=selected_topic, classes=target_classes)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM (app kapanışı) veya Ctrl+C: gürültülü traceback yerine temiz çıkış.
        node.get_logger().info("YOLOE tespit düğümü sonlandırılıyor...")
    except Exception as exc:
        # Kapanış yarışı: SIGINT/SIGTERM sırasında spin bazen ExternalShutdown
        # yerine ham RCLError fırlatır. Bağlam zaten kapandıysa sessiz yut;
        # değilse gerçek bir hatadır, yeniden fırlat.
        if rclpy.ok():
            raise
        node.get_logger().info(f"Kapanış sırasında yoksayıldı: {exc}")
    finally:
        node.destroy_node()
        # Bağlam zaten kapatıldıysa (SIGTERM) tekrar kapatmayı deneme.
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
