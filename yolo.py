import cv2
import numpy as np
import os
import time
import json
import math
from collections import deque
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
# Kamera yatay görüş açısı (radyan). mini_talon_vtail bottom_camera: 80°, nadir
# (pitch 90 => tam aşağı) bakar. Fiziksel boyut kapısının ölçek hesabı buna dayanır.
YOLO_CAM_HFOV = float(os.environ.get("YOLO_CAM_HFOV", "1.3962634"))
# Fiziksel boyut kapısı: nadir kamerada irtifa + FOV bilindiği için bir bbox'ın
# YERDE kaç metre ettiği hesaplanabilir (bkz. px_per_m). Rotadaki en küçük cisim
# ~3.2 m (sırt çantası), en büyüğü ~6.5 m (traktör); ölçeklenmiş insan aktörüne
# pay bırakıp üst sınır 9 m. Bu aralığın dışı fiziksel olarak hedef OLAMAZ:
# alt sınırın altı = pist boyası/QR deseni detayı, üstü = zemin/gölge yaması.
# YOLO_MAX_BBOX_FRAC'tan çok daha keskin: 15 m'de %40 kare hâlâ ~12 m'lik bir
# kutuya izin verirken (build_20260717_103118 frame 95: box:0.40 = 8.8x11.5 m
# sahte pozitif geçmişti) bu kapı onu eler.
# --- Hedef yer koordinatı (geo-projeksiyon) ---
# Nadir kamerada bbox merkezi, irtifa+FOV+yön bilindiği için YERDE bir noktaya
# düşürülebilir. Eskiden her tespite İHA'nın kendi GPS'i yazılıyordu: haritada
# bütün hedefler uçuş hattının üstünde birikiyordu (yanal hata ±13 m'ye kadar).
YOLO_GEO_PROJECT = os.environ.get("YOLO_GEO_PROJECT", "1") == "1"
# Kare ile GPS örneği arasındaki ARTIK gecikme (sn), ölçülen GPS yaşının üstüne
# eklenir. Pozitif = kare, sandığımızdan daha yeni (İHA daha ileride).
# NEDEN: build_20260721_105828 loglarındaki tespitler worlds/*.sdf'teki gerçek
# kutu konumlarına göre rota boyunca sistematik olarak ~6.5-11 m GERİDE
# çıkıyordu. Sebep MAVROS GPS'inin düşük hızda yayınlanması: kullanılan konum
# örneği kareden ~0.5 sn eskiydi (14 m/s × 0.5 s ≈ 7 m). Bu yüzden telafi sabit
# bir sayı değil, örneğin GERÇEK YAŞI (now - gps_zaman_damgası) üzerinden
# yapılır; aşağıdaki sabit yalnızca kamera/köprü kuyruk gecikmesi içindir.
# Ölçmek için: bir uçuş logundaki object_latitude/longitude'u worlds/*.sdf
# pose'larıyla karşılaştır (ortalama hata bu değerle minimize edilir).
YOLO_GEO_LAG_SEC = float(os.environ.get("YOLO_GEO_LAG_SEC", "0.0"))
# Aynı fiziksel cismi kareler arası eşleştirme yarıçapı (m). Kare hızı düşük
# (~2 FPS) olduğu için görüntü-uzayı IoU eşleşmesi çalışmıyordu: aynı kutu her
# karede yeni ID alıyor, hits hep 1 kalıyor, QR aynı cisim için defalarca
# çözülüyordu. Yer koordinatı üzerinden eşleşme kare hızından bağımsızdır.
YOLO_GEO_MATCH_M = float(os.environ.get("YOLO_GEO_MATCH_M", "6.0"))
# Bir yer-izinin (geo track) canlı sayıldığı süre (sn).
YOLO_GEO_TRACK_TTL = float(os.environ.get("YOLO_GEO_TRACK_TTL", "60.0"))
YOLO_MIN_OBJ_M = float(os.environ.get("YOLO_MIN_OBJ_M", "1.2"))
YOLO_MAX_OBJ_M = float(os.environ.get("YOLO_MAX_OBJ_M", "9.0"))
# İnsana özel alt sınır: tepeden bakışta bir insanın ayak izi GERÇEKTEN küçük.
# Ölçüldü (2026-07-17, rotadaki aktörler): 15 m'de bbox ~30-31 px = 1.16-1.23 m,
# yani genel 1.2 m tabanının tam üstünde/altında — aktör pozuna göre rastgele
# elenip elenmemesi anlamına geliyordu. person_conf_min ile aynı mantık: insan
# kendi eşiğini alır. 0.4 m (~10 px) doku gürültüsünü hâlâ eler ama insanı tutar.
YOLO_MIN_OBJ_M_PERSON = float(os.environ.get("YOLO_MIN_OBJ_M_PERSON", "0.4"))
# Ego (kendi gölgesi) bastırma parametreleri — bkz. EgoStaticZones.
YOLO_EGO_FILTER = os.environ.get("YOLO_EGO_FILTER", "1") == "1"
YOLO_EGO_RADIUS_PX = float(os.environ.get("YOLO_EGO_RADIUS_PX", "40"))
YOLO_EGO_MIN_HITS = int(os.environ.get("YOLO_EGO_MIN_HITS", "6"))
YOLO_EGO_MIN_TRAVEL_M = float(os.environ.get("YOLO_EGO_MIN_TRAVEL_M", "40"))
YOLO_EGO_TTL_S = float(os.environ.get("YOLO_EGO_TTL_S", "20"))
# Ego kanıtının KESİNTİSİZ olması için izin verilen en büyük boşluk (sn).
# Gölge pratikte her karede görünür; bu süreden uzun süre kaybolup geri gelen bir
# kutu aynı cisim sayılmaz, kanıt sayacı sıfırlanır. Bu kural olmadan rotadaki
# ARDIŞIK cisimler aynı piksel izinden geçtikçe bir bölgeyi yanlışlıkla "sabit"
# gibi gösterip onaylatabiliyor (bkz. test_safety: gerçek cisim bastırılıyordu).
YOLO_EGO_MAX_GAP_S = float(os.environ.get("YOLO_EGO_MAX_GAP_S", "1.5"))
# İç içe kutu bastırma: aynı kategoriden büyük bir kutunun içinde kalan küçük
# kutu (QR deseni/pist rakamı içindeki alt kareler) atılır.
YOLO_NESTED_FILTER = os.environ.get("YOLO_NESTED_FILTER", "1") == "1"
# Görüntü ön işleme modu: none | clahe | gamma | sharpen | clahe_gamma.
# Geriye dönük uyumluluk: eski YOLO_CLAHE=1 varsayılanını korumak için varsayılan
# "clahe". YOLO_PREPROCESS açıkça verilirse o kazanır.
YOLO_PREPROCESS = os.environ.get("YOLO_PREPROCESS", "clahe").lower()
YOLO_GAMMA = float(os.environ.get("YOLO_GAMMA", "1.2"))
# Tiling: GPU varsa veya kullanıcı açıkça YOLO_ENABLE_TILING=1 verdiyse açık,
# CPU'da 4 ekstra çıkarım kare hızını ~0.5 FPS'e düşürüp video akışını dondurduğu
# için CPU ortamında varsayılan kapalı ("0").
default_tiling = "1" if torch.cuda.is_available() else "0"
YOLO_ENABLE_TILING = os.environ.get("YOLO_ENABLE_TILING", default_tiling) == "1"
YOLO_TILE_GRID = max(1, int(os.environ.get("YOLO_TILE_GRID", "2")))
# Tam karede tespit sayısı bunun altındaysa tiling devreye girer.
YOLO_TILE_MIN_DETS = int(os.environ.get("YOLO_TILE_MIN_DETS", "1"))
# Temporal smoothing: bir hedef en az bu kadar kare görülmeden tabloya/loga
# geçmez (tek karelik sahte pozitifleri eler). Varsayılan 1 = mevcut davranış
# (bastırma yok); bu ortamda sorun kaçırılan tespitler olduğu için düşük tutulur.
YOLO_MIN_HITS = max(1, int(os.environ.get("YOLO_MIN_HITS", "1")))
# QR çözme: cargo_box tespitlerinin bbox'ı kırpılıp büyütülür ve içindeki QR
# kodun METNİ okunur (cv2.QRCodeDetector; ek bağımlılık yok). Rotadaki QR
# kutular models/textures/make_qr_text_textures.py ile üretilen metinli QR
# taşır. Ölçüm: 15 m irtifada 3 m kutu ~84 px görünür; 4x cubic büyütme sonrası
# çözüm güvenilir (60 px'e kadar çalıştı). Metin track_id başına bir kez
# çözülür ve önbelleğe alınır.
YOLO_QR_DECODE = os.environ.get("YOLO_QR_DECODE", "1") == "1"
YOLO_QR_UPSCALE = float(os.environ.get("YOLO_QR_UPSCALE", "4.0"))
YOLO_QR_MARGIN_PX = int(os.environ.get("YOLO_QR_MARGIN_PX", "8"))
# Renkli kutu sınıflandırması: insan aktörlerinin yerine konan KIRMIZI/MAVİ
# kutular (2026-07-18, bkz. world + make_color_box_textures.py). Düz renkli
# kutu nadir'de görünmez olduğu için (ölçüm: turuncu kutu 0.05) renkli kutular
# ölçülmüş QR desenini korur → YOLOE onları yine 'qr code box' olarak bulur;
# RENK ayrımı burada yapılır: bbox kırpıntısının HSV baskın rengi kutu
# alanının en az MIN_FRAC'ı kadar kırmızı/maviyse kategori red_box/blue_box'a
# çevrilir. Doku ölçümü: kare doku alanının 0.55'i saf renk (çerçeve+modüller),
# siyah-beyaz QR kutularda 0.00 → 0.15 eşiği ikisini geniş marjla ayırır.
YOLO_BOX_COLOR_CLASSIFY = os.environ.get("YOLO_BOX_COLOR_CLASSIFY", "1") == "1"
YOLO_BOX_COLOR_MIN_FRAC = float(os.environ.get("YOLO_BOX_COLOR_MIN_FRAC", "0.15"))


# Benzer sınıfları tek hedef kategorisine normalize eder: aynı cisim bir karede
# "box", diğerinde "crate" gelirse iki farklı hedef gibi sayılmasın. Ham etiket
# (raw label) log ve payload'da ayrıca korunur.
CLASS_NORMALIZATION = {
    "person": "person",
    "human": "person",
    "pedestrian": "person",
    "walking person": "person",
    "car": "vehicle",
    "automobile": "vehicle",
    "sedan": "vehicle",
    "suv": "vehicle",
    "hatchback": "vehicle",
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
    # NOT: 'drone/uav/quadcopter' KASITLI yok. Bu prompt'lar rotadaki drone
    # cismini bulmaktan çok SİHA'nın KENDİ GÖLGESİNİ 'uav' olarak işaretliyordu
    # (bkz. build_20260717_103118). Drone cismi world'den kaldırılıp yerine
    # otomobil kondu; ilgili prompt'lar da kaldırıldı.
    "backpack": "backpack",
    "rucksack": "backpack",
    "knapsack": "backpack",
    # Rotaya 2026-07-17'de eklenen yeni cisim türleri (ölçümle seçildi,
    # bkz. scratchpad/probe2: stop sign 0.74, checkerboard 0.79 — nadir'de
    # doğru etiketle ateşleyen tek iki yeni aday).
    "stop sign": "stop_sign",
    "checkerboard": "checker_panel",
    "chessboard": "checker_panel",
    # 3. tur ölçümle eklenen çeşitlilik (bkz. make_route_object_textures.py):
    # nişan tahtası 'concentric circles':0.57-0.67.
    # 'target' KASITLI kullanılmıyor: nişan tahtasında 0.61 veriyor ama SIRT
    # ÇANTASINI da 0.23 ile 'target' sanıyordu (eşiğin üstü => çanta tabloda
    # "Nişan Tahtası" görünüyordu). 'concentric circles' aynı cismi 0.64 ile
    # bulup çantada 0.02'de kalıyor.
    "concentric circles": "bullseye",
    "target": "bullseye",
    "bullseye": "bullseye",
}


# Sınıf-bazlı güven TABANI (normalize kategori -> min güven). Genel eşiğin
# (effective_conf) ÜSTÜNE biner, altına inmez. person_conf_min'in tersi:
# o bir cismi kurtarmak için eşiği DÜŞÜRÜR, bu ise hayalet etiketi elemek
# için YÜKSELTİR.
#
# bullseye=0.30: 'concentric circles' promptu nişan tahtasını 0.57-0.67 ile
# bulurken STOP TABELASINI da 0.15-0.22 ile "eş merkezli halka" sanıyor (stop
# tabelası sekizgen + beyaz halka). Eşik 15 m'de 0.10 olduğu için bu hayalet
# tabloya "Nişan Tahtası" satırı olarak düşüyordu. 0.30 tabanı ikisinin arasına
# giriyor (3 bakış konumundan ölçüldü: gerçek min 0.57, hayalet maks 0.22).
# stop_sign=0.30: hayalet ters yönde de işliyor — nişan tahtası (kırmızı/beyaz
# halkalar) 0.15 ile 'stop sign' sanılıyor (stop tabelası da kırmızı-beyaz
# yuvarlakça bir levha). Gerçek stop tabelaları 3 bakış konumunda da 0.71-0.81
# verdiği için 0.30 tabanı ikisini geniş marjla ayırıyor.
# checker_panel=0.30: 'checkerboard' promptu boş zemin/gölge yamalarında ~0.11-0.13
# seviyesinde sahte pozitif üretiyordu. Gerçek dama paneli 0.79 güven ürettiği
# için 0.30 tabanı zemin gürültüsünü eler.
CLASS_CONF_MIN = {
    "bullseye": float(os.environ.get("YOLO_BULLSEYE_CONF_MIN", "0.30")),
    "stop_sign": float(os.environ.get("YOLO_STOP_SIGN_CONF_MIN", "0.30")),
    "checker_panel": float(os.environ.get("YOLO_CHECKER_PANEL_CONF_MIN", "0.30")),
    "vehicle": float(os.environ.get("YOLO_VEHICLE_CONF_MIN", "0.12")),
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
    # 1.3: hedefin kendi yer koordinatı (object_latitude/longitude) eklendi;
    # latitude/longitude alanları İHA'nın konumu olmayı sürdürür.
    schema_version: str = "1.3"
    event_type: Literal["object_detection"] = "object_detection"
    timestamp: datetime
    local_time: str
    uav_altitude_m: Optional[float] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    object_latitude: Optional[float] = Field(
        default=None, description="Hedefin yerdeki enlemi (bbox merkezinin geo-projeksiyonu)")
    object_longitude: Optional[float] = Field(
        default=None, description="Hedefin yerdeki boylamı (bbox merkezinin geo-projeksiyonu)")
    object: DetectionObjectLog
    track_id: Optional[int] = None
    bbox_xyxy: Optional[List[float]] = None
    qr_text: Optional[str] = Field(default=None, description="Kutunun QR kodundan çözülen metin")
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
        # Mesajda hedefin KENDİ koordinatı gösterilir (varsa); yoksa İHA'nınki.
        obj_lat = self.object_latitude if self.object_latitude is not None else self.latitude
        obj_lon = self.object_longitude if self.object_longitude is not None else self.longitude
        if obj_lat is not None and obj_lon is not None:
            coord_part = f"[{obj_lat:.6f}, {obj_lon:.6f}] koordinatında"
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
        qr_part = f" QR içeriği: \"{self.qr_text}\"." if self.qr_text else ""
        return f"Saat {self.local_time}'te, {alt_part}{coord_part} {obj_part}{hits_part}{qr_part}"


class EgoStaticZones:
    """İHA uçarken KARE İÇİNDE SABİT kalan sahte tespitleri (kendi gölgesi,
    görüşe giren gövde parçası) öğrenip bastırır.

    Ayırt edici işaret şu: 15 m irtifada kamera yerde ~25 m'lik bir şerit görür,
    gerçek bir cisim ~13 m/s hızda kareyi ~2 sn'de (~20 m yolda) süpürüp çıkar —
    kare içinde sabit KALAMAZ. SİHA'nın kendi gölgesi ise güneş açısı sabit
    olduğu için uçakla birlikte taşınır, dolayısıyla hep aynı piksel bölgesinde
    durur. Bir aday bölge, uçak min_travel_m yol aldığı halde hâlâ aynı
    piksellerde görünüyorsa ego artefaktı olarak onaylanır ve o bölgeye düşen
    tespitler bastırılır.

    build_20260717_103118 bu imzayı birebir gösteriyor: track#9, 69 kare boyunca
    bbox=[54,163,112,208] (±1 px), İHA ~13 m/s ilerlerken.

    Yanlışlıkla gerçek cismi bastırmamak için üç koruma var:
      * Onay yalnızca uçak GERÇEKTEN yol alırken verilir (min_travel_m).
      * Kanıt KESİNTİSİZ olmalı (max_gap_s): bölge her karede yeniden
        görülmezse sayaç sıfırlanır. Bu olmadan rotadaki ardışık cisimler aynı
        piksel izinden geçtikçe bir bölgeyi "sabit" gibi gösterebiliyor —
        kanıt farklı cisimlerden birikiyor.
      * Bastırma, bölgenin öğrendiği boyuta yakın kutulara uygulanır; bölgenin
        üstünden geçen daha büyük bir cisim (ör. traktör) bastırılmaz.
    """

    def __init__(self, radius_px=YOLO_EGO_RADIUS_PX, min_hits=YOLO_EGO_MIN_HITS,
                 min_travel_m=YOLO_EGO_MIN_TRAVEL_M, ttl_s=YOLO_EGO_TTL_S,
                 max_gap_s=YOLO_EGO_MAX_GAP_S):
        self.radius_px = radius_px
        self.min_hits = min_hits
        self.min_travel_m = min_travel_m
        self.ttl_s = ttl_s
        self.max_gap_s = max_gap_s
        self.zones = []

    @staticmethod
    def _center_wh(bbox):
        x1, y1, x2, y2 = bbox
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0, abs(x2 - x1), abs(y2 - y1)

    def _match(self, cx, cy):
        best, best_d = None, self.radius_px
        for z in self.zones:
            d = math.hypot(z["cx"] - cx, z["cy"] - cy)
            if d <= best_d:
                best, best_d = z, d
        return best

    def update(self, bboxes, travel_m, now):
        """Ham tespit kutularını bölgelere işler. Bastırmadan ÖNCE çağrılmalı ki
        bastırılan gölge kendi bölgesini taze tutmaya devam etsin."""
        for bbox in bboxes:
            if not bbox:
                continue
            cx, cy, w, h = self._center_wh(bbox)
            z = self._match(cx, cy)
            if z is None:
                self.zones.append({
                    "cx": cx, "cy": cy, "w": w, "h": h, "hits": 1,
                    "travel_start": travel_m, "last_seen": now, "confirmed": False,
                })
                continue
            # Kanıt kesintiye uğradıysa (bölge bir süre görünmeyip geri geldiyse)
            # bu AYNI cismin sürekli orada durduğunu göstermez — büyük olasılıkla
            # aynı iz üzerinden geçen BAŞKA bir cisim. Sayacı sıfırdan başlat.
            if now - z["last_seen"] > self.max_gap_s:
                z["cx"], z["cy"], z["w"], z["h"] = cx, cy, w, h
                z["hits"] = 1
                z["travel_start"] = travel_m
                z["last_seen"] = now
                z["confirmed"] = False
                continue
            # EMA: gölge, güneş açısı/uçuş yönü değiştikçe yavaşça kayar.
            z["cx"] += 0.2 * (cx - z["cx"])
            z["cy"] += 0.2 * (cy - z["cy"])
            z["w"] += 0.2 * (w - z["w"])
            z["h"] += 0.2 * (h - z["h"])
            z["hits"] += 1
            z["last_seen"] = now
            if (not z["confirmed"] and z["hits"] >= self.min_hits
                    and travel_m - z["travel_start"] >= self.min_travel_m):
                z["confirmed"] = True
        self.zones = [z for z in self.zones if now - z["last_seen"] <= self.ttl_s]

    def is_ego(self, bbox):
        """bbox onaylanmış bir ego bölgesine (ve o bölgenin boyut sınıfına) düşüyor mu?"""
        if not bbox:
            return False
        cx, cy, w, h = self._center_wh(bbox)
        for z in self.zones:
            if not z["confirmed"]:
                continue
            if math.hypot(z["cx"] - cx, z["cy"] - cy) > self.radius_px:
                continue
            zone_area = max(1.0, z["w"] * z["h"])
            ratio = (w * h) / zone_area
            if 0.4 <= ratio <= 2.5:
                return True
        return False

    def confirmed_zones(self):
        return [z for z in self.zones if z["confirmed"]]


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
        # sup_* : hangi sahte-pozitif kapısının kaç kutu elediği (tuning için).
        self.metrics = {"raw": 0, "filtered": 0, "tracked": 0, "missed_frames": 0,
                        "sup_size": 0, "sup_ego": 0, "sup_nested": 0}
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

        # QR çözücü: cargo_box tespitlerinin içindeki QR metnini okur.
        self.qr_detector = cv2.QRCodeDetector() if YOLO_QR_DECODE else None
        self.qr_texts = {}   # track_id -> çözülen metin (track başına tek çözüm)

        # Ego (kendi gölgesi) bastırma: kare içinde sabit kalan tespitleri öğrenir.
        self.ego_zones = EgoStaticZones() if YOLO_EGO_FILTER else None
        # Kat edilen toplam yol (m). Ego bölgesinin onayı buna dayanır; düz
        # rotada yer değiştirme = yol, ama loiter/dönüş ihtimaline karşı adım
        # adım biriktirilir.
        self.travel_m = 0.0
        self._last_gps = None
        self.last_ego_log = 0.0
        self.recent_draw_boxes = {}  # Çerçevenin kaybolmasını önlemek için son çizim kutuları tutma önbelleği

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
        self.heading_deg = 0.0
        # Zaman damgalı GPS tamponu (yer hızı + gecikme telafisi için).
        self.gps_buf = deque(maxlen=60)
        # Yer koordinatına çıpalanmış izler: {"id", "category", "lat", "lon",
        # "hits", "last_seen"}. Aynı fiziksel cisim kareler arası buradan
        # eşleşir (bkz. assign_object_id).
        self.geo_tracks = []

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
            "min_obj_m": YOLO_MIN_OBJ_M,
            "min_obj_m_person": YOLO_MIN_OBJ_M_PERSON,
            "max_obj_m": YOLO_MAX_OBJ_M,
            "cam_hfov": YOLO_CAM_HFOV,
            "ego_filter": YOLO_EGO_FILTER,
            "nested_filter": YOLO_NESTED_FILTER,
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
            # Zaman damgalı konum tamponu: yer hızı buradan türetilir (gecikme
            # telafisi için), ölçüm gürültüsüne karşı ~1 sn'lik pencere kullanılır.
            self.gps_buf.append((time.time(), self.latitude, self.longitude))
            # Kat edilen yolu biriktir (ego bölgesi onayı için). Alt eşik park
            # halindeki GPS gürültüsünün yol gibi birikmesini, üst eşik ise
            # EKF/GPS ilk kilitlenmesindeki sıçramanın tek adımda yüzlerce metre
            # eklemesini engeller.
            if self._last_gps is not None:
                step = self.distance_m(self._last_gps[0], self._last_gps[1],
                                       self.latitude, self.longitude)
                if 0.5 <= step <= 100.0:
                    self.travel_m += step
            self._last_gps = (self.latitude, self.longitude)

    def pose_cb(self, msg):
        self.altitude = float(msg.pose.position.z)
        # Pusula yönü (0=Kuzey, saat yönünde artar). MAVROS local_position/pose
        # ENU'dur: yaw Doğu ekseninden CCW ölçülür (kuzeye uçarken yaw=+90°).
        # threads/telemetri_t.py ile AYNI dönüşüm — geo-projeksiyonun kareyi
        # kuzeye hizalaması buna dayanır.
        try:
            q = msg.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.heading_deg = (90.0 - math.degrees(yaw)) % 360.0
        except Exception:
            pass

    def ground_velocity(self):
        """GPS tamponundan (kuzey, doğu) yer hızı (m/s). Veri yetersizse (0,0)."""
        if len(self.gps_buf) < 2:
            return 0.0, 0.0
        t1, lat1, lon1 = self.gps_buf[-1]
        # ~1 sn geriye git: tek adım GPS gürültüsü hızda büyük sıçrama yaratır.
        t0, lat0, lon0 = self.gps_buf[0]
        for t, la, lo in reversed(self.gps_buf):
            if t1 - t >= 1.0:
                t0, lat0, lon0 = t, la, lo
                break
        dt = t1 - t0
        if dt <= 0.05:
            return 0.0, 0.0
        vn = (lat1 - lat0) * 111000.0 / dt
        ve = (lon1 - lon0) * 111000.0 * math.cos(math.radians(lat1)) / dt
        return vn, ve

    def pose_snapshot(self):
        """Karenin işlenmeye BAŞLADIĞI andaki poz (konum/irtifa/yön/hız).

        Çıkarım CPU'da ~0.3-0.5 sn sürdüğü için eskiden tespitlere çıkarım
        BİTTİKTEN sonraki GPS yazılıyordu; hedefler rota boyunca ~10 m geride
        işaretleniyordu. Kare başında bir kez örnekleyip bütün kare boyunca aynı
        pozu kullanmak bu hatayı ortadan kaldırır."""
        vn, ve = self.ground_velocity()
        now = time.time()
        # Kullanılan GPS örneğinin yaşı: MAVROS konumu kameradan çok daha seyrek
        # yayınlıyor, bu yüzden "en güncel" konum bile kareden eski olabilir.
        gps_age = (now - self.gps_buf[-1][0]) if self.gps_buf else 0.0
        return {
            "t": now,
            "lat": self.latitude,
            "lon": self.longitude,
            "alt": self.altitude,
            "heading": self.heading_deg,
            "vn": vn,
            "ve": ve,
            "gps_age": max(0.0, min(2.0, gps_age)),   # saçma değerlere karşı sınırla
        }

    def project_to_ground(self, bbox, frame_w, frame_h, pose):
        """bbox merkezinin YERDEKİ koordinatını (lat, lon) döner; hesaplanamazsa
        (irtifa/GPS yok, geo-projeksiyon kapalı) (None, None).

        Nadir kamera: görüntü merkezinden sapma × (1/px_per_m) = metre cinsinden
        sağa/ileri kayma. Görüntüde YUKARI = uçuş yönü (ileri) — bu, uçuş
        loglarındaki kutuların hareket yönüyle doğrulandı. Sonra yön (heading)
        ile kuzey/doğu bileşenlerine çevrilir ve İHA konumuna eklenir.
        YOLO_GEO_LAG_SEC kadar geri (hız × gecikme) düzeltme uygulanır."""
        if not YOLO_GEO_PROJECT or bbox is None:
            return None, None
        lat, lon, alt = pose.get("lat"), pose.get("lon"), pose.get("alt")
        if lat is None or lon is None or alt is None or alt <= 1.0:
            return None, None
        ground_w = 2.0 * alt * math.tan(YOLO_CAM_HFOV / 2.0)
        if ground_w <= 0.0 or frame_w <= 0:
            return None, None
        scale = frame_w / ground_w            # piksel / metre
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0
        right_m = (cx - frame_w / 2.0) / scale
        fwd_m = -(cy - frame_h / 2.0) / scale
        hd = math.radians(pose.get("heading") or 0.0)
        north = fwd_m * math.cos(hd) - right_m * math.sin(hd)
        east = fwd_m * math.sin(hd) + right_m * math.cos(hd)
        # Konum örneği kareden eski: İHA bu sürede ilerlediği için konumu
        # yer hızıyla İLERİ taşı (ölçülen GPS yaşı + artık gecikme sabiti).
        dt = pose.get("gps_age", 0.0) + YOLO_GEO_LAG_SEC
        north += pose.get("vn", 0.0) * dt
        east += pose.get("ve", 0.0) * dt
        obj_lat = lat + north / 111000.0
        obj_lon = lon + east / (111000.0 * math.cos(math.radians(lat)))
        return round(obj_lat, 7), round(obj_lon, 7)

    def px_per_m(self, frame_w: int) -> Optional[float]:
        """Nadir kamerada 1 metrenin kaç piksel ettiği; irtifa yoksa None.

        Kamera tam aşağı baktığı için yerde görülen şeridin genişliği
        2*irtifa*tan(hfov/2)'dir; ölçek = kare genişliği / bu şerit. 15 m'de
        80° FOV ve 640 px ile ~25 px/m => 4 m'lik kutu ~100 px.
        """
        if self.altitude is None or self.altitude <= 1.0:
            return None
        ground_w = 2.0 * self.altitude * math.tan(YOLO_CAM_HFOV / 2.0)
        if ground_w <= 0.0:
            return None
        return frame_w / ground_w

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

        # Pozu ÇIKARIMDAN ÖNCE örnekle: aşağıdaki bütün konum işleri (başlangıç
        # bölgesi kapısı, hedef geo-projeksiyonu, log kaydı) bu kareye ait pozu
        # kullanır. Eskiden çıkarım bittikten sonraki (0.3-0.5 sn daha yeni) GPS
        # okunuyordu ve hedefler rota boyunca geride işaretleniyordu.
        pose = self.pose_snapshot()

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

        # Filter detections in the Results object before plotting.
        # İki geçiş: önce cisim OLABİLECEK adaylar süzülür (1. geçiş), sonra ego
        # gölge bölgeleri bu adaylarla güncellenip bastırma uygulanır (2. geçiş).
        # Ayrım şart: gölge bölgesi ancak bastırılan tespitlerle taze kalabilir,
        # bu yüzden bölge güncellemesi bastırmadan ÖNCE olmalı.
        frame_area = float(frame.shape[0] * frame.shape[1])
        scale = self.px_per_m(frame.shape[1])
        if len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            try:
                candidates = []  # 1. geçişi geçen adaylar
                for idx, box in enumerate(results[0].boxes):
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = names.get(class_id, str(class_id))
                    norm_cat = normalize_category(class_name)

                    # Güven kapısı önce gelir: çıkarım YOLO_RAW_CONF (0.05)
                    # tabanında koşuyor, yani kutuların çoğu tanılama gürültüsü.
                    # Bunları baştan elemek hem gereksiz işi hem de sup_*
                    # metriklerinin (kapıların ne elediğini ölçer) şişmesini önler.
                    # İnsanlar tepe perspektifinde ~0.05-0.08 güven ürettiği için
                    # özel düşük eşik alır. Eşik ham etiketten hesaplanır; aşağıdaki
                    # cargo_box->vehicle düzeltmesi person üretemediği için bu
                    # sıra eşiği değiştirmez.
                    is_person = (norm_cat == "person" or class_name.lower() in
                                 ["person", "human", "pedestrian", "walking person"])
                    if is_person:
                        min_conf = self.person_conf_min
                    else:
                        # CLASS_CONF_MIN: hayalet etiket üreten sınıflar genel
                        # eşiğin üstünde bir taban ister (bkz. tablodaki not).
                        min_conf = max(effective_conf, CLASS_CONF_MIN.get(norm_cat, 0.0))
                    if confidence < min_conf:
                        continue

                    try:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                    except Exception:
                        continue
                    bw, bh = abs(x2 - x1), abs(y2 - y1)
                    bbox = [x1, y1, x2, y2]

                    # Tam-kare sahte pozitif filtresi (irtifa yokken tek savunma).
                    bbox_area = bw * bh
                    if frame_area > 0 and bbox_area > YOLO_MAX_BBOX_FRAC * frame_area:
                        continue

                    # Fiziksel boyut kapısı: irtifa biliniyorsa kutunun yerdeki
                    # metre karşılığı hesaplanır. Hedef aralığının dışı cisim
                    # olamaz => pist boyası/QR alt deseni (küçük) ve zemin/gölge
                    # yaması (büyük) burada elenir. İnsan kendi alt sınırını alır:
                    # tepeden ayak izi ~1.2 m, genel tabanın tam sınırında.
                    if scale is not None:
                        long_m = max(bw, bh) / scale
                        min_m = YOLO_MIN_OBJ_M_PERSON if is_person else YOLO_MIN_OBJ_M
                        if long_m > YOLO_MAX_OBJ_M or long_m < min_m:
                            self.metrics["sup_size"] += 1
                            continue

                    # Kare kutu filtresi: Araçlar (car/vehicle) kuş bakışı uzun yapılıdır (aspect_ratio > 1.25).
                    # Kare (1.0-1.25) kutular araç olarak etiketlendiğinde ve conf < 0.40 ise kırpılmış kutu hayaletidir.
                    aspect_ratio = max(bw, bh) / (min(bw, bh) + 1e-5)
                    if norm_cat == "vehicle" and aspect_ratio < 1.25 and confidence < 0.40:
                        continue

                    # Başlangıç ve kalkış bölgesi bastırması (0-150m, irtifa < 10m veya GPS henüz alınmadıysa)
                    if pose["lat"] is None or pose["lon"] is None:
                        continue
                    dist_to_start = self.distance_m(39.920782, 32.854115, pose["lat"], pose["lon"])
                    if dist_to_start < 150.0 or (pose["alt"] is not None and pose["alt"] < 10.0):
                        continue

                    candidates.append({
                        "idx": idx, "bbox": bbox, "area": bbox_area,
                        "category": norm_cat, "cls": class_name,
                    })

                # Ego bölgelerini adaylarla güncelle (bastırmadan önce).
                if self.ego_zones is not None:
                    before = len(self.ego_zones.confirmed_zones())
                    self.ego_zones.update([c["bbox"] for c in candidates], self.travel_m, now)
                    zones = self.ego_zones.confirmed_zones()
                    if len(zones) > before:
                        self.get_logger().info(
                            "Ego (kendi gölgesi) bölgesi onaylandı: "
                            + ", ".join(f"({z['cx']:.0f},{z['cy']:.0f})~{z['w']:.0f}x{z['h']:.0f}px"
                                        for z in zones)
                            + f" — İHA {self.travel_m:.0f}m yol aldığı halde bu kutu"
                              " kare içinde sabit kaldı; bu bölge bastırılacak."
                        )

                keep_indices = []
                for cand in candidates:
                    # Kendi gölgesi: uçak yol alırken kare içinde sabit kalan bölge.
                    if self.ego_zones is not None and self.ego_zones.is_ego(cand["bbox"]):
                        self.metrics["sup_ego"] += 1
                        continue
                    # İç içe kutu: QR deseni / pist rakamı gibi yapılarda model hem
                    # cismin tamamını hem içindeki alt kareleri ayrı 'box' sanıyor.
                    # Aynı kategoriden belirgin daha büyük bir kutunun içinde kalan
                    # küçük kutu o cismin parçasıdır => yalnızca büyüğü tut.
                    if YOLO_NESTED_FILTER and any(
                        o is not cand
                        and o["category"] == cand["category"]
                        and o["area"] >= 2.0 * cand["area"]
                        and self._containment(cand["bbox"], o["bbox"]) >= 0.8
                        for o in candidates
                    ):
                        self.metrics["sup_nested"] += 1
                        continue
                    keep_indices.append(cand["idx"])
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
                # Hedefin YERDEKİ koordinatı (İHA'nınki değil): haritada cisim
                # kendi yerine düşer ve aynı cisim kareler arası buradan eşleşir.
                obj_lat, obj_lon = self.project_to_ground(
                    bbox, frame.shape[1], frame.shape[0], pose
                )
                # BÜTÜN tespitlere benzersiz integer ID ataması (yer eşleşmesi →
                # ByteTrack → IoU fallback sırasıyla)
                final_id, hits = self.assign_object_id(raw_track_id, norm_cat, bbox,
                                                       obj_lat, obj_lon)

                det_list.append({
                    "cls": class_name,
                    "category": norm_cat,
                    "conf": confidence,
                    "track_id": final_id,
                    "bbox": bbox,
                    "hits": hits,
                    "lat": obj_lat,
                    "lon": obj_lon,
                })

        # Tiling: tam kare az/hiç tespit verdiyse kareyi parçalayıp küçük cisimleri
        # kurtarmayı dene. Yeni tespitleri det_list'e ekle ve annotated'a manuel çiz.
        if YOLO_ENABLE_TILING and len(det_list) < YOLO_TILE_MIN_DETS:
            annotated = self._merge_tile_detections(
                frame, annotated, det_list, effective_conf, frame_area, pose
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

        # Renkli kutu ayrımı: cargo_box tespitleri bbox'ın baskın rengine göre
        # red_box/blue_box alt kategorisine ayrılır (QR çözümünden ÖNCE, çünkü
        # çözüm kapısı bu kategorileri de kapsar).
        self.classify_box_colors(frame, det_list)

        # QR metni çözme: kutu tespitlerinin içindeki QR kod okunur, metin
        # payload'a/loga eklenir ve annotated kareye yazılır.
        if self.qr_detector is not None:
            self.decode_qr_texts(frame, det_list, annotated)

        # Bütün güncel kare hedeflerini (det_list) annotated kareye belirgin çiz
        for d in det_list:
            bbox = d.get("bbox")
            if bbox is not None:
                try:
                    x1, y1, x2, y2 = [int(p) for p in bbox]
                    cat = d.get("category", "object")
                    cls_name = d.get("cls", cat)
                    conf = d.get("conf", 0.0)
                    tid = d.get("track_id")

                    label_text = f"{cls_name}:{conf:.2f}"
                    if tid is not None:
                        label_text += f" #{tid}"

                    # İnsana özel parlak sarı/turkuaz vurgu rengi (BGR: 0, 230, 255), çizgi kalınlığı 3px
                    if cat == "person":
                        color = (0, 235, 255)
                        thick = 3
                    elif cat == "red_box":
                        color = (40, 40, 230)
                        thick = 3
                    elif cat == "blue_box":
                        color = (230, 100, 30)
                        thick = 3
                    elif cat == "stop_sign":
                        color = (0, 0, 255)
                        thick = 2
                    elif cat == "checker_panel":
                        color = (255, 0, 255)
                        thick = 2
                    else:
                        color = (0, 255, 0)
                        thick = 2

                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thick)
                    # Arka plan kutucuğu ile okunabilir etiket metni
                    (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
                    y_text_bg = max(th + 6, y1)
                    cv2.rectangle(annotated, (x1, y_text_bg - th - 6), (x1 + tw + 6, y_text_bg), (20, 20, 20), -1)
                    cv2.putText(annotated, label_text, (x1 + 3, y_text_bg - 3),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1)
                except Exception:
                    pass

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
                self.log_detections(det_list, effective_conf, pose)
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
                f"missed_frames={m['missed_frames']} "
                f"sup_size={m['sup_size']} sup_ego={m['sup_ego']} "
                f"sup_nested={m['sup_nested']} "
                f"ego_zones={len(self.ego_zones.confirmed_zones()) if self.ego_zones else 0} "
                f"travel={self.travel_m:.0f}m conf={effective_conf:.2f} "
                f"iou={YOLO_IOU_THRESHOLD:.2f} imgsz={YOLO_IMGSZ} "
                f"preprocess={self.preprocess_mode} "
                f"tiling={int(YOLO_ENABLE_TILING)} "
                # Geo tanılama: kaç ayrı fiziksel hedef izleniyor ve konum
                # örneği ne kadar eski (gecikme telafisi bunun üstüne kurulu).
                f"geo_tracks={len(self.geo_tracks)} "
                f"gps_age={pose.get('gps_age', 0.0):.2f}s"
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
                    "geo_tracks": len(self.geo_tracks),
                    "gps_age_s": round(pose.get("gps_age", 0.0), 3),
                }), flush=True)
            except Exception as e:
                self.get_logger().warn(f"Metrik loglama hatası: {e}")
            # Pencereyi sıfırla + track_hits'i sınırla (uzun uçuşta şişmesin).
            self.metrics = {"raw": 0, "filtered": 0, "tracked": 0, "missed_frames": 0,
                            "sup_size": 0, "sup_ego": 0, "sup_nested": 0}
            self.metrics_frames = 0
            self.last_metrics_log = now
            if len(self.track_hits) > 2000:
                self.track_hits.clear()

    # Baskın renk -> normalize alt kategori (insan aktörlerinin yerine konan
    # renkli kutular; bkz. YOLO_BOX_COLOR_CLASSIFY notu).
    BOX_COLOR_CATEGORY = {"red": "red_box", "blue": "blue_box"}

    def classify_box_colors(self, frame, det_list):
        """cargo_box tespitlerini bbox içindeki baskın renge göre alt kategoriye
        ayırır (red_box / blue_box). Renkli kutu dokusunun ~%55'i saf renk
        (çerçeve + QR modülleri), siyah-beyaz QR kutularda renk oranı ~0.00;
        eşik (YOLO_BOX_COLOR_MIN_FRAC=0.15) ikisini geniş marjla ayırır."""
        if not YOLO_BOX_COLOR_CLASSIFY:
            return
        h, w = frame.shape[:2]
        for d in det_list:
            if d.get("category") != "cargo_box" or not d.get("bbox"):
                continue
            try:
                x1, y1, x2, y2 = [int(v) for v in d["bbox"]]
            except Exception:
                continue
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 - x1 < 8 or y2 - y1 < 8:
                continue
            color = self._dominant_box_color(frame[y1:y2, x1:x2])
            if color:
                d["category"] = self.BOX_COLOR_CATEGORY[color]
                d["box_color"] = color

    @staticmethod
    def _dominant_box_color(crop):
        """Kırpıntının kırmızı/mavi piksel oranını HSV'de ölçer; eşik üstündeki
        baskın rengi döndürür, ikisi de eşik altındaysa None (renksiz kutu)."""
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hch, sch, vch = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        sat = (sch > 70) & (vch > 50)
        red_frac = float(np.mean(sat & ((hch <= 10) | (hch >= 170))))
        blue_frac = float(np.mean(sat & (hch >= 100) & (hch <= 130)))
        if red_frac < YOLO_BOX_COLOR_MIN_FRAC and blue_frac < YOLO_BOX_COLOR_MIN_FRAC:
            return None
        return "red" if red_frac >= blue_frac else "blue"

    def decode_qr_texts(self, frame, det_list, annotated):
        """cargo_box tespitlerinin bbox'ını kırpıp büyüterek içindeki QR kodun
        metnini çözer. Metin det kaydına (qr_text) eklenir, track başına bir kez
        JSONL 'qr_decoded' olayı olarak loglanır ve annotated kareye yazılır.

        15 m irtifada 3 m kutu ~84 px göründüğü için doğrudan çözüm çoğu kez
        başarısız; YOLO_QR_UPSCALE (4x cubic) büyütme sonrası cv2.QRCodeDetector
        güvenilir çalışıyor (ölçüm: 60 px'e kadar). Çözüm başarısız olursa
        sessizce geçilir — sonraki karede kutu daha merkezde/az bulanık gelir."""
        h, w = frame.shape[:2]
        for d in det_list:
            if d.get("category") not in ("cargo_box", "red_box", "blue_box") or not d.get("bbox"):
                continue
            tid = d.get("track_id")
            cached = self.qr_texts.get(tid) if tid is not None else None
            if cached:
                d["qr_text"] = cached
                self._draw_qr_text(annotated, d["bbox"], cached)
                continue
            x1, y1, x2, y2 = d["bbox"]
            m = YOLO_QR_MARGIN_PX
            cx1, cy1 = max(0, int(x1) - m), max(0, int(y1) - m)
            cx2, cy2 = min(w, int(x2) + m), min(h, int(y2) + m)
            if cx2 - cx1 < 20 or cy2 - cy1 < 20:
                continue
            crop = frame[cy1:cy2, cx1:cx2]
            text = ""
            try:
                up = cv2.resize(crop, None, fx=YOLO_QR_UPSCALE, fy=YOLO_QR_UPSCALE,
                                interpolation=cv2.INTER_CUBIC)
                text, _, _ = self.qr_detector.detectAndDecode(up)
                if not text:
                    # Gri + Otsu eşikleme: düşük kontrast/render gürültüsünde
                    # ikinci şans.
                    gray = cv2.cvtColor(up, cv2.COLOR_BGR2GRAY)
                    _, binary = cv2.threshold(gray, 0, 255,
                                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    text, _, _ = self.qr_detector.detectAndDecode(binary)
            except Exception:
                continue
            if not text:
                continue
            d["qr_text"] = text
            if tid is not None:
                self.qr_texts[tid] = text
                if len(self.qr_texts) > 500:
                    self.qr_texts.clear()
            self._draw_qr_text(annotated, d["bbox"], text)
            now_local = datetime.now(LOCAL_TZ)
            self.get_logger().info(
                f"QR metni çözüldü (hedef #{tid}): \"{text}\""
            )
            print(json.dumps({
                "event_type": "qr_decoded",
                "timestamp": now_local.isoformat(),
                "local_time": now_local.strftime("%H:%M:%S"),
                "track_id": tid,
                "qr_text": text,
                "uav_altitude_m": self.altitude,
                "latitude": self.latitude,
                "longitude": self.longitude,
                "message": f"Kutunun ({tid}) üzerindeki QR kod okundu: \"{text}\"",
            }, ensure_ascii=False), flush=True)

    @staticmethod
    def _draw_qr_text(annotated, bbox, text):
        """Çözülen QR metnini annotated karede kutunun altına yazar."""
        try:
            x1 = int(bbox[0])
            y2 = int(bbox[3])
            y = min(annotated.shape[0] - 4, y2 + 12)
            cv2.putText(annotated, text, (max(0, x1), y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)
        except Exception:
            pass

    def _merge_tile_detections(self, frame, annotated, det_list, effective_conf, frame_area,
                               pose=None):
        """Tile tespitlerini filtreleyip (bbox-frac, başlangıç bastırma, eşik)
        det_list'e ekler ve annotated kareye manuel kutu çizer. Güncellenmiş
        annotated kareyi döndürür."""
        tile_dets = self.run_tiles(frame, min(effective_conf, YOLO_RAW_CONF))
        p = pose or {"lat": self.latitude, "lon": self.longitude, "alt": self.altitude}
        suppress_start = False
        if p.get("lat") is None or p.get("lon") is None:
            suppress_start = True
        else:
            dist_to_start = self.distance_m(39.920782, 32.854115, p["lat"], p["lon"])
            if dist_to_start < 150.0 or (p.get("alt") is not None and p["alt"] < 10.0):
                suppress_start = True
        for td in tile_dets:
            norm_cat = normalize_category(td["cls"])
            is_person = (norm_cat == "person" or td["cls"].lower() in
                         ["person", "human", "pedestrian", "walking person"])
            if is_person:
                min_conf = self.person_conf_min
            else:
                min_conf = max(effective_conf, CLASS_CONF_MIN.get(norm_cat, 0.0))
            if td["conf"] < min_conf:
                continue
            if suppress_start:
                continue
            bbox = td["bbox"]
            if bbox is not None and frame_area > 0:
                area = max(0.0, bbox[2] - bbox[0]) * max(0.0, bbox[3] - bbox[1])
                if area > YOLO_MAX_BBOX_FRAC * frame_area:
                    continue
            # Tam kare yolundaki aynı kapılar burada da geçerli: fiziksel boyut
            # ve kendi gölgesi. (Tile tespitleri plot()'a girmediği için elle.)
            scale = self.px_per_m(frame.shape[1])
            if bbox is not None and scale is not None:
                long_m = max(bbox[2] - bbox[0], bbox[3] - bbox[1]) / scale
                is_person = normalize_category(td["cls"]) == "person"
                min_m = YOLO_MIN_OBJ_M_PERSON if is_person else YOLO_MIN_OBJ_M
                if long_m > YOLO_MAX_OBJ_M or long_m < min_m:
                    self.metrics["sup_size"] += 1
                    continue
            if self.ego_zones is not None and self.ego_zones.is_ego(bbox):
                self.metrics["sup_ego"] += 1
                continue
            # Tam-kare tespitleriyle çakışıyorsa (aynı cisim) atla.
            if any(self._iou(bbox, d.get("bbox")) >= 0.4 for d in det_list):
                continue
            td["category"] = normalize_category(td["cls"])
            # Tile tespitleri de tam-kare yolundakiyle aynı yer koordinatını ve
            # kararlı ID'yi alır; aksi halde haritada İHA konumuna düşüp her
            # karede yeni hedef gibi görünüyorlardı.
            if pose is not None:
                td["lat"], td["lon"] = self.project_to_ground(
                    bbox, frame.shape[1], frame.shape[0], pose
                )
                td["track_id"], td["hits"] = self.assign_object_id(
                    None, td["category"], bbox, td["lat"], td["lon"]
                )
            else:
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

    def assign_object_id(self, track_id: Optional[int], category: str, bbox: Optional[List[float]],
                         obj_lat: Optional[float] = None, obj_lon: Optional[float] = None) -> Tuple[int, int]:
        """Her tespite kararlı bir integer ID ve hits (kaç kez görüldü) atar.

        Sıra: (1) YER KOORDİNATI eşleşmesi — aynı kategoriden bir iz
        YOLO_GEO_MATCH_M metre içindeyse aynı cisimdir; (2) ByteTrack ID;
        (3) görüntü-uzayı IoU fallback'i. Yer eşleşmesi ÖNCE gelir çünkü çıkarım
        ~2 FPS koştuğunda cisim kareler arasında yüzlerce piksel yer değiştirir:
        IoU de ByteTrack de kopar, aynı kutu her karede yeni ID alır (hits hep 1,
        QR defalarca çözülür, tabloda onlarca sahte satır açılır). Yer koordinatı
        kare hızından bağımsızdır. ID asla None kalmaz."""
        now = time.time()

        if obj_lat is not None and obj_lon is not None:
            self.geo_tracks = [t for t in self.geo_tracks
                               if now - t["last_seen"] <= YOLO_GEO_TRACK_TTL]
            best, best_d = None, YOLO_GEO_MATCH_M
            for trk in self.geo_tracks:
                if trk["category"] != category:
                    continue
                d = self.distance_m(trk["lat"], trk["lon"], obj_lat, obj_lon)
                if d < best_d:
                    best, best_d = trk, d
            if best is not None:
                best["hits"] += 1
                best["last_seen"] = now
                # Konumu yumuşat: her yeni gözlem izin konumunu biraz çeker,
                # tek karelik projeksiyon gürültüsü izi kaçırmaz.
                best["lat"] += (obj_lat - best["lat"]) * 0.3
                best["lon"] += (obj_lon - best["lon"]) * 0.3
                return best["id"], best["hits"]
            new_id = self.next_fallback_id
            self.next_fallback_id += 1
            self.geo_tracks.append({
                "id": new_id, "category": category, "lat": obj_lat, "lon": obj_lon,
                "hits": 1, "last_seen": now,
            })
            return new_id, 1

        if track_id is not None:
            hits = self.track_hits.get(track_id, 0) + 1
            self.track_hits[track_id] = hits
            return track_id, hits

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
    def _containment(inner, outer):
        """inner kutusunun alanının ne kadarı outer'ın içinde kalıyor (0-1).

        IoU'dan farkı: boyutları çok farklı kutularda IoU küçük çıkar (küçük kutu
        büyüğün içinde tamamen dursa bile), containment ise 1.0 verir — iç içe
        kutuyu yakalamak için doğru ölçü budur.
        """
        if not inner or not outer:
            return 0.0
        ix1, iy1 = max(inner[0], outer[0]), max(inner[1], outer[1])
        ix2, iy2 = min(inner[2], outer[2]), min(inner[3], outer[3])
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        inner_area = max(0.0, inner[2] - inner[0]) * max(0.0, inner[3] - inner[1])
        return inter / inner_area if inner_area > 0 else 0.0

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
                    "qr_text": d.get("qr_text"),
                    # Hedefin kendi yer koordinatı (geo-projeksiyon). None ise
                    # arayüz eskisi gibi İHA konumuna düşer.
                    "lat": d.get("lat"),
                    "lon": d.get("lon"),
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

    def log_detections(self, det_list, effective_conf, pose=None):
        """Her tespit edilen cisim için Pydantic ile doğrulanmış tek satır JSONL
        kaydı yazar. stdout, app.py tarafından yolo.log dosyasına yönlendirilir.
        `pose` verilirse (karenin başındaki poz) İHA konumu oradan yazılır."""
        now = datetime.now(LOCAL_TZ)
        tracker = "bytetrack" if self.use_tracking else "none"
        pose = pose or {}
        for d in det_list:
            try:
                record = YoloDetectionLog(
                    timestamp=now,
                    local_time=now.strftime("%H:%M"),
                    uav_altitude_m=pose.get("alt", self.altitude),
                    latitude=pose.get("lat", self.latitude),
                    longitude=pose.get("lon", self.longitude),
                    object_latitude=d.get("lat"),
                    object_longitude=d.get("lon"),
                    object=DetectionObjectLog(
                        label=d["cls"],
                        normalized_category=d.get("category", normalize_category(d["cls"])),
                        count=1,
                        confidence=d["conf"],
                    ),
                    track_id=d["track_id"],
                    bbox_xyxy=d["bbox"],
                    qr_text=d.get("qr_text"),
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
    # virgülle ayrık). Liste rotayla BİREBİR eşleşir; kuralı şu: karşılığı
    # rotada olmayan prompt yalnızca sahte pozitif yüzeyidir, o yüzden çıkarılır.
    #
    # 2026-07-17'de rota ölçüme göre sadeleşti (bkz. world dosyasındaki tablo):
    # 15 m nadir'de YOLOE-26s'in ateşlediği TEK kavram 'qr code box' (QR desenli
    # alçak kutu ~0.62, sırt çantası dokusu ~0.50); insan 0.09'da person_conf_min
    # ile geçiyor. Traktör/otomobil/tekne/çadır/varil/koni ve DÜZ turuncu kutu
    # ölçüldü => tespit yok; hepsi rotadan kaldırıldı. Dolayısıyla:
    #   * 'drone/quadcopter/uav' YOK — cisim kaldırıldı ve bu prompt'lar SİHA'nın
    #     kendi gölgesini kilitliyordu (build_20260717_103118: 33/33 tespit gölge).
    #   * 'car/sedan/suv/truck/semi-truck/tractor' YOK — rotada araç kalmadı.
    #     Zaten nadir'de 'car' ateşlemiyordu (gerçek mesh 4.3 m => 0.05) ama
    #     'car' geçmişte zemini tam-kare sanıyordu (build_144152: car:0.57 = %99
    #     kare), yani net sahte pozitif yüzeyiydi.
    #   * 'orange box' YOK — düz turuncu kutular rotadan kaldırıldı.
    # Araç gerekirse: YOLO_TARGET_CLASSES ile eklenebilir (CLASS_NORMALIZATION
    # eşlemeleri duruyor), ama nadir'de ateşlemeyeceğini bilerek.
    # 2026-07-17 rota güncellemesi: stop tabelası ve dama paneli eklendi
    # (ölçümle doğrulandı: stop sign 0.74, checkerboard 0.79 — nadir'de doğru
    # etiketle ateşliyorlar; bkz. world dosyası + scratchpad/probe2).
    # 2026-07-17 güncelleme-2: nişan tahtası ('concentric circles') eklendi —
    # GERÇEK dünyada, rotanın kendi asfaltı üzerinde, üç bakış konumundan
    # ölçüldü (0.57-0.67). CLASS_CONF_MIN['bullseye']=0.30 ile birlikte gelir.
    # Elenen adaylar (hepsi ölçüldü, tekrar denemeye değmez):
    #   * 'crosswalk' / 'barcode' — desen güçlü ateşliyor ama 'qr code box'a
    #     karışıp QR kutularla aynı kategoriye düşüyor.
    #   * 'helicopter landing pad' (çapraz X işareti) — ÇİMENDE 0.397, ama
    #     rotanın asfaltı üzerinde 0.063'e düşüyor; bu rotada kullanılamaz.
    #   * 'warning sign' (tehlike üçgeni) — üçgeni 0.30-0.37 ile buluyor ama
    #     STOP TABELASINI da 0.37 ile yakalıyor; aynı bantta oldukları için
    #     hiçbir eşik ayıramıyor. Alternatif promptlar da ölçüldü, hepsi ya
    #     zayıf ya başka cismi çalıyor (bkz. make_route_object_textures.py).
    #   * 'target' — sırt çantasını çalıyor, yerine 'concentric circles'.
    # 2026-07-17 güncelleme-4: DAF tır yerine Prius Hybrid kondu (route_prius_1);
    # tepeden 'car' ateşleyip ateşlemediğini GERÇEK nadir kamerayla ölçmek için
    # 'car,sedan,automobile' eklendi (CLASS_NORMALIZATION -> vehicle). 'car' geçmişte
    # zemini tam-kare 'car:0.57' sanıyordu ama YOLO_MAX_BBOX_FRAC (0.40) + fiziksel
    # boyut kapısı (maks 9 m) bu tam-kare hayaleti eliyor. Ateşlemezse bu üç prompt
    # geri çıkarılır (bkz. memory siha-nadir-arac-taninmiyor).
    # 2026-07-18 güncelleme-5: İNSAN AKTÖRLERİ ROTADAN KALDIRILDI, yerlerine
    # iki renkli kutu (kırmızı/mavi) kondu. Kural gereği rotada karşılığı
    # kalmayan person promptları listeden çıkarıldı (yalnızca sahte pozitif
    # yüzeyiydiler). Renkli kutular ölçülmüş QR desenini taşıdığı için mevcut
    # 'qr code box' promptlarıyla ateşler; RENK ayrımı prompt'la değil
    # classify_box_colors (HSV baskın renk) ile yapılır → red_box / blue_box.
    default_classes = ("box,crate,cargo box,qr code box,"
                       "stop sign,checkerboard")
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
