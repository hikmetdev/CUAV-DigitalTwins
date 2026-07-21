import sys
import os
import math
import json
os.environ["RCUTILS_LOGGING_SEVERITY_THRESHOLD"] = "ERROR"
# NOT: Takip haritası artık QtWebEngine/Leaflet değil, yerli QPainter widget'ı
# (interface/harita.py). CDN'e ve Chromium renderer'ına bağımlılık kalmadığı
# için harita ağ/GPU/heap sorunlarında boş kalmaz; 8 GB RAM'de en büyük
# RAM/çökme kaynağı (WebEngine) da devreden çıkmış olur.

from datetime import datetime
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def _load_env_file(path):
    """Basit .env yükleyici (ekstra bağımlılık yok). KEY=VALUE satırlarını
    okuyup ortama ekler; zaten tanımlı değişkenleri ezmez. AI Copilot'un
    GEMINI_API_KEY/GEMINI_MODEL ayarları buradan gelir. threads importundan
    ÖNCE çağrılmalı (gemini_t modül yüklenirken env okur)."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, value = line.partition("=")
                key, value = key.strip(), value.strip().strip('"').strip("'")
                if key and key not in os.environ:
                    os.environ[key] = value
    except FileNotFoundError:
        pass


_load_env_file(os.path.join(BASE_DIR, "config", ".env"))
LOGS_DIR = os.path.join(BASE_DIR, "logs")
# Her çalıştırma (build) için ayrı log klasörü, en tepede oluşturulur ki ROS
# log'ları da (logs/ros yerine) bu klasörün içinde tutulabilsin.
BUILD_DIR = os.path.join(LOGS_DIR, f"build_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
os.makedirs(BUILD_DIR, exist_ok=True)
ROS_LOG_DIR = os.path.join(BUILD_DIR, "ros")
WORLD_FILE = os.path.join(BASE_DIR, "worlds", "vtail_straight_route.sdf")
SITL_PARAM_FILE = os.path.join(BASE_DIR, "config", "straight_route_sitl.param")
GAZEBO_BOOT_DELAY_SEC = 1
AUTONOMOUS_START_DELAY_MS = 7000
STRAIGHT_ROUTE_HEADING_DEG = 0  # 0° = Kuzey; uçak pistin uzun ekseni boyunca gider
ROUTE_STEP_DEG = 0.000045
ROUTE_REACHED_DEG = 0.00006
START_LAT = 39.920782
START_LON = 32.854115
# Harita önizleme hedefleri — worlds/vtail_straight_route.sdf'teki cisimlerle
# birebir aynı (id, tür, kalkıştan mesafe). World değişirse burası da güncellenir.
TARGET_DETECTIONS = [
    {"id": "T-01", "type": "Kutu",           "conf": 0.90, "time": "12:30:00", "distance_m": 200.0},
    {"id": "T-02", "type": "Stop Tabelası",  "conf": 0.90, "time": "12:32:00", "distance_m": 280.0},
    {"id": "T-03", "type": "Kutu",           "conf": 0.90, "time": "12:35:00", "distance_m": 440.0},
    {"id": "T-04", "type": "Dama Paneli",    "conf": 0.90, "time": "12:36:00", "distance_m": 520.0},
    {"id": "T-05", "type": "Kutu",           "conf": 0.90, "time": "12:37:00", "distance_m": 600.0},
    {"id": "T-06", "type": "Kırmızı Kutu",   "conf": 0.90, "time": "12:38:00", "distance_m": 675.0},
    {"id": "T-07", "type": "Kutu",           "conf": 0.90, "time": "12:39:00", "distance_m": 740.0},
    {"id": "T-08", "type": "Stop Tabelası",  "conf": 0.90, "time": "12:40:00", "distance_m": 810.0},
    {"id": "T-09", "type": "Mavi Kutu",      "conf": 0.90, "time": "12:41:00", "distance_m": 855.0},
]
os.makedirs(ROS_LOG_DIR, exist_ok=True)
# Alt-süreçler ve thread'ler bu env'i devralıp aynı build/ros klasörünü kullanır.
os.environ["ROS_LOG_DIR"] = ROS_LOG_DIR
import time
import random
from collections import deque
from datetime import datetime
import subprocess
import signal
import rclpy
import logging
import traceback

import cv2  

from PySide6.QtCore import Qt, QTimer, QUrl, Slot, QSize, Signal
from PySide6.QtGui import QColor, QFont, QTextCursor, QImage
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QPushButton, QLineEdit, QTextBrowser, 
    QTableWidget, QTableWidgetItem, QHeaderView, QSplitter, 
    QFrame, QPlainTextEdit, QProgressBar, QFileDialog, QTabWidget
)
from styles import DARK_THEME_QSS
from interface import (CompassWidget, CameraSimulatorWidget, TimelineWidget,
                       TacticalMapWidget, McpWorkflowTrackerWidget, TelemetryChartsPanel,
                       TacticalReportPdf)
from threads import (CameraStreamThread, TelemetryStreamThread, DetectionStreamThread,
                     GeminiChatThread, VideoYoloThread, McpClient, McpError)
from scene_db import SceneDatabase

# ----------------------------------------------------------------------
# LOGGING SETUP
# ----------------------------------------------------------------------
# BUILD_DIR en tepede oluşturuldu (ROS_LOG_DIR onun içine bağlandığı için).
# Her bileşenin (gazebo, ardupilot, mavros, bridge, siha_control, yolo, app)
# terminal çıktısı bu klasörde KENDİ dosyasında, karışmadan tutulur.
APP_LOG = os.path.join(BUILD_DIR, "app.log")


def open_component_log(filename, title):
    """Build klasöründe bir bileşen için ayrı log dosyası açar, başlık yazıp
    dosya handle'ını döner. Alt-süreçlerin stdout/stderr'i bu handle'a
    yönlendirilir; böylece her bileşenin terminal çıktısı temiz ve tek dosyada
    toplanır. Dosya açılamazsa sessizce subprocess.DEVNULL döner."""
    path = os.path.join(BUILD_DIR, filename)
    try:
        f = open(path, "a", encoding="utf-8")
        f.write("=" * 60 + "\n")
        f.write(f"=== {title} ===\n")
        f.write(f"=== Başlangıç: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        f.write("=" * 60 + "\n\n")
        f.flush()
        return f
    except Exception:
        return subprocess.DEVNULL


logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] (%(filename)s:%(lineno)d): %(message)s',
    handlers=[
        logging.FileHandler(APP_LOG, encoding='utf-8')
    ]
)
logger = logging.getLogger("GCS")

def destination_point(lat_deg, lon_deg, bearing_deg, distance_m):
    radius_m = 6371000.0
    angular_distance = distance_m / radius_m
    bearing = math.radians(bearing_deg)
    lat1 = math.radians(lat_deg)
    lon1 = math.radians(lon_deg)
    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_distance)
        + math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular_distance) * math.cos(lat1),
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)

def build_forward_detections(home_lat, home_lon, heading_deg):
    detections = []
    for det in TARGET_DETECTIONS:
        lat, lon = destination_point(home_lat, home_lon, heading_deg, det["distance_m"])
        item = det.copy()
        item["lat"] = lat
        item["lon"] = lon
        detections.append(item)
    return detections

# Uncaught Exception Hook to log GUI crashes and tracebacks
def log_uncaught_exception(exctype, value, tb):
    tb_text = "".join(traceback.format_exception(exctype, value, tb))
    logger.critical(f"Unhandled Exception occurred:\n{tb_text}")
    sys.__excepthook__(exctype, value, tb)

sys.excepthook = log_uncaught_exception

# ----------------------------------------------------------------------
# 5. MAIN GROUND CONTROL STATION WINDOW
# ----------------------------------------------------------------------
class GroundControlStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGENTIC DIGITAL-TWIN GCS | Yer Kontrol İstasyonu")
        self.resize(1200, 760)
        self.setStyleSheet(DARK_THEME_QSS)
        
        # Initial State Parameters
        self.telemetry = {
            "alt": 0.0,
            "speed": 0.0,
            "battery": 100,
            "voltage": 25.2,
            "mode": "DISCONNECTED",
            "lat": START_LAT,
            "lon": START_LON,
            "heading": 0,
            "sats": 0
        }
        
        # AI Copilot'un "geçmiş telemetri" aracı için 1 Hz örnekleme tamponu
        # (update_telemetry_loop doldurur). 1800 örnek ≈ 30 dk uçuş.
        self.mission_start_ts = time.time()
        self.takeoff_start_ts = None
        self.flight_started = False
        self.telemetry_history = deque(maxlen=1800)
        # Kat edilen toplam yol (m) ve son GPS sabiti — bkz.
        # update_travelled_distance. Düz çizgi uzaklığı DEĞİL, adım adım birikir.
        self.travel_m = 0.0
        self._last_travel_fix = None

        self.detections = []
        self.waypoints = [{"lat": START_LAT, "lon": START_LON, "name": "START"}] + [
            {"lat": det["lat"], "lon": det["lon"], "name": det["id"]} for det in build_forward_detections(START_LAT, START_LON, STRAIGHT_ROUTE_HEADING_DEG)
        ]
        self.path_index = 1
        
        self.timeline_events = [
            {"time": datetime.now().strftime("%H:%M:%S"), "type": "command", "desc": "Sistem Başlatıldı"}
        ]
        
        self.vlm_summaries = [
            "Sistemler kararlı. Kamera ve telemetri yayını bekleniyor..."
        ]
        self.vlm_index = 0

        # Sahne özeti veritabanı: panelde gösterilen her özet database/<build>.db
        # SQLite dosyasına da yazılır (dosya adı = bu koşunun log klasörü adı).
        # DB açılamazsa uçuş arayüzü etkilenmez, kayıt sessizce devre dışı kalır.
        try:
            self.scene_db = SceneDatabase(
                os.path.join(BASE_DIR, "database"), os.path.basename(BUILD_DIR)
            )
            # NOT: logs_terminal henüz kurulmadı; terminal yerine dosya loguna yaz.
            logger.info(
                f"Sahne özeti veritabanı hazır: database/{os.path.basename(BUILD_DIR)}.db"
            )
        except Exception as exc:
            self.scene_db = None
            logger.warning(f"Sahne özeti veritabanı açılamadı: {exc}")

        # AI Copilot (Gemini) durumu: canlı tool-çağıran asistan.
        self.gemini_api_key = os.environ.get("GEMINI_API_KEY", "").strip()
        self.chat_thread = None          # aktif GeminiChatThread (tekil)
        self.chat_history_ctx = []       # Gemini konuşma bağlamı (role/parts)
        # MCP araç sunucusu istemcisi: araçlar mcp_server.py alt sürecinde
        # yaşar; ilk sorguda başlatılır (tembel) ve uygulama ömrünce paylaşılır.
        self.mcp_client = None
        self.camera_thread = None
        self.video_yolo_thread = None
        # Aktif kamera akışı: başlangıçta ham alt kamera; YOLO başlayınca kutulu
        # /yolo/image_annotated akışına geçilir (switch_camera_source ile).
        self.current_camera_source = "/bottom_camera/image"
        
        # Ensure a clean and fresh yolo.log without simulation data
        yolo_log_path = os.path.join(BUILD_DIR, "yolo.log")
        if os.path.exists(yolo_log_path):
            try:
                os.remove(yolo_log_path)
            except Exception:
                pass
        
        # Build UI layout
        self.setup_ui()

        # GAZEBO VE ARDUPİLOT BAĞLANTISI
        self.gazebo_process = None
        self.ardupilot_process = None
        self.start_simulation()
        
        # Telemetry thread and flag
        self.use_simulation = False
        self.telemetry_thread = TelemetryStreamThread()
        self.telemetry_thread.telemetry_received.connect(self.handle_real_telemetry)
        self.telemetry_thread.log_signal.connect(self.add_terminal_log)
        self.telemetry_thread.start()

        # Aktif Kamera & Ekran Modu: "gazebo" veya "video"
        self.active_camera_mode = "gazebo"

        # Gazebo kamera akışı YOLO tespit verileri & geçmişi
        self.gazebo_det_counter = 0
        self.gazebo_live_detections = []
        self.gazebo_detection_log = []
        self.gazebo_track_rows = {}

        # Canlı video (drone.mp4) YOLO tespit verileri & geçmişi
        self.video_det_counter = 0
        self.video_live_detections = []
        self.video_detection_log = []
        self.video_track_rows = {}

        # Aktif ekranda gösterilen tespit verisi referansları
        self._det_counter = self.gazebo_det_counter
        self.live_detections = self.gazebo_live_detections
        self.detection_log = self.gazebo_detection_log
        self._track_rows = self.gazebo_track_rows

        self.detection_thread = DetectionStreamThread()
        self.detection_thread.detection_received.connect(self.handle_detection)
        self.detection_thread.log_signal.connect(self.add_terminal_log)
        self.detection_thread.start()

        # Auto-connect camera stream after a short delay
        QTimer.singleShot(1500, self.toggle_camera_stream)
        
        # Core Update Timers
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.update_telemetry_loop)
        self.telemetry_timer.start(1000) # Update every 1 second
        
        self.vlm_timer = QTimer(self)
        self.vlm_timer.timeout.connect(self.update_vlm_summary)
        self.vlm_timer.start(16000) # Update VLM explanation every 16 seconds
        
        # Harita kurulumu, TacticalMapWidget loadFinished sinyalini yayınlayınca yapılır

        # Uçak simülasyon başladığında doğrudan havalanmaz (standby).
        # Operatör AI Copilot'a "uçak uçmaya hazır", "kalkışa geç" vb. yazdığında havalanır.
        self.add_terminal_log("Uçak pistte hazır (standby). Uçuşa başlamak için AI Copilot'a 'uçak uçmaya hazır' yazın.", "INFO")
        
        # Populate widgets initial data
        self.update_detections_table()
        if self.timeline_widget:
            self.timeline_widget.set_events(self.timeline_events)
        self.update_vlm_summary()
        self.add_terminal_log("MAVLink GCS Yer Kontrol İstasyonu Bağlantısı Kuruldu (TCP:5760).", "INFO")
        self.add_terminal_log("YOLOE & Vision-Language Model entegrasyonu aktif.", "INFO")

    def start_simulation(self):
        self.add_terminal_log("Simülasyon başlatılıyor...", "INFO")
        
        # Prepare a clean environment for subprocesses to avoid OpenCV Qt plugin pollution
        sub_env = os.environ.copy()
        if "QT_QPA_PLATFORM_PLUGIN_PATH" in sub_env:
            del sub_env["QT_QPA_PLATFORM_PLUGIN_PATH"]
        gazebo_models_path = "/home/delpiyero/ardupilot/Tools/sitl_models/Gazebo/models"
        existing_resource_path = sub_env.get("GZ_SIM_RESOURCE_PATH", "")
        if gazebo_models_path not in existing_resource_path.split(":"):
            sub_env["GZ_SIM_RESOURCE_PATH"] = (
                f"{gazebo_models_path}:{existing_resource_path}"
                if existing_resource_path
                else gazebo_models_path
            )
        existing_ign_resource_path = sub_env.get("IGN_GAZEBO_RESOURCE_PATH", "")
        if gazebo_models_path not in existing_ign_resource_path.split(":"):
            sub_env["IGN_GAZEBO_RESOURCE_PATH"] = (
                f"{gazebo_models_path}:{existing_ign_resource_path}"
                if existing_ign_resource_path
                else gazebo_models_path
            )
        
        # Gazebo terminal çıktısı için ayrı log dosyası
        gazebo_log = open_component_log("gazebo.log", "GAZEBO SIMULATION")

        gazebo_cmd = ["gz", "sim", "-s", "-r", "-v", "4", WORLD_FILE]
        
        try:
            self.gazebo_process = subprocess.Popen(
                gazebo_cmd,
                stdout=gazebo_log,
                stderr=gazebo_log,
                env=sub_env,
                preexec_fn=os.setsid
            )
            self.add_terminal_log("Gazebo Simülasyonu arka planda (headless) başlatıldı.", "INFO")
            self.add_terminal_log("Gazebo world yüklemesi için hızlı başlangıç beklemesi uygulanıyor...", "INFO")
            time.sleep(GAZEBO_BOOT_DELAY_SEC)
        except Exception as e:
            self.add_terminal_log(f"Gazebo başlatılamadı: {e}", "WARN")

        # ArduPilot SITL terminal çıktısı için ayrı log dosyası
        ardupilot_log = open_component_log("ardupilot_sitl.log", "ARDUPILOT SITL")

        ardupilot_dir = "/home/delpiyero/ardupilot"
        ardupilot_cmd = [
            "sim_vehicle.py",
            "-N",
            "-v", "ArduPlane",
            "-f", "plane",
            "--model", "JSON",
            "--add-param-file=/home/delpiyero/ardupilot/Tools/sitl_models/Gazebo/config/mini_talon_vtail.param",
            f"--add-param-file={SITL_PARAM_FILE}",
            "--no-mavproxy",
            "-l", f"{START_LAT},{START_LON},584,{STRAIGHT_ROUTE_HEADING_DEG}"
        ]
        try:
            self.ardupilot_process = subprocess.Popen(
                ardupilot_cmd,
                cwd=ardupilot_dir,
                stdout=ardupilot_log,
                stderr=ardupilot_log,
                env=sub_env,
                preexec_fn=os.setsid
            )
            self.add_terminal_log("ArduPilot SITL başlatıldı (MavProxy devre dışı, doğrudan TCP bağlantı).", "INFO")
        except Exception as e:
            self.add_terminal_log(f"ArduPilot SITL başlatılamadı: {e}", "WARN")

        # ROS-Gazebo köprüsü terminal çıktısı için ayrı log dosyası
        bridge_log = open_component_log("ros_gz_bridge.log", "ROS-GAZEBO BRIDGE")

        try:
            bridge_cmd = [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/bottom_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/bottom_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
            ]
            self.bridge_process = subprocess.Popen(
                bridge_cmd,
                stdout=bridge_log,
                stderr=bridge_log,
                env=sub_env,
                preexec_fn=os.setsid
            )
            self.add_terminal_log("ROS-Gazebo Köprüsü başlatıldı (camera/image -> ROS 2).", "INFO")
        except Exception as e:
            self.add_terminal_log(f"ROS-Gazebo Köprüsü başlatılamadı: {e}", "WARN")

        # MAVROS terminal çıktısı için ayrı log dosyası
        mavros_log = open_component_log("mavros.log", "MAVROS")

        try:
            mavros_cmd = [
                "ros2", "launch", "mavros", "apm.launch",
                "fcu_url:=tcp://127.0.0.1:5760"
            ]
            self.mavros_process = subprocess.Popen(
                mavros_cmd,
                stdout=mavros_log,
                stderr=mavros_log,
                env=sub_env,
                preexec_fn=os.setsid
            )
            self.add_terminal_log("MAVROS Bağlantısı başlatılıyor (TCP:5760).", "INFO")
            # Deactivate mock simulation as we are starting the real flight stack
            self.use_simulation = False
        except Exception as e:
            self.add_terminal_log(f"MAVROS başlatılamadı: {e}", "WARN")

    def handle_real_telemetry(self, data):
        self.use_simulation = False
        self.telemetry.update(data)

    # ------------------------------------------------------------------
    # UI SETUP & PANEL ASSEMBLY
    # ------------------------------------------------------------------
    def setup_ui(self):
        # Base central widget and vertical box
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 1. TOP HEADER PANEL
        header_widget = QWidget()
        header_widget.setFixedHeight(50)
        header_widget.setStyleSheet("background-color: #090d16; border-bottom: 1px solid #1e293b;")
        header_layout = QHBoxLayout(header_widget)
        header_layout.setContentsMargins(15, 0, 15, 0)
        
        logo_label = QLabel("<span style='color:#10b981; font-weight:normal;'>AD-GCS v1.4</span>")
        logo_label.setStyleSheet("font-size: 13px; font-weight: bold; font-family: 'Roboto Mono'; color: #f8fafc;")
        header_layout.addWidget(logo_label)
        
        # Connection status badges (hidden as per user request)
        self.conn_badge = QLabel("SİMÜLASYON VERİSİ")
        self.recording_badge = QLabel("● KAYDEDİLİYOR")
        
        header_layout.addStretch()
        
        # Clock & Toggle Logs Button
        time_layout = QHBoxLayout()
        time_layout.setSpacing(15)
        
        self.clock_label = QLabel("13:07:07")
        self.clock_label.setStyleSheet("font-family: 'Roboto Mono'; font-size: 11px; color: #94a3b8;")
        time_layout.addWidget(self.clock_label)
        
        self.toggle_logs_btn = QPushButton("Logları Aç")
        self.toggle_logs_btn.setProperty("class", "btn-secondary")
        self.toggle_logs_btn.setFixedWidth(90)
        self.toggle_logs_btn.clicked.connect(self.toggle_bottom_logs)
        time_layout.addWidget(self.toggle_logs_btn)
        
        header_layout.addLayout(time_layout)
        main_layout.addWidget(header_widget)
        
        # 2. MAIN HORIZONTAL SPLITTER (GCS grid vs Sidebar AI Chat)
        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.setHandleWidth(1)
        main_splitter.setStyleSheet("QSplitter::handle { background-color: #1e293b; }")
        
        # LEFT/MIDDLE WORKSPACE (Vertical Splitter: Top widgets, Map, Logs)
        workspace_splitter = QSplitter(Qt.Vertical)
        workspace_splitter.setHandleWidth(1)
        workspace_splitter.setStyleSheet("QSplitter::handle { background-color: #1e293b; }")
        
        # TOP ROW split (Camera stream vs Telemetry)
        top_row_splitter = QSplitter(Qt.Horizontal)
        top_row_splitter.setHandleWidth(1)
        
        # Panel 1: Camera Feed YOLO
        camera_panel = QFrame()
        camera_panel.setProperty("class", "panel")
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(0, 0, 0, 0)
        camera_layout.setSpacing(0)
        
        # Title bar
        cam_title_bar = QFrame()
        cam_title_bar.setProperty("class", "panel-header")
        cam_title_bar.setFixedHeight(28)
        cam_title_layout = QHBoxLayout(cam_title_bar)
        cam_title_layout.setContentsMargins(10, 0, 10, 0)
        
        cam_title_lbl = QLabel("CANLI GÖRÜNTÜ AKIŞI & TESPİTLER")
        cam_title_lbl.setProperty("class", "title-label")
        cam_title_layout.addWidget(cam_title_lbl)
        cam_title_layout.addStretch()
        
        camera_layout.addWidget(cam_title_bar)
        
        # Sub-header: Stream Configuration Bar
        cam_config_bar = QFrame()
        cam_config_bar.setStyleSheet("background-color: #0b111c; border-bottom: 1px solid #1e293b;")
        cam_config_bar.setFixedHeight(30)
        cam_config_layout = QHBoxLayout(cam_config_bar)
        cam_config_layout.setContentsMargins(10, 0, 10, 0)
        cam_config_layout.setSpacing(8)
        
        # Gazebo akışında izlenecek görüş: alt kamera / ön kamera. Topic adı
        # sabit DEĞİL — alt kamera, YOLO düğümü çalışıyorsa kutulanmış
        # /yolo/image_annotated akışına, çalışmıyorsa ham /bottom_camera/image
        # akışına bağlanır (camera_topic_for). Eskiden liste doğrudan topic
        # tutuyordu ve açılıştaki /bottom_camera/image listede olmadığı için
        # buton etiketi ile gerçek akış birbirini tutmuyordu.
        self.camera_views = [
            ("alt", "Alt Kamera"),
            ("on", "Ön Kamera"),
        ]
        self.camera_view_index = 0
        self.btn_camera_toggle = QPushButton("Kamera: Alt Kamera")
        self.btn_camera_toggle.setFixedHeight(18)
        self.btn_camera_toggle.setStyleSheet("font-size: 9px; padding: 0 10px; background-color: #0284c7; color: white; border: none; font-weight: bold; border-radius: 2px;")
        self.btn_camera_toggle.clicked.connect(self.toggle_camera_view)
        cam_config_layout.addWidget(self.btn_camera_toggle)

        # Kaynak geçiş butonu: Gazebo simülasyon kamerası <-> yerel drone.mp4
        # video akışı arasında geçiş yapar.
        self.btn_video_toggle = QPushButton("Video'ya Geç")
        self.btn_video_toggle.setFixedHeight(18)
        self.btn_video_toggle.setStyleSheet("font-size: 9px; padding: 0 10px; background-color: #10b981; color: white; border: none; font-weight: bold; border-radius: 2px;")
        self.btn_video_toggle.clicked.connect(self.toggle_video_yolo)
        cam_config_layout.addWidget(self.btn_video_toggle)

        cam_config_layout.addStretch()
        camera_layout.addWidget(cam_config_bar)
        
        # Canvas Simulator
        self.camera_sim = CameraSimulatorWidget()
        camera_layout.addWidget(self.camera_sim)
        
        # Canlı Tespitler Table
        self.detections_table = QTableWidget()
        self.detections_table.setColumnCount(5)
        self.detections_table.setHorizontalHeaderLabels(["ID", "Nesne Tipi", "Güven Skoru", "Zaman", "GPS Koordinatı"])
        self.detections_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.detections_table.setFixedHeight(120)
        self.detections_table.itemClicked.connect(self.handle_table_row_click)
        camera_layout.addWidget(self.detections_table)
        
        top_row_splitter.addWidget(camera_panel)
        
        # Panel 2: Telemetry dashboard (Right side of top row)
        telemetry_panel = QFrame()
        telemetry_panel.setProperty("class", "panel")
        telemetry_layout = QVBoxLayout(telemetry_panel)
        telemetry_layout.setContentsMargins(0, 0, 0, 0)
        telemetry_layout.setSpacing(0)
        
        tel_title_bar = QFrame()
        tel_title_bar.setProperty("class", "panel-header")
        tel_title_bar.setFixedHeight(28)
        tel_title_layout = QHBoxLayout(tel_title_bar)
        tel_title_layout.setContentsMargins(10, 0, 10, 0)
        
        tel_title_lbl = QLabel("İHA CANLI TELEMETRİ GÖSTERGELERİ")
        tel_title_lbl.setProperty("class", "title-label")
        tel_title_layout.addWidget(tel_title_lbl)
        tel_title_layout.addStretch()
        
        telemetry_layout.addWidget(tel_title_bar)
        
        # Telemetry Tabbed Container (Dark Navy & Emerald Green Theme)
        self.telemetry_tab_widget = QTabWidget()
        self.telemetry_tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: none;
                background-color: #030712;
            }
            QTabBar::tab {
                background-color: #0b111c;
                color: #94a3b8;
                font-family: 'Roboto Mono';
                font-size: 10px;
                font-weight: bold;
                padding: 6px 12px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                border: 1px solid #1e293b;
                border-bottom: none;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: #0d1522;
                color: #10b981;
                border-bottom: 2px solid #10b981;
            }
            QTabBar::tab:hover {
                color: #e2e8f0;
            }
        """)

        # ------------------- 1. Sekme (Anlık Telemetri) -------------------
        tel_items_widget = QWidget()
        tel_items_layout = QVBoxLayout(tel_items_widget)
        tel_items_layout.setContentsMargins(10, 8, 10, 8)
        tel_items_layout.setSpacing(8)
        
        # Altitude Card
        self.lbl_alt = QLabel("İRTİFA (ALT): <span style='color:#06b6d4; font-weight:bold;'>15.0 m</span>")
        self.lbl_alt.setStyleSheet("font-family: 'Roboto Mono'; font-size: 11px; background-color:#0d1522; padding:6px 10px; border-radius:4px; border-left: 3px solid #06b6d4;")
        self.lbl_alt.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        tel_items_layout.addWidget(self.lbl_alt, 1)
        
        # Speed Card
        self.lbl_speed = QLabel("HIZ (VELOCITY): <span style='color:#06b6d4; font-weight:bold;'>12.8 m/s</span>")
        self.lbl_speed.setStyleSheet("font-family: 'Roboto Mono'; font-size: 11px; background-color:#0d1522; padding:6px 10px; border-radius:4px; border-left: 3px solid #06b6d4;")
        self.lbl_speed.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        tel_items_layout.addWidget(self.lbl_speed, 1)
        
        # Battery Card
        bat_widget = QWidget()
        # Kart kenar çizgisi de doluluk kademesiyle birlikte renklenir (bkz.
        # update_telemetry_loop → BATTERY_TIERS), bu yüzden referansı saklanır.
        self.bat_card = bat_widget
        bat_widget.setStyleSheet("background-color:#0d1522; border-radius:4px; border-left: 3px solid #10b981;")
        bat_lyt = QVBoxLayout(bat_widget)
        bat_lyt.setContentsMargins(10, 6, 10, 6)
        bat_lyt.setSpacing(3)
        
        self.lbl_battery = QLabel("BATARYA (BAT): <span style='color:#10b981; font-weight:bold;'>100%</span> (25.2 V)")
        self.lbl_battery.setStyleSheet("font-family: 'Roboto Mono'; font-size: 11px; background:none;")
        bat_lyt.addWidget(self.lbl_battery)
        
        self.bar_battery = QProgressBar()
        self.bar_battery.setValue(100)
        self.bar_battery.setTextVisible(False)
        self.bar_battery.setFixedHeight(5)
        self.bar_battery.setStyleSheet("QProgressBar { background-color: #030712; border-radius: 2px; } QProgressBar::chunk { background-color: #10b981; }")
        bat_lyt.addWidget(self.bar_battery)
        tel_items_layout.addWidget(bat_widget, 1)
        
        # Mode Card
        self.lbl_mode = QLabel("UÇUŞ MODU: <span style='color:#ef4444; font-weight:bold;'>DISCONNECTED</span>")
        self.lbl_mode.setStyleSheet("font-family: 'Roboto Mono'; font-size: 11px; background-color:#0d1522; padding:6px 10px; border-radius:4px; border-left: 3px solid #f59e0b;")
        self.lbl_mode.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        tel_items_layout.addWidget(self.lbl_mode, 1)
        
        # Heading & Compass Card
        heading_card = QWidget()
        heading_card.setStyleSheet("background-color:#0d1522; border-radius:4px; border-left: 3px solid #06b6d4;")
        heading_layout = QHBoxLayout(heading_card)
        heading_layout.setContentsMargins(10, 4, 10, 4)
        
        self.lbl_heading = QLabel(f"YÖNELİM: <span style='color:#06b6d4; font-weight:bold;'>{STRAIGHT_ROUTE_HEADING_DEG}° (Yaw)</span>")
        self.lbl_heading.setStyleSheet("font-family: 'Roboto Mono'; font-size: 11px; background:none;")
        self.lbl_heading.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        heading_layout.addWidget(self.lbl_heading)
        heading_layout.addStretch()
        
        self.compass_widget = CompassWidget()
        self.compass_widget.set_heading(STRAIGHT_ROUTE_HEADING_DEG)
        heading_layout.addWidget(self.compass_widget)
        tel_items_layout.addWidget(heading_card, 1)
        
        # GPS Card
        self.lbl_gps = QLabel("GPS KOORDİNAT:<br><span style='color:#f8fafc;'>LAT: 39.920782<br>LON: 32.854115</span>")
        self.lbl_gps.setStyleSheet("font-family: 'Roboto Mono'; font-size: 10px; background-color:#0d1522; padding:6px 10px; border-radius:4px; border-left: 3px solid #64748b;")
        self.lbl_gps.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        tel_items_layout.addWidget(self.lbl_gps, 1)

        # Uçuş Süresi ve Kat Edilen Mesafe (Tek Kartta Yan Yana)
        self.lbl_flight_dist = QLabel("UÇUŞ SÜRESİ: <span style='color:#06b6d4; font-weight:bold;'>00:00</span> | MESAFE: <span style='color:#f59e0b; font-weight:bold;'>0 m</span>")
        self.lbl_flight_dist.setStyleSheet("font-family: 'Roboto Mono'; font-size: 11px; background-color:#0d1522; padding:6px 10px; border-radius:4px; border-left: 3px solid #10b981;")
        self.lbl_flight_dist.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        tel_items_layout.addWidget(self.lbl_flight_dist, 1)

        self.telemetry_tab_widget.addTab(tel_items_widget, "Anlık Telemetri")

        # ------------------- 2. Sekme (Telemetri Grafikleri 2x2) -------------------
        self.telemetry_chart = TelemetryChartsPanel()
        self.telemetry_tab_widget.addTab(self.telemetry_chart, "Telemetri Grafikleri")

        telemetry_layout.addWidget(self.telemetry_tab_widget)
        top_row_splitter.addWidget(telemetry_panel)
        
        workspace_splitter.addWidget(top_row_splitter)
        
        # MIDDLE ROW Split (Map vs Timeline/VLM)
        middle_row_splitter = QSplitter(Qt.Horizontal)
        middle_row_splitter.setHandleWidth(1)
        
        # Panel 3: Map View
        map_panel = QFrame()
        map_panel.setProperty("class", "panel")
        map_panel_layout = QVBoxLayout(map_panel)
        map_panel_layout.setContentsMargins(0, 0, 0, 0)
        map_panel_layout.setSpacing(0)
        
        map_title_bar = QFrame()
        map_title_bar.setProperty("class", "panel-header")
        map_title_bar.setFixedHeight(28)
        map_title_layout = QHBoxLayout(map_title_bar)
        map_title_layout.setContentsMargins(10, 0, 10, 0)
        
        map_title_lbl = QLabel("TAKTIKSEL TAKİP HARİTASI")
        map_title_lbl.setProperty("class", "title-label")
        map_title_layout.addWidget(map_title_lbl)
        map_title_layout.addStretch()
        
        # Automatic tracking status badge
        map_auto_track_lbl = QLabel("● OTOMATİK TAKİP AKTİF")
        map_auto_track_lbl.setProperty("class", "badge-green")
        map_title_layout.addWidget(map_auto_track_lbl)
        
        map_panel_layout.addWidget(map_title_bar)
        
        # Yerli taktik harita widget'ı (WebEngine/Leaflet yerine; offscreen
        # modda da sorunsuz çalıştığı için ayrı headless mock'a gerek yok).
        self.map_view = TacticalMapWidget(START_LAT, START_LON)
        self.map_loaded = False
        self.map_view.loadFinished.connect(self.on_map_load_finished)
        map_panel_layout.addWidget(self.map_view)
        
        middle_row_splitter.addWidget(map_panel)
        
        # Panel 4: Timeline & VLM Scene Description
        timeline_vlm_panel = QFrame()
        timeline_vlm_panel.setProperty("class", "panel")
        timeline_vlm_layout = QVBoxLayout(timeline_vlm_panel)
        timeline_vlm_layout.setContentsMargins(0, 0, 0, 0)
        timeline_vlm_layout.setSpacing(0)
        
        tv_title_bar = QFrame()
        tv_title_bar.setProperty("class", "panel-header")
        tv_title_bar.setFixedHeight(28)
        tv_title_layout = QHBoxLayout(tv_title_bar)
        tv_title_layout.setContentsMargins(10, 0, 10, 0)
        
        tv_title_lbl = QLabel("SAHNE ÖZETİ")
        tv_title_lbl.setProperty("class", "title-label")
        tv_title_layout.addWidget(tv_title_lbl)
        tv_title_layout.addStretch()
        
        timeline_vlm_layout.addWidget(tv_title_bar)
        
        tv_content = QWidget()
        tv_content_lyt = QHBoxLayout(tv_content)
        tv_content_lyt.setContentsMargins(8, 6, 8, 6)
        tv_content_lyt.setSpacing(8)
        
        # Sol Taraf: VLM Sahne Analizi
        vlm_container = QWidget()
        vlm_lyt = QVBoxLayout(vlm_container)
        vlm_lyt.setContentsMargins(0, 0, 0, 0)
        vlm_lyt.setSpacing(6)

        vlm_label_head = QLabel("SAHNE ANALİZİ (CANLI)")
        vlm_label_head.setStyleSheet("color: #f59e0b; font-size: 10px; font-weight: bold; font-family: 'Roboto Mono';")
        vlm_lyt.addWidget(vlm_label_head)
        
        self.vlm_text_box = QTextBrowser()
        self.vlm_text_box.setStyleSheet("font-family: 'Roboto Mono'; font-size: 11px; color:#e2e8f0; background-color: rgba(245,158,11,0.04); border:1px solid rgba(245,158,11,0.2); padding:10px; border-radius:4px; border-left:3px solid #f59e0b;")
        vlm_lyt.addWidget(self.vlm_text_box)
        
        tv_content_lyt.addWidget(vlm_container, stretch=1)

        # Sağ Taraf: MCP Workflow Tracker
        self.mcp_tracker = McpWorkflowTrackerWidget()
        tv_content_lyt.addWidget(self.mcp_tracker, stretch=1)

        self.timeline_widget = None
        
        timeline_vlm_layout.addWidget(tv_content)
        middle_row_splitter.addWidget(timeline_vlm_panel)
        
        workspace_splitter.addWidget(middle_row_splitter)
        
        # COLLAPSIBLE LOGS PANEL (Bottom of GCS workspace)
        self.logs_panel = QFrame()
        self.logs_panel.setProperty("class", "panel")
        self.logs_panel.setFixedHeight(120)
        self.logs_panel.setVisible(False) # Collapsed by default
        
        logs_lyt = QVBoxLayout(self.logs_panel)
        logs_lyt.setContentsMargins(0, 0, 0, 0)
        logs_lyt.setSpacing(0)
        
        logs_title_bar = QFrame()
        logs_title_bar.setProperty("class", "panel-header")
        logs_title_bar.setFixedHeight(24)
        logs_title_lyt = QHBoxLayout(logs_title_bar)
        logs_title_lyt.setContentsMargins(10, 0, 10, 0)
        
        logs_lbl = QLabel("MAVLINK TELEMETRİ & SİSTEM LOG PARSER")
        logs_lbl.setProperty("class", "title-label")
        logs_lbl.setStyleSheet("font-size: 9px;")
        logs_title_lyt.addWidget(logs_lbl)
        
        logs_title_lyt.addStretch()
        
        close_logs_btn = QPushButton("×")
        close_logs_btn.setStyleSheet("color:#94a3b8; font-size:14px; background:none; border:none; padding:0 5px;")
        close_logs_btn.clicked.connect(self.toggle_bottom_logs)
        logs_title_lyt.addWidget(close_logs_btn)
        
        logs_lyt.addWidget(logs_title_bar)
        
        self.logs_terminal = QPlainTextEdit()
        self.logs_terminal.setProperty("class", "console")
        self.logs_terminal.setReadOnly(True)
        logs_lyt.addWidget(self.logs_terminal)
        
        workspace_splitter.addWidget(self.logs_panel)
        
        main_splitter.addWidget(workspace_splitter)
        
        # 3. AI COPILOT CHAT SIDEBAR (Right Panel)
        sidebar_panel = QFrame()
        sidebar_panel.setProperty("class", "panel")
        sidebar_panel.setFixedWidth(340)
        sidebar_lyt = QVBoxLayout(sidebar_panel)
        sidebar_lyt.setContentsMargins(0, 0, 0, 0)
        sidebar_lyt.setSpacing(0)
        
        sb_title_bar = QFrame()
        sb_title_bar.setProperty("class", "panel-header")
        sb_title_bar.setFixedHeight(34)
        sb_title_layout = QHBoxLayout(sb_title_bar)
        sb_title_layout.setContentsMargins(12, 0, 12, 0)
        
        sb_title_lbl = QLabel("AI COPILOT ORCHESTRATOR")
        sb_title_lbl.setProperty("class", "title-label")
        sb_title_layout.addWidget(sb_title_lbl)
        sb_title_layout.addStretch()
        
        sidebar_lyt.addWidget(sb_title_bar)
        
        # Chat history output
        self.chat_history = QTextBrowser()
        self.chat_history.setProperty("class", "chat-history")
        self.chat_history.setOpenExternalLinks(True)
        self.chat_history.setHtml(
            "<div style='margin-bottom: 8px;'><b style='color:#10b981;'>SYSTEM COPILOT</b><br>"
            "Merhaba Operatör. Ben Gemini destekli canlı AI Copilot. Telemetri, batarya, YOLO tespitleri ve VLM sahne verilerine "
            "MCP (Model Context Protocol) araç sunucusu üzerinden erişiyorum; hem anlık değerleri hem uçuş başından beri biriken geçmişi okuyabiliyorum.<br><br>"
            "Aşağıdaki araç butonlarını kullanabilir (<b>Rapor Oluştur</b>, <b>Genel Özet</b>, <b>Telemetri</b>, <b>YoloE</b>) "
            "veya serbest soru sorabilirsin. Uçağı da yönlendirebilirim: "
            "<i>\"sağa dön\"</i>, <i>\"100 metre ileri git\"</i>, <i>\"irtifayı 30 metreye çıkar\"</i> gibi "
            "komutlar uçağa gerçek zamanlı iletilir (otonom rota izleme o anda devre dışı kalır).</div>"
        )
        
        chat_padding_widget = QWidget()
        chat_padding_lyt = QVBoxLayout(chat_padding_widget)
        chat_padding_lyt.setContentsMargins(10, 8, 10, 8)
        chat_padding_lyt.setSpacing(8)
        chat_padding_lyt.addWidget(self.chat_history)
        
        # MCP araç kısayol butonları: her buton, MCP sunucusunda (mcp_server.py)
        # yaşayan araçlardan ilgili kümeye Gemini'yi zorlar (forced_tools),
        # böylece buton kapsamı dışına çıkamaz. Araç tanımları burada değil
        # sunucuda durur; sorgu anında tools/list ile keşfedilir.
        pills_layout = QGridLayout()
        pills_layout.setSpacing(4)

        for idx, (lbl, desc, query, tools) in enumerate(self.COPILOT_TOOL_BUTTONS):
            btn = QPushButton(lbl)
            btn.setProperty("class", "suggestion-pill")
            btn.setToolTip(desc)
            btn.clicked.connect(
                lambda checked=False, q=query, t=tools: self.handle_chat_query(q, forced_tools=t)
            )
            pills_layout.addWidget(btn, idx // 2, idx % 2)

        chat_padding_lyt.addLayout(pills_layout)
        
        # Message inputs
        input_layout = QHBoxLayout()
        input_layout.setSpacing(5)
        
        self.chat_input = QLineEdit()
        self.chat_input.setProperty("class", "chat-input")
        self.chat_input.setPlaceholderText("Girdi bekleniyor...")
        self.chat_input.returnPressed.connect(self.submit_chat_message)
        input_layout.addWidget(self.chat_input)
        
        chat_send_btn = QPushButton("Gönder")
        chat_send_btn.setProperty("class", "btn-primary")
        chat_send_btn.clicked.connect(self.submit_chat_message)
        input_layout.addWidget(chat_send_btn)
        
        chat_padding_lyt.addLayout(input_layout)
        sidebar_lyt.addWidget(chat_padding_widget)
        
        main_splitter.addWidget(sidebar_panel)
        main_layout.addWidget(main_splitter)

    # ------------------------------------------------------------------
    # MAP & TELEMETRY STREAM SIMULATION LOOPS
    # ------------------------------------------------------------------
    def on_map_load_finished(self, ok):
        if ok:
            self.map_loaded = True
            self.add_terminal_log("Harita başarıyla yüklendi.", "INFO")
            self.setup_map_initial_waypoints()
        else:
            self.add_terminal_log("Harita yüklenemedi!", "WARN")

    def setup_map_initial_waypoints(self):
        self.map_view.set_waypoints(self.waypoints)

    # Batarya doluluk kademeleri: (alt sınır %, renk, yazı eki). Yukarıdan aşağı
    # ilk eşleşen kademe uygulanır. Kamera HUD'ı ve telemetri kartı aynı eşikleri
    # kullanır ki operatör iki yerde farklı renk görmesin.
    BATTERY_TIERS = (
        (80, "#10b981", ""),            # Yeşil  — güvenli
        (60, "#facc15", ""),            # Sarı   — izlenmeli
        (20, "#f97316", " [DÜŞÜK]"),    # Turuncu— düşük
        (0,  "#ef4444", " [ALARM]"),    # Kırmızı— kritik
    )

    @classmethod
    def battery_tier(cls, percent):
        """Doluluk yüzdesine karşılık gelen (renk, yazı eki) çiftini döner."""
        for threshold, color, suffix in cls.BATTERY_TIERS:
            if percent >= threshold:
                return color, suffix
        return cls.BATTERY_TIERS[-1][1], cls.BATTERY_TIERS[-1][2]

    # Kat edilen mesafe (kilometre sayacı) eşikleri:
    #  - alt sınır: duran uçakta GPS gürültüsü (birkaç cm/örnek) yol gibi birikip
    #    sayaç kendi kendine artmasın;
    #  - üst sınır: EKF/GPS ilk kilitlenmesindeki sıçrama tek adımda yüzlerce
    #    metre eklemesin. yolo.py'deki travel_m ile aynı mantık.
    TRAVEL_MIN_STEP_M = 0.5
    TRAVEL_MAX_STEP_M = 100.0

    def update_travelled_distance(self):
        """KAT EDİLEN toplam yolu (m, tamsayı) döner ve biriktirir.

        Eskiden başlangıç noktasına olan DÜZ ÇİZGİ uzaklığı gösteriliyordu: uçak
        geri döndüğünde 'kat edilen mesafe' azalıyordu. Artık ardışık GPS
        örnekleri arasındaki adımlar toplanır — kilometre sayacı gibi yalnızca
        artar, rota geri dönse de doğru kalır."""
        lat = self.telemetry.get("lat")
        lon = self.telemetry.get("lon")
        if lat is None or lon is None:
            return int(self.travel_m)
        prev = self._last_travel_fix
        self._last_travel_fix = (lat, lon)
        if prev is None:
            return int(self.travel_m)
        d_lat = (lat - prev[0]) * 111000.0
        d_lon = (lon - prev[1]) * 111000.0 * math.cos(math.radians(lat))
        step = math.sqrt(d_lat * d_lat + d_lon * d_lon)
        if self.TRAVEL_MIN_STEP_M <= step <= self.TRAVEL_MAX_STEP_M:
            self.travel_m += step
        return int(self.travel_m)

    def update_telemetry_loop(self):
        # 1. Update clock
        self.clock_label.setText(datetime.now().strftime("%H:%M:%S"))

        dist_m = self.update_travelled_distance()

        # 1b. AI Copilot geçmiş telemetri tamponu (saniyede bir örnek).
        self.telemetry_history.append({
            "t": time.time() - self.mission_start_ts,
            "time": datetime.now().strftime("%H:%M:%S"),
            "alt": self.telemetry["alt"],
            "speed": self.telemetry["speed"],
            "battery": self.telemetry["battery"],
            "voltage": self.telemetry["voltage"],
            "mode": self.telemetry["mode"],
            "lat": self.telemetry["lat"],
            "lon": self.telemetry["lon"],
            "heading": self.telemetry["heading"],
            "pitch": self.telemetry.get("pitch", 0.0),
            "roll": self.telemetry.get("roll", 0.0),
            "sats": self.telemetry["sats"],
            "dist": dist_m,
        })

        # 5. Update Indicators UI
        self.lbl_alt.setText(f"İRTİFA (ALT): <span style='color:#06b6d4; font-weight:bold;'>{self.telemetry['alt']:.1f} m</span>")
        self.lbl_speed.setText(f"HIZ (VELOCITY): <span style='color:#06b6d4; font-weight:bold;'>{self.telemetry['speed']:.1f} m/s</span>")
        
        rounded_bat = int(self.telemetry["battery"])
        self.bar_battery.setValue(rounded_bat)

        # Batarya kademesi: doluluk düştükçe hem yazı, hem çubuk, hem kart kenar
        # çizgisi aynı renge geçer. Eskiden %50'nin üstündeki HER değer yeşil
        # yandığı için %60'ta hâlâ "tam dolu" izlenimi veriyordu; artık %70'in
        # altında sarıya, %40'ın altında turuncuya, %20'nin altında kırmızıya döner.
        bat_color, bat_suffix = self.battery_tier(rounded_bat)
        self.lbl_battery.setText(
            f"BATARYA (BAT): <span style='color:{bat_color}; font-weight:bold;'>"
            f"{rounded_bat}%{bat_suffix}</span> ({self.telemetry['voltage']} V)"
        )
        self.bar_battery.setStyleSheet(
            "QProgressBar { background-color: #030712; border-radius: 2px; } "
            f"QProgressBar::chunk {{ background-color: {bat_color}; }}"
        )
        if hasattr(self, "bat_card"):
            self.bat_card.setStyleSheet(
                f"background-color:#0d1522; border-radius:4px; border-left: 3px solid {bat_color};"
            )


        mode_color = "#ef4444" if self.telemetry["mode"] == "DISCONNECTED" else "#f59e0b"
        self.lbl_mode.setText(f"UÇUŞ MODU: <span style='color:{mode_color}; font-weight:bold;'>{self.telemetry['mode']}</span>")

        self.lbl_heading.setText(f"YÖNELİM: <span style='color:#06b6d4; font-weight:bold;'>{self.telemetry['heading']}° (Yaw)</span>")
        self.compass_widget.set_heading(self.telemetry["heading"])
        
        # Uçuş Süresi (Uçak harekete geçtiğinde / kalkış yaptığında başlar)
        cur_alt = float(self.telemetry.get("alt", 0.0))
        cur_speed = float(self.telemetry.get("speed", 0.0))
        cur_mode = str(self.telemetry.get("mode", "")).upper()

        if not self.flight_started and (cur_alt > 1.0 or cur_speed > 1.5 or cur_mode in ("TAKEOFF", "AUTO", "GUIDED", "FBWA")):
            self.flight_started = True
            self.takeoff_start_ts = time.time()
            self.add_terminal_log("Uçak harekete geçti / kalkış başladı! Uçuş sayacı başlatıldı.", "INFO")

        if self.flight_started and self.takeoff_start_ts is not None:
            elapsed_sec = int(time.time() - self.takeoff_start_ts)
        else:
            elapsed_sec = 0

        mins, secs = divmod(elapsed_sec, 60)

        self.lbl_flight_dist.setText(
            f"UÇUŞ SÜRESİ: <span style='color:#06b6d4; font-weight:bold;'>{mins:02d}:{secs:02d}</span> | "
            f"MESAFE: <span style='color:#f59e0b; font-weight:bold;'>{dist_m} m</span>"
        )

        self.lbl_gps.setText(f"GPS KOORDİNAT:<br><span style='color:#f8fafc;'>LAT: {self.telemetry['lat']:.6f}<br>LON: {self.telemetry['lon']:.6f}</span>")

        # Telemetri Zaman Serisi Grafiğini Güncelle
        if hasattr(self, "telemetry_chart"):
            self.telemetry_chart.set_history(self.telemetry_history)
        
        # Update connection badge based on telemetry status
        if self.telemetry["mode"] == "DISCONNECTED":
            self.conn_badge.setText("BAĞLANTI YOK")
            self.conn_badge.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 10px; font-family: 'Roboto Mono'; background-color: rgba(239, 68, 68, 0.1); border: 1px solid rgba(239, 68, 68, 0.3); padding: 2px 6px; border-radius:3px;")
        else:
            self.conn_badge.setText("● CANLI BAĞLANTI")
            self.conn_badge.setStyleSheet("color: #10b981; font-weight: bold; font-size: 10px; font-family: 'Roboto Mono'; background-color: rgba(16, 185, 129, 0.1); border: 1px solid rgba(16, 185, 129, 0.3); padding: 2px 6px; border-radius:3px;")

        # Feed inputs to Camera HUD simulator widget
        self.camera_sim.set_telemetry(self.telemetry["lat"], self.telemetry["lon"], self.telemetry["heading"], self.telemetry["speed"])
        
        # Haritadaki İHA konum/iz işaretini güncelle
        if self.map_loaded:
            self.map_view.update_uav(
                self.telemetry["lat"], self.telemetry["lon"], self.telemetry["heading"],
                self.telemetry["mode"], self.telemetry["alt"], self.telemetry["speed"],
            )

        # drone.mp4 akışı gerçek bir uçuşun kaydı; ekrandaki SITL telemetrisiyle
        # ilgisi yok. Yan yana gösterilince "bu videonun verisi" sanıldığı için
        # video modunda panel ve grafikler boşaltılır. Yukarıdaki hesaplar
        # (geçmiş tamponu, uçuş sayacı, harita, Copilot) aynen sürer; Gazebo'ya
        # dönüldüğünde panel bir sonraki tik'te kendi verisiyle geri dolar.
        if getattr(self, "active_camera_mode", "gazebo") == "video":
            self._blank_telemetry_ui()

    # Veri yokken telemetri kartlarında gösterilen yer tutucu.
    TELEMETRY_PLACEHOLDER = "<span style='color:#64748b; font-weight:bold;'>---</span>"

    def _blank_telemetry_ui(self):
        """Telemetri sekmesindeki tüm sayısal alanları ve 2x2 grafik panelini
        boşaltır (video akışı aktifken kullanılır)."""
        ph = self.TELEMETRY_PLACEHOLDER
        self.lbl_alt.setText(f"İRTİFA (ALT): {ph}")
        self.lbl_speed.setText(f"HIZ (VELOCITY): {ph}")
        self.lbl_battery.setText(f"BATARYA (BAT): {ph}")
        self.bar_battery.setValue(0)
        self.bar_battery.setStyleSheet(
            "QProgressBar { background-color: #030712; border-radius: 2px; } "
            "QProgressBar::chunk { background-color: #334155; }"
        )
        if hasattr(self, "bat_card"):
            self.bat_card.setStyleSheet(
                "background-color:#0d1522; border-radius:4px; border-left: 3px solid #334155;"
            )
        self.lbl_mode.setText(f"UÇUŞ MODU: {ph}")
        self.lbl_heading.setText(f"YÖNELİM: {ph}")
        self.compass_widget.set_heading(0)
        self.lbl_gps.setText(f"GPS KOORDİNAT:<br>{ph}")
        self.lbl_flight_dist.setText(f"UÇUŞ SÜRESİ: {ph} | MESAFE: {ph}")
        if hasattr(self, "telemetry_chart"):
            # Boş geçmiş → grafikler "Veri bekleniyor..." durumuna döner.
            self.telemetry_chart.set_history([])

    # ------------------------------------------------------------------
    # YOLO TESPİTLER & HARİTA ETKİLEŞİMİ
    # ------------------------------------------------------------------
    def update_detections_table(self):
        target_list = self.detection_log if hasattr(self, "detection_log") and self.detection_log else self.live_detections
        self.detections_table.setRowCount(len(target_list))
        for row, det in enumerate(target_list):
            id_item = QTableWidgetItem(det["id"])
            type_item = QTableWidgetItem(det["type"])
            conf_item = QTableWidgetItem(f"%{int(det['conf']*100)}")
            time_item = QTableWidgetItem(det.get("time", det.get("first_time", "-")))
            lat, lon = det.get("lat"), det.get("lon")
            if lat is not None and lon is not None:
                gps_item = QTableWidgetItem(f"{lat:.5f}, {lon:.5f}")
            else:
                gps_item = QTableWidgetItem("Video Akışı")
            # İrtifa bilgisini tespit verisiyle birlikte tut (tooltip olarak göster).
            if det.get("alt") is not None:
                gps_item.setToolTip(f"İrtifa: {det['alt']:.0f} m")
            # Kaç karede doğrulandığını (hits) ve normalize kategoriyi tip tooltip'inde göster.
            tip_parts = []
            if det.get("category"):
                tip_parts.append(f"Kategori: {det['category']}")
            if det.get("hits"):
                tip_parts.append(f"{det['hits']} karede görüldü")
            if det.get("qr_text"):
                # QR metni çözülen kutu tabloda görünür işaret alır; metnin
                # tamamı tooltip'te.
                type_item.setText(f"{det['type']} [QR]")
                tip_parts.append(f"QR: {det['qr_text']}")
            if tip_parts:
                type_item.setToolTip(" • ".join(tip_parts))

            # Make items read-only
            for item in (id_item, type_item, conf_item, time_item, gps_item):
                item.setFlags(item.flags() ^ Qt.ItemIsEditable)
                
            self.detections_table.setItem(row, 0, id_item)
            self.detections_table.setItem(row, 1, type_item)
            self.detections_table.setItem(row, 2, conf_item)
            self.detections_table.setItem(row, 3, time_item)
            self.detections_table.setItem(row, 4, gps_item)

    # Aynı cismi tekrar tekrar (farklı track_id veya farklı sınıf etiketiyle)
    # bastırmamak için: yeni tespit, son DET_MERGE_WINDOW saniye içinde görülmüş
    # ve bbox'ı DET_MERGE_IOU oranında örtüşen bir kayıtla eşleşirse aynı cisim
    # sayılır; yalnızca tepe güven ve o güvenin etiketi tabloda tutulur.
    DET_MERGE_WINDOW = 5.0   # saniye
    DET_MERGE_IOU = 0.3

    @staticmethod
    def _bbox_iou(a, b):
        """İki [x1,y1,x2,y2] kutusunun IoU (kesişim/birleşim) oranı."""
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
    def _cls_to_tr(cls):
        # YOLO ham sınıf adını veya normalize kategoriyi Türkçe nesne tipine çevirir.
        mapping = {
            # ham etiketler
            "box": "Kutu", "crate": "Kutu", "container": "Kutu",
            "cargo box": "Kutu", "blue cargo box": "Kutu", "package": "Kutu",
            "car": "Araç", "truck": "Araç", "vehicle": "Araç",
            "pickup truck": "Araç", "van": "Araç",
            "automobile": "Araç", "sedan": "Araç", "suv": "Araç",
            "hatchback": "Araç", "semi-truck": "Araç", "tractor": "Araç",
            "human": "Kişi", "person": "Kişi", "pedestrian": "Kişi",
            # NOT: 'drone/uav' eşlemesi KASITLI kaldırıldı — rotadaki drone cismi
            # otomobille değiştirildi, prompt'tan da çıkarıldı (bkz. yolo.py).
            "backpack": "Sırt Çantası", "rucksack": "Sırt Çantası",
            "knapsack": "Sırt Çantası",
            "stop sign": "Stop Tabelası",
            "checkerboard": "Dama Paneli", "chessboard": "Dama Paneli",
            "concentric circles": "Nişan Tahtası", "target": "Nişan Tahtası",
            "bullseye": "Nişan Tahtası",
            # normalize kategoriler
            "cargo_box": "Kutu",
            "stop_sign": "Stop Tabelası",
            "checker_panel": "Dama Paneli",
            # İnsan aktörlerinin yerine konan renkli kutular: yolo.py bbox'ın
            # baskın HSV rengine göre kategoriyi red_box/blue_box'a çevirir.
            "red_box": "Kırmızı Kutu",
            "blue_box": "Mavi Kutu",
        }
        return mapping.get(str(cls).lower(), str(cls))

    def handle_detection(self, data):
        """yolo.py'den gelen tespit paketini (JSON) tabloya + zaman çizelgesine
        canlı olarak düşürür. track_id varsa aynı hedef için yeni satır açmak
        yerine mevcut satırı günceller; track_id yoksa sınıf bazlı tekrar
        bastırma (3 sn) devreye girer."""
        dets = data.get("detections", []) if isinstance(data, dict) else []
        if not dets:
            return
        now = time.time()
        now_str = datetime.now().strftime("%H:%M:%S")
        uav_lat = self.telemetry["lat"]
        uav_lon = self.telemetry["lon"]
        alt = self.telemetry.get("alt")
        changed = False
        for d in dets:
            # yolo.py artık her hedefin KENDİ yer koordinatını gönderiyor
            # (bbox merkezinin geo-projeksiyonu). Yoksa (irtifa/GPS yok, eski
            # sürüm) eskisi gibi İHA konumuna düşülür — haritada hepsi uçuş
            # hattının üstünde birikir ama hiçbir şey kırılmaz.
            lat = d.get("lat") if d.get("lat") is not None else uav_lat
            lon = d.get("lon") if d.get("lon") is not None else uav_lon
            cls = d.get("cls", "?")
            conf = float(d.get("conf", 0.0))
            track_id = d.get("track_id")
            bbox = d.get("bbox")
            hits = int(d.get("hits", 1))
            # Normalize kategori (box↔crate aynı hedef) varsa onu kullan; böylece
            # aynı cisim farklı ham etiketle gelse de aynı Türkçe tip gösterilir.
            category = d.get("category") or cls
            type_tr = self._cls_to_tr(category)

            # --- Aynı cismi mi görüyoruz? ---
            # 1) Aynı track_id daha önce görüldüyse doğrudan o satır.
            # 2) track_id churn ettiyse veya cisim farklı bir sınıf etiketiyle
            #    (ör. box↔truck) geldiyse: son DET_MERGE_WINDOW sn içinde görülmüş
            #    ve bbox'ı örtüşen kayıt aynı cisimdir.
            matched = None
            if track_id is not None and track_id in self.gazebo_track_rows:
                matched = self.gazebo_track_rows[track_id]
            if matched is None and bbox is not None:
                for e in self.gazebo_live_detections:
                    if now - e.get("_seen", 0.0) > self.DET_MERGE_WINDOW:
                        continue
                    if self._bbox_iou(bbox, e.get("bbox")) >= self.DET_MERGE_IOU:
                        matched = e
                        break

            if matched is not None:
                # Aynı cisim: satırı canlı tut, ama SADECE daha yüksek güven gelirse
                # oranı ve etiketi güncelle (tepe değeri ve o değerin sınıfını yaz).
                matched["_seen"] = now
                matched["time"] = now_str
                matched["lat"], matched["lon"], matched["alt"] = lat, lon, alt
                matched["hits"] = max(matched.get("hits", 1), hits)
                matched["category"] = category
                if bbox is not None:
                    matched["bbox"] = bbox
                if track_id is not None:
                    self.gazebo_track_rows[track_id] = matched
                if conf > matched["conf"]:
                    matched["conf"] = conf
                    matched["type"] = type_tr
                self._register_qr_text(matched, d.get("qr_text"))
                changed = True
                continue

            # Yeni hedef.
            self.gazebo_det_counter += 1
            target_id = f"Y-{self.gazebo_det_counter:02d}"
            entry = {
                "id": target_id,
                "type": type_tr,
                "category": category,
                "conf": conf,
                "time": now_str,
                "first_time": now_str,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "bbox": bbox,
                "hits": hits,
                "_seen": now,
            }
            self._register_qr_text(entry, d.get("qr_text"), announce=False)
            self.gazebo_live_detections.append(entry)
            # Aynı nesne referansı geçmiş kaydına da girer: tablodan düşse bile
            # AI Copilot uçuş boyunca görülen hedefi raporlayabilir.
            self.gazebo_detection_log.append(entry)
            if len(self.gazebo_detection_log) > 200:
                self.gazebo_detection_log.pop(0)
            if track_id is not None:
                self.gazebo_track_rows[track_id] = entry
            # Tabloyu makul uzunlukta tut; düşen kaydın track eşlemesini de temizle.
            if len(self.gazebo_live_detections) > 25:
                removed = self.gazebo_live_detections.pop(0)
                for tid, e in list(self.gazebo_track_rows.items()):
                    if e is removed:
                        del self.gazebo_track_rows[tid]
            changed = True
            # Timeline ve harita yalnızca hedef ilk kez görüldüğünde işaretlenir.
            self.add_timeline_event("detection", f"YOLO tespiti: {type_tr} (%{int(conf*100)})")
            self.add_terminal_log(f"YOLO tespiti [{target_id}]: {type_tr} — güven %{int(conf*100)}", "INFO")
            if getattr(self, "map_loaded", False):
                self.map_view.add_target(target_id, type_tr, lat, lon, conf)
            if entry.get("qr_text"):
                self._announce_qr_text(entry)
        if changed and self.active_camera_mode == "gazebo":
            self.live_detections = self.gazebo_live_detections
            self.detection_log = self.gazebo_detection_log
            self._track_rows = self.gazebo_track_rows
            self.update_detections_table()
            QTimer.singleShot(100, self.update_vlm_from_log)

    def _register_qr_text(self, entry, qr_text, announce=True):
        """yolo.py'nin kutudan çözdüğü QR metnini hedef kaydına işler. Hedef
        başına bir kez duyurulur (terminal log + zaman çizelgesi + harita).
        announce=False: kayıt henüz duyurulmaya hazır değil (yeni hedefin ID
        logu önce gelsin), duyuruyu çağıran yapar."""
        if not qr_text or entry.get("qr_text"):
            return
        entry["qr_text"] = qr_text
        if announce:
            self._announce_qr_text(entry)

    def _announce_qr_text(self, entry):
        qr_text = entry.get("qr_text", "")
        self.add_terminal_log(
            f"QR metni okundu [{entry['id']}]: \"{qr_text}\"", "INFO"
        )
        self.add_timeline_event("detection", f"QR okundu: {qr_text}")
        if getattr(self, "map_loaded", False):
            self.map_view.set_target_qr_text(entry["id"], qr_text)

    def handle_table_row_click(self, item):
        row = item.row()
        target_list = self.detection_log if hasattr(self, "detection_log") and self.detection_log else self.live_detections
        if row >= len(target_list):
            return
        det = target_list[row]
        lat, lon = det.get("lat"), det.get("lon")
        if lat is not None and lon is not None:
            if self.map_loaded:
                self.map_view.center_on(lat, lon)
            self.add_terminal_log(f"Harita {det['id']} koordinatına odaklandı ({lat:.5f}, {lon:.5f}).", "NAV")

    def center_map_on_uav(self):
        if self.map_loaded:
            self.map_view.center_on_uav()
        self.add_terminal_log("Harita İHA anlık konumuna merkezlendi.", "NAV")

    # ------------------------------------------------------------------
    # TIMELINE & VLM SAHNE ANALİZLERİ
    # ------------------------------------------------------------------
    def add_timeline_event(self, ev_type, desc):
        now_str = datetime.now().strftime("%H:%M:%S")
        self.timeline_events.append({"time": now_str, "type": ev_type, "desc": desc})
        
        # Keep latest 6 events
        if len(self.timeline_events) > 6:
            self.timeline_events.pop(0)
            
        if self.timeline_widget:
            self.timeline_widget.set_events(self.timeline_events)

    def update_vlm_from_log(self):
        if self.active_camera_mode == "video":
            if getattr(self, "video_detection_log", None):
                messages = []
                for d in self.video_detection_log[-15:]:
                    t_str = d.get('time', d.get('first_time', ''))
                    msg = f"Video tespiti [{d['id']}]: {d['type']} (%{int(d['conf']*100)}) - Saat: {t_str}"
                    if msg not in messages:
                        messages.append(msg)
                if messages:
                    html_content = ""
                    for msg in messages:
                        html_content += f"<div style='margin-bottom: 6px; line-height: 1.3;'>• {msg}</div>"
                    self.vlm_text_box.setHtml(html_content)
                    self.vlm_text_box.verticalScrollBar().setValue(self.vlm_text_box.verticalScrollBar().maximum())
                    if self.scene_db is not None:
                        self.scene_db.record(
                            "\n".join(messages), source="video_detection",
                            telemetry=self.telemetry,
                            mission_time_s=time.time() - self.mission_start_ts,
                            message_count=len(messages),
                        )
                    return True
            return False

        yolo_log_path = os.path.join(BUILD_DIR, "yolo.log")
        if os.path.exists(yolo_log_path):
            try:
                messages = []
                with open(yolo_log_path, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith("{") and line.endswith("}"):
                            try:
                                data = json.loads(line)
                                if data.get("event_type") == "object_detection" and "message" in data:
                                    msg = data["message"]
                                    if msg not in messages:
                                        messages.append(msg)
                            except Exception:
                                continue
                if messages:
                    html_content = ""
                    for msg in messages:
                        html_content += f"<div style='margin-bottom: 6px; line-height: 1.3;'>• {msg}</div>"
                    self.vlm_text_box.setHtml(html_content)
                    # Scroll to bottom
                    self.vlm_text_box.verticalScrollBar().setValue(self.vlm_text_box.verticalScrollBar().maximum())
                    # Gösterilen özeti veritabanına işle (yalnızca içerik
                    # değiştiyse satır yazılır; SceneDatabase dedup yapar).
                    if self.scene_db is not None:
                        self.scene_db.record(
                            "\n".join(messages), source="detection",
                            telemetry=self.telemetry,
                            mission_time_s=time.time() - self.mission_start_ts,
                            message_count=len(messages),
                        )
                    return True
            except Exception as e:
                logger.warning(f"yolo.log okunurken hata oluştu: {e}")
        return False

    def update_vlm_summary(self):
        if self.update_vlm_from_log():
            return
        summary = self.vlm_summaries[self.vlm_index]
        self.vlm_text_box.setHtml(f"<div style='margin-bottom: 6px; line-height: 1.3;'>• {summary}</div>")
        self.vlm_index = (self.vlm_index + 1) % len(self.vlm_summaries)
        # Yer tutucu özet de arayüzde "gösterilen" içeriktir; kaydet (dedup
        # sayesinde aynı metin dönerken tek satır kalır).
        if self.scene_db is not None:
            self.scene_db.record(
                summary, source="placeholder", telemetry=self.telemetry,
                mission_time_s=time.time() - self.mission_start_ts,
            )

    # ------------------------------------------------------------------
    # BOTTOM COLLAPSIBLE LOGS TERMINAL
    # ------------------------------------------------------------------
    def toggle_bottom_logs(self):
        is_visible = self.logs_panel.isVisible()
        self.logs_panel.setVisible(not is_visible)
        self.toggle_logs_btn.setText("Logları Kapat" if not is_visible else "Logları Aç")
        if not is_visible:
            # Scroll to bottom
            self.logs_terminal.verticalScrollBar().setValue(self.logs_terminal.verticalScrollBar().maximum())

    def add_terminal_log(self, text, log_type="INFO"):
        if log_type == "WARN":
            logger.warning(text)
        elif log_type == "DEBUG":
            logger.debug(text)
        elif log_type == "ERROR" or log_type == "CRITICAL":
            logger.error(text)
        else:
            logger.info(text)

        time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Color formatting
        color = "#10b981" # Green
        if log_type == "WARN":
            color = "#f59e0b" # Yellow
        elif log_type == "NAV":
            color = "#06b6d4" # Cyan
        elif log_type == "DEBUG":
            color = "#64748b" # Gray
            
        log_line = f"<span style='color:#475569;'>[{time_str}]</span> <span style='color:{color}; font-weight:bold;'>[{log_type}]</span> {text}"
        self.logs_terminal.appendHtml(log_line)
        
        # Limit rows
        if self.logs_terminal.document().blockCount() > 120:
            # Remove first block
            cursor = self.logs_terminal.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.select(QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar() # delete remaining carriage return
            
        # Auto-scroll if visible
        if self.logs_panel.isVisible():
            self.logs_terminal.verticalScrollBar().setValue(self.logs_terminal.verticalScrollBar().maximum())



    # ------------------------------------------------------------------
    # AI COPILOT CHAT ENGINE
    # ------------------------------------------------------------------
    # Sohbet kutusunun üstündeki 4 araç kısayolu — MCP mimarisi:
    #   (buton etiketi, tooltip, Gemini'ye gidecek sorgu, izin verilen MCP araçları).
    # Araç adları mcp_server.py'deki tools/list kayıtlarıyla birebir eşleşir;
    # tanım/şema/yürütme sunucudadır, buton yalnızca ad listesiyle kapsam kilitler.
    # 'forced_tools' listesi Gemini'nin function-calling'ini o araçlara kilitler;
    # böylece "Telemetri" butonu YOLO verisine, "YoloE" butonu telemetriye kaymaz.
    COPILOT_TOOL_BUTTONS = [
        ("Rapor Oluştur", "Telemetri, tespit ve VLM verilerini birleştiren taktik raporu derle ve indir.",
         "Taktik raporu derle ve indirme penceresini aç; ardından raporun içeriğini kısaca özetle.",
         ["generate_tactical_report"]),
        ("Genel Özet", "Telemetri + YOLO'nun geçmiş ve anlık verilerini birleştiren genel durum özeti.",
         "Telemetri ve YOLO tespitlerinin hem geçmiş hem anlık verilerini birleştirip genel "
         "taktiksel durum özeti çıkar.",
         ["get_situation_summary"]),
        ("Telemetri", "Sadece telemetrinin anlık değerleri ve uçuş geçmişindeki trendi.",
         "Sadece telemetriyi raporla: anlık değerler ve uçuş başından bu yana geçmiş trend "
         "(irtifa, hız, batarya, mod değişimleri, kat edilen mesafe).",
         ["get_telemetry", "get_telemetry_history", "get_battery_status"]),
        ("YoloE", "Sadece YOLO'nun anlık aktif ve geçmiş tespitleri.",
         "Sadece YOLO tespitlerini raporla: şu an aktif hedefler ve uçuş başından bu yana "
         "tespit edilen tüm hedeflerin geçmişi (tür, güven, zaman, koordinat).",
         ["get_detections", "get_detection_history"]),
    ]

    def submit_chat_message(self):
        query = self.chat_input.text().strip()
        if not query:
            return
        self.chat_input.clear()
        self.handle_chat_query(query)

    def handle_chat_query(self, query, forced_tools=None):
        """Operatör sorgusunu Gemini'ye iletir. forced_tools verilirse (araç
        kısayol butonları) model ilk turda yalnızca o araçları çağırabilir;
        serbest yazılan sorularda (None) aracı model kendi seçer."""
        # 1. Operatör mesajını sohbet kutusuna yaz.
        self.chat_history.append(
            f"<div style='margin-bottom: 8px;'><b style='color:#06b6d4;'>OPERATOR</b><br>{query}</div>"
        )
        self.add_terminal_log(f"Operatör AI Copilot sorgusu: '{query}'", "INFO")

        # Uçuşa başlama anahtar kelimeleri kontrolü: eğer uçak henüz kalkmadıysa ve
        # operatör "uçak uçmaya hazır", "kalkışa geç" vb. dediyse otomatik kalkışı başlat.
        FLIGHT_START_KEYWORDS = [
            "uçmaya hazır", "uçuşa hazır", "kalkışa geç", "kalkış yap",
            "uçuşa geç", "uçuşu başlat", "uçağı kaldır", "kalkabilirsin",
            "havalan", "uçuşa başla", "takeoff", "ready to fly", "start flight"
        ]
        query_lower = query.lower()
        if any(kw in query_lower for kw in FLIGHT_START_KEYWORDS):
            if not hasattr(self, "otonom_flight_process") or self.otonom_flight_process is None or self.otonom_flight_process.poll() is not None:
                self.start_otonom_flight()

        # Aynı anda tek sorgu: önceki thread hâlâ çalışıyorsa yeni sorguyu reddet.
        if self.chat_thread is not None and self.chat_thread.isRunning():
            self.chat_history.append(
                "<div style='margin-bottom: 8px;'><b style='color:#f59e0b;'>SYSTEM COPILOT</b>"
                "<br>Önceki sorgu hâlâ işleniyor, lütfen bekleyin.</div>"
            )
            self._scroll_chat_bottom()
            return

        if not self.gemini_api_key:
            self._append_copilot_html(
                "<span style='color:#f59e0b;'>Gemini API anahtarı ayarlı değil.</span><br>"
                "config/.env dosyasına <b>GEMINI_API_KEY</b> ekleyip uygulamayı yeniden başlatın."
            )
            return

        # 2. MCP araç sunucusunun ayakta olduğundan emin ol (ilk sorguda
        # başlatılır; süreç öldüyse yeniden başlatılır).
        if not self._ensure_mcp_client():
            return

        # 3. "Yazıyor..." göstergesi.
        self.chat_history.append(
            "<div id='copilot-typing' style='margin-bottom: 8px; color:#64748b;'>"
            "<i>AI Copilot düşünüyor…</i></div>"
        )
        self._scroll_chat_bottom()

        # 4. UI thread'inden anlık veri snapshot'ı al (thread-güvenli okuma için).
        snapshot = self._build_copilot_snapshot()

        # 5. Gemini thread'ini başlat (araçlara MCP istemcisi üzerinden erişir).
        self.chat_thread = GeminiChatThread(
            api_key=self.gemini_api_key,
            history=self.chat_history_ctx,
            user_query=query,
            snapshot=snapshot,
            mcp_client=self.mcp_client,
            forced_tools=forced_tools,
        )
        self._pending_query = query
        self.chat_thread.response_ready.connect(self._on_copilot_response)
        self.chat_thread.error_occurred.connect(self._on_copilot_error)
        self.chat_thread.action_requested.connect(self._on_copilot_action)
        self.chat_thread.log_signal.connect(self.add_terminal_log)
        if hasattr(self, "mcp_tracker") and self.mcp_tracker is not None:
            self.chat_thread.tool_signal.connect(self.mcp_tracker.handle_tool_event)
        self.chat_thread.start()

    def _ensure_mcp_client(self):
        """MCP araç sunucusunu (mcp_server.py alt süreci) hazır eder. İlk
        sorguda başlatılır; süreç öldüyse yeniden başlatılır. Başlatılamazsa
        sohbete hata düşer ve False döner — araçsız Gemini veri uyduracağı
        için sorgu araçsız sürdürülmez."""
        if self.mcp_client is not None and self.mcp_client.is_alive():
            return True
        try:
            self.mcp_client = McpClient()
            info = self.mcp_client.start()
            self.add_terminal_log(
                f"MCP araç sunucusu başlatıldı: {info.get('name', '?')} "
                f"v{info.get('version', '?')} (stdio).", "INFO"
            )
            return True
        except (McpError, OSError) as exc:
            self.mcp_client = None
            self.add_terminal_log(f"MCP araç sunucusu başlatılamadı: {exc}", "WARN")
            self._append_copilot_html(
                "<span style='color:#ef4444;'>MCP araç sunucusu başlatılamadı.</span><br>"
                f"Ayrıntı: {exc}"
            )
            return False

    # AI Copilot'un "şu an aktif hedef" saydığı süre (saniye): bu pencerede
    # tekrar görülmemiş tespitler yalnızca geçmişte kalır.
    COPILOT_ACTIVE_WINDOW = 6.0

    def _build_copilot_snapshot(self):
        """AI Copilot araçlarının okuyacağı durum kopyası: anlık telemetri/tespitler
        + uçuş başından beri biriken geçmiş. Derin kopya alınır; Gemini thread'i
        UI verisini bloklamadan/yarışmadan okur."""
        vlm_text = ""
        if hasattr(self, "vlm_text_box"):
            vlm_text = self.vlm_text_box.toPlainText().strip()
        if not vlm_text:
            vlm_text = self.vlm_summaries[(self.vlm_index - 1) % len(self.vlm_summaries)]
        now = time.time()
        return {
            "telemetry": dict(self.telemetry),
            "telemetry_history": [dict(s) for s in self.telemetry_history],
            "detections": [dict(d) for d in self.live_detections],
            "active_detections": [
                dict(d) for d in self.live_detections
                if now - d.get("_seen", 0.0) <= self.COPILOT_ACTIVE_WINDOW
            ],
            "detection_log": [dict(d) for d in self.detection_log],
            "vlm_summary": vlm_text,
            "mission_time_s": now - self.mission_start_ts,
            # Görev başlangıç (kalkış/home) noktası: return_to_start MCP aracı
            # bunu hedef alır. SITL uçağı her koşuda bu sabit konumda doğar
            # (siha_control home'u ilk GPS kilidinden alır, aynı noktadır).
            "home": {"lat": START_LAT, "lon": START_LON},
        }

    def _remove_typing_indicator(self):
        """'Yazıyor…' geçici satırını sohbet geçmişinden temizler. Tek aktif
        sorgu olduğu için gösterge daima son bloktur; QTextCursor ile o bloğu
        (regex'ten daha güvenilir biçimde) siler."""
        cursor = self.chat_history.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.select(QTextCursor.BlockUnderCursor)
        if "AI Copilot düşünüyor" in cursor.selectedText():
            cursor.removeSelectedText()

    def _append_copilot_html(self, inner_html):
        self.chat_history.append(
            f"<div style='margin-bottom: 8px;'><b style='color:#10b981;'>SYSTEM COPILOT</b>"
            f"<br>{inner_html}</div>"
        )
        self._scroll_chat_bottom()

    def _scroll_chat_bottom(self):
        sb = self.chat_history.verticalScrollBar()
        sb.setValue(sb.maximum())

    @staticmethod
    def _markdown_to_html(text):
        """Gemini yanıtındaki sade Markdown'ı HTML'e çevirir: başlık, madde imi,
        kalın ve `kod`. Araç butonları yapılandırılmış rapor istediği için model
        sık sık '### Başlık' / '* madde' üretir; bunlar çevrilmezse ham görünür."""
        import re
        # HTML özel karakterlerini kaçır.
        text = (text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))

        lines = []
        for raw in text.split("\n"):
            stripped = raw.strip()
            indent = len(raw) - len(raw.lstrip(" "))

            heading = re.match(r"^#{1,6}\s+(.*)$", stripped)
            if heading:
                lines.append(f"<b style='color:#06b6d4;'>{heading.group(1)}</b>")
                continue

            # "* madde", "- madde", "1. madde" -> girintili •
            bullet = re.match(r"^(?:[*+-]|\d+\.)\s+(.*)$", stripped)
            if bullet:
                lines.append("&nbsp;" * (indent + 2) + f"• {bullet.group(1)}")
                continue

            lines.append(stripped)
        text = "<br>".join(lines)

        # **kalın** -> <b> (madde imleri ayrıştırıldıktan sonra).
        text = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", text)
        # `kod` -> vurgulu metin (hedef ID/koordinatlar için).
        text = re.sub(r"`([^`]+)`", r"<span style='color:#06b6d4;'>\1</span>", text)
        return text

    def _on_copilot_response(self, text):
        # stderr /dev/null'a yönlendirildiği için slot içi istisnalar sessizce
        # kaybolur ve "yanıt hiç gelmiyor" gibi görünür; her ihtimale karşı sar+logla.
        try:
            self._remove_typing_indicator()
            self._append_copilot_html(self._markdown_to_html(text))
            # Konuşma bağlamını güncelle (çok turlu diyalog için).
            query = getattr(self, "_pending_query", "")
            if query:
                self.chat_history_ctx.append({"role": "user", "parts": [{"text": query}]})
            self.chat_history_ctx.append({"role": "model", "parts": [{"text": text}]})
            # Bağlamı son ~12 mesajla sınırla (token/hafıza kontrolü).
            if len(self.chat_history_ctx) > 12:
                self.chat_history_ctx = self.chat_history_ctx[-12:]
            self.add_timeline_event("command", "AI: Taktik Sorgu Cevaplandı")
        except Exception as e:
            self.add_terminal_log(f"AI Copilot yanıtı işlenirken hata: {e}", "WARN")
            try:
                self._append_copilot_html(
                    "<span style='color:#ef4444;'>Yanıt gösterilemedi (iç hata).</span>"
                )
            except Exception:
                pass

    def _on_copilot_error(self, message):
        try:
            self._remove_typing_indicator()
            self._append_copilot_html(
                f"<span style='color:#ef4444;'>AI Copilot hatası:</span> {self._markdown_to_html(message)}"
            )
        except Exception as e:
            self.add_terminal_log(f"AI Copilot hata mesajı gösterilemedi: {e}", "WARN")
        self.add_terminal_log(f"AI Copilot hatası: {message}", "WARN")

    def _on_copilot_action(self, action, payload):
        """Gemini araçlarının talep ettiği UI eylemleri (ana thread'de çalışır)."""
        if action == "download_report":
            self.download_tactic_report()
            self.add_timeline_event("command", "AI: Taktik Raporu Derlendi")
        elif action == "start_flight":
            self.start_otonom_flight()
            self.add_timeline_event("command", "AI: Otomatik Pist Kalkışı Başlatıldı")
            self.add_terminal_log("Uçak motorları çalıştırıldı, otomatik kalkış ve otonom rota başlatıldı.", "INFO")
        elif action == "timeline_event":
            # MCP uçuş komutları (turn_heading/fly_forward/change_altitude)
            # gönderildiğinde zaman çizelgesine ve terminale iz düşülür.
            label = payload.get("label", "AI Uçuş Komutu")
            self.add_timeline_event("command", label)
            self.add_terminal_log(label, "INFO")

    def download_tactic_report(self):
        report_content = (
            f"--- AGENTIC DIGITAL-TWIN GCS TACTICAL REPORT ---\n"
            f"Generated At: {datetime.now().isoformat()}\n"
            f"Mission Duration: {(time.time() - self.mission_start_ts) / 60.0:.1f} min\n\n"
            f"Telemetry Summary (anlık):\n"
            f"- Altitude: {self.telemetry['alt']:.2f} m\n"
            f"- Speed: {self.telemetry['speed']:.2f} m/s\n"
            f"- Battery: {self.telemetry['battery']:.1f}% ({self.telemetry['voltage']}V)\n"
            f"- Coordinates: Lat {self.telemetry['lat']:.6f}, Lon {self.telemetry['lon']:.6f}\n"
            f"- Autopilot Mode: {self.telemetry['mode']}\n\n"
        )

        # Uçuş geçmişi (1 Hz örnek tamponundan türetilen min/maks/ortalama).
        hist = list(self.telemetry_history)
        if hist:
            alts = [s["alt"] for s in hist]
            speeds = [s["speed"] for s in hist]
            batts = [s["battery"] for s in hist]
            report_content += (
                f"Telemetry History ({len(hist)} örnek, {hist[-1]['t'] / 60.0:.1f} dk):\n"
                f"- Altitude  min/max/avg: {min(alts):.1f} / {max(alts):.1f} / {sum(alts)/len(alts):.1f} m\n"
                f"- Speed     min/max/avg: {min(speeds):.1f} / {max(speeds):.1f} / {sum(speeds)/len(speeds):.1f} m/s\n"
                f"- Battery   start -> now: {batts[0]:.0f}% -> {batts[-1]:.0f}%\n\n"
            )

        report_content += "Target Detection Logs (YOLO — uçuş başından bu yana):\n"
        for d in self.detection_log:
            qr_part = f" | QR: \"{d['qr_text']}\"" if d.get("qr_text") else ""
            report_content += (
                f"- ID: {d['id']} | Nesne: {d['type']} | Tepe Güven: %{int(d['conf']*100)} "
                f"| İlk: {d.get('first_time', '?')} | Son: {d['time']} "
                f"| GPS: {d['lat']:.5f}, {d['lon']:.5f}{qr_part}\n"
            )
        if not self.detection_log:
            report_content += "- (Tespit kaydedilmedi)\n"

        report_content += f"\nVision-Language Model Scene Summary:\n"
        report_content += f"\"{self.vlm_text_box.toPlainText()}\"\n"
        report_content += f"--------------------- END OF REPORT ---------------------\n"
        
        # Save dialog — varsayılan biçim GÖRSELLEŞTİRİLMİŞ PDF; düz metin
        # (ham veri, tam tespit listesi) seçenek olarak korunur. Raporlar proje
        # kökündeki reports/ klasöründe toplanır (kullanıcı pencereden başka bir
        # yer seçebilir).
        default_name = os.path.join(
            self._reports_dir(),
            f"Taktik_Rapor_{datetime.now().strftime('%Y-%m-%d_%H%M%S')}.pdf",
        )
        file_path, selected_filter = QFileDialog.getSaveFileName(
            self, "Taktik Rapor Kaydet", default_name,
            "PDF Raporu (*.pdf);;Text Files (*.txt);;All Files (*)"
        )
        if not file_path:
            return

        # Uzantı yoksa seçilen filtreye göre tamamla.
        if not os.path.splitext(file_path)[1]:
            file_path += ".txt" if "txt" in (selected_filter or "").lower() else ".pdf"

        if file_path.lower().endswith(".pdf"):
            try:
                pages = TacticalReportPdf(self._build_report_data()).save(file_path)
                self.add_terminal_log(
                    f"Görselleştirilmiş PDF raporu yazıldı ({pages} sayfa): "
                    f"{os.path.basename(file_path)}", "INFO"
                )
            except Exception as e:
                # PDF üretimi başarısızsa görev raporsuz kalmasın: yanına metin
                # sürümünü yaz ve kullanıcıyı bilgilendir.
                self.add_terminal_log(f"PDF raporu oluşturulamadı: {e} — metin sürümü yazılıyor.", "WARN")
                txt_path = os.path.splitext(file_path)[0] + ".txt"
                try:
                    with open(txt_path, "w", encoding="utf-8") as f:
                        f.write(report_content)
                    self.add_terminal_log(f"Metin raporu yazıldı: {os.path.basename(txt_path)}", "INFO")
                except Exception as e2:
                    self.add_terminal_log(f"Rapor dosyası kaydedilemedi: {e2}", "WARN")
            return

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(report_content)
            self.add_terminal_log(f"Rapor başarıyla diske yazıldı: {os.path.basename(file_path)}", "INFO")
        except Exception as e:
            self.add_terminal_log(f"Rapor dosyası kaydedilemedi: {str(e)}", "WARN")

    def _reports_dir(self):
        """Raporların toplandığı proje kökündeki reports/ klasörü; yoksa oluşturur.
        Oluşturulamazsa (izin vb.) proje köküne düşer, rapor kaydı hiç engellenmez."""
        path = os.path.join(BASE_DIR, "reports")
        try:
            os.makedirs(path, exist_ok=True)
        except OSError as e:
            self.add_terminal_log(f"reports/ klasörü oluşturulamadı ({e}); proje kökü kullanılacak.", "WARN")
            return BASE_DIR
        return path

    def _build_report_data(self):
        """PDF çizicisinin (interface/rapor.py) beklediği veri sözlüğünü derler.
        Uygulama durumunun anlık bir KOPYASIDIR; çizim sırasında canlı yapılar
        değişse bile rapor tutarlı kalır."""
        return {
            "generated_at": datetime.now(),
            "mission_min": (time.time() - self.mission_start_ts) / 60.0,
            "camera_mode": getattr(self, "active_camera_mode", "gazebo"),
            "telemetry": dict(self.telemetry),
            "history": [dict(s) for s in self.telemetry_history],
            "detections": [dict(d) for d in (self.detection_log or self.live_detections)],
            "timeline": [dict(e) for e in getattr(self, "timeline_events", [])],
            "waypoints": [dict(w) for w in getattr(self, "waypoints", [])],
            "vlm": self.vlm_text_box.toPlainText(),
        }

    # ------------------------------------------------------------------
    # CAMERA STREAM CONNECTOR
    # ------------------------------------------------------------------
    def start_camera_stream(self):
        """ROS kamera alıcısını (current_camera_source konusuyla) başlatır.
        Zaten çalışıyorsa dokunmaz."""
        if getattr(self, "camera_thread", None) is not None:
            return
        self.camera_thread = CameraStreamThread(self.current_camera_source)
        self.camera_thread.frame_received.connect(self.camera_sim.set_frame)
        self.camera_thread.log_signal.connect(self.add_terminal_log)
        self.camera_thread.start()

    def stop_camera_stream(self):
        """ROS kamera alıcısını durdurur. Durdurmadan ÖNCE sinyalleri keser:
        aksi halde kuyrukta bekleyen eski karelerin yeni kaynağın üstüne düşüp
        bir an eski görüntüyü göstermesine yol açıyordu."""
        thread = getattr(self, "camera_thread", None)
        if thread is None:
            return
        try:
            thread.frame_received.disconnect()
            thread.log_signal.disconnect()
        except (RuntimeError, TypeError):
            pass
        thread.stop()
        self.camera_thread = None

    def toggle_camera_stream(self):
        """ROS kamera akışını açar/kapatır (açılışta bir kez çağrılır)."""
        if getattr(self, "camera_thread", None) is None:
            self.start_camera_stream()
        else:
            self.stop_camera_stream()
            self.camera_sim.set_frame(None)  # Clear frame to show simulator fallback
    
    def start_otonom_flight(self):
        if hasattr(self, "otonom_flight_process") and self.otonom_flight_process is not None:
            if self.otonom_flight_process.poll() is None:
                self.add_terminal_log("Otonom uçuş kontrolcüsü zaten çalışıyor; ikinci başlatma engellendi.", "DEBUG")
                return
            self.otonom_flight_process = None

        self.add_terminal_log("Sürekli otonom düz uçuş süreci tetikleniyor...", "INFO")
        siha_log = open_component_log("siha_control.log", "AUTONOMOUS MISSION CONTROL (SIHA_CONTROL)")

        try:
            cmd = ["python3", "/home/delpiyero/Staj/siha_control.py"]
            self.otonom_flight_process = subprocess.Popen(
                cmd,
                stdout=siha_log,
                stderr=siha_log,
                preexec_fn=os.setsid
            )
            self.add_terminal_log("Otonom uçuş kontrolcüsü başarıyla başlatıldı.", "INFO")
            # Uçuş başladığı anda YOLO tespitini de başlat
            self.start_yolo_detection()
        except Exception as e:
            self.add_terminal_log(f"Otonom uçuş başlatılamadı: {e}", "WARN")

    def start_yolo_detection(self):
        """Uçuşla birlikte YOLO tespit düğümünü (yolo.py) ayrı süreç olarak
        başlatır. yolo.py alt kamera akışını (/bottom_camera/image) dinler; uçak
        cismin üzerinden geçerken tespitleri yolo.log dosyasına yazar."""
        if hasattr(self, "yolo_process") and self.yolo_process is not None:
            if self.yolo_process.poll() is None:
                self.add_terminal_log("YOLO tespit süreci zaten çalışıyor; ikinci başlatma engellendi.", "DEBUG")
                return
            self.yolo_process = None

        self.add_terminal_log("YOLO tespit düğümü başlatılıyor (model yükleniyor, birkaç saniye sürebilir)...", "INFO")
        yolo_log = open_component_log("yolo.log", "YOLO OBJECT DETECTION")

        # OpenCV'nin Qt eklenti kirliliğini önlemek için temiz ortam kopyası
        yolo_env = os.environ.copy()
        yolo_env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
        # 8 GB RAM / GTX 1650 için kaynak paylaşımı: BLAS/OMP kütüphanelerinin tüm
        # çekirdekleri kaplamasını engelle ki Gazebo fiziği + MAVROS/EKF aç kalmasın.
        # yolo.py bu değerleri (YOLO_*) okuyup cihaz/FPS/thread ayarını yapar.
        for _k, _v in {
            "OMP_NUM_THREADS": "4",
            "OPENBLAS_NUM_THREADS": "4",
            "MKL_NUM_THREADS": "4",
            "NUMEXPR_NUM_THREADS": "4",
            "YOLO_CPU_THREADS": "4",
            "YOLO_MAX_FPS": "4",
            # Kamera 320x240 native; imgsz'i 512'ye büyütmek ~10-25 px havadan
            # cisimlere stride-grid'de daha çok hücre verir → kenarda/kırpık
            # yakalanan küçük cisimlerin güveni 0.10 eşiğinin üstüne çıkar.
            # (416'da T-07/08/09 sadece eşik-altı 0.06-0.08 üretip kaçmıştı.)
            "YOLO_IMGSZ": "512",
        }.items():
            yolo_env.setdefault(_k, _v)

        try:
            cmd = ["python3", "/home/delpiyero/Staj/yolo.py"]
            self.yolo_process = subprocess.Popen(
                cmd,
                stdout=yolo_log,
                stderr=yolo_log,
                env=yolo_env,
                cwd=BASE_DIR,  # yolo.py modeli göreli yolla yüklüyor; CWD'yi projeye sabitle
                preexec_fn=os.setsid
            )
            self.add_terminal_log("YOLO tespit düğümü başlatıldı (alt kamera izleniyor).", "INFO")
            # Model yüklenip yayına başladıktan sonra GUI kamerasını kutulanmış
            # (bounding box'lı) /yolo/image_annotated akışına geçir — ama yalnızca
            # kullanıcı hâlâ alt kamerayı seçiliyken (ön kameraya geçtiyse ezme).
            QTimer.singleShot(6000, self._switch_to_annotated_if_bottom)
        except Exception as e:
            self.add_terminal_log(f"YOLO tespit düğümü başlatılamadı: {e}", "WARN")

    def _switch_to_annotated_if_bottom(self):
        """YOLO düğümü yayına başlayınca alt kamera görüşünü kutulanmış akışa
        yükseltir; kullanıcı bu arada ön kameraya geçtiyse dokunmaz."""
        if self.camera_views[self.camera_view_index][0] != "alt":
            return
        self.switch_camera_source("/yolo/image_annotated")

    # --- Gazebo kamera görüşü (alt / ön) --------------------------------
    def camera_topic_for(self, key):
        """Görüş anahtarını (alt/on) o an geçerli ROS topic'ine çevirir.
        Alt kamera: YOLO düğümü çalışıyorsa kutulanmış akış, aksi halde ham akış
        (aksi halde yayıncısı olmayan bir topic'e bağlanıp ekran donuk kalıyordu)."""
        if key == "on":
            return "/camera/image"
        yolo = getattr(self, "yolo_process", None)
        if yolo is not None and yolo.poll() is None:
            return "/yolo/image_annotated"
        return "/bottom_camera/image"

    @staticmethod
    def camera_key_for_topic(topic):
        """Topic → görüş anahtarı. Ön kamera dışındaki her şey alt kameradır."""
        return "on" if topic == "/camera/image" else "alt"

    def switch_camera_source(self, topic):
        """GUI kamera akışını verilen ROS topic'ine yeniden bağlar (ör. YOLO'nun
        kutulanmış /yolo/image_annotated yayınına geçmek için). Buton etiketini
        ve seçili görüşü topic'e göre eşitler."""
        key = self.camera_key_for_topic(topic)
        label = dict(self.camera_views).get(key, key)
        self.current_camera_source = topic
        for i, (k, _lbl) in enumerate(self.camera_views):
            if k == key:
                self.camera_view_index = i
                break
        if hasattr(self, "btn_camera_toggle"):
            self.btn_camera_toggle.setText(f"Kamera: {label}")

        # Video modundayken ROS akışını ekrana bağlama: iki kaynak aynı
        # widget'a kare basıp birbirinin görüntüsünü eziyordu. Seçim saklanır,
        # Gazebo'ya dönüldüğünde bu kaynakla bağlanılır.
        if getattr(self, "active_camera_mode", "gazebo") == "video":
            self.add_terminal_log(f"Kamera kaynağı {topic} olarak kaydedildi (video modunda bağlanmadı).", "DEBUG")
            return

        self.stop_camera_stream()
        self.camera_sim.set_source_label("GAZEBO", busy=True)
        self.camera_sim.set_frame(None)  # eski görüşün karesi ekranda kalmasın
        self.camera_status(f"{label} akışına geçiliyor ({topic})...", "INFO")
        self.start_camera_stream()
        self._watch_camera_stream(topic, label)

    def _watch_camera_stream(self, topic, label):
        """Geçişten sonra kare gelip gelmediğini denetler; gelmiyorsa kullanıcıyı
        görüntü üstünde uyarır (ör. YOLO düğümü henüz yayına başlamamış olabilir)."""
        start_count = self.camera_sim.frame_count

        def check():
            if self.current_camera_source != topic or getattr(self, "active_camera_mode", "gazebo") != "gazebo":
                return  # bu arada başka kaynağa geçilmiş
            if self.camera_sim.frame_count > start_count:
                self.camera_sim.set_source_label("GAZEBO", busy=False)
                self.camera_status(f"Aktif kamera: {label} ({topic})", "OK")
            else:
                self.camera_sim.set_source_label("GAZEBO", busy=False)
                self.camera_status(
                    f"{label} ({topic}) yayını gelmiyor — konu henüz yayınlanmıyor olabilir.",
                    "WARN",
                )

        QTimer.singleShot(3000, check)

    def toggle_camera_view(self):
        """'Kamera: Alt/Ön Kamera' butonu: Gazebo akışında ön kamera ile alt kamera
        arasında geçiş yapar. Video (drone.mp4) modunda ROS akışı ekranda olmadığı
        için yalnızca seçim kaydedilir, kullanıcı görüntü üstünde uyarılır."""
        self.camera_view_index = (self.camera_view_index + 1) % len(self.camera_views)
        key, label = self.camera_views[self.camera_view_index]
        topic = self.camera_topic_for(key)
        if getattr(self, "active_camera_mode", "gazebo") == "video":
            self.current_camera_source = topic
            self.btn_camera_toggle.setText(f"Kamera: {label}")
            self.camera_status(
                f"{label} seçildi — Gazebo kamerasına dönünce etkin olacak.", "WARN"
            )
            return
        self.switch_camera_source(topic)

    def camera_status(self, text, level="INFO", sticky=False, terminal=True):
        """Kamera kaynağı geçişiyle ilgili log satırını GÖRÜNTÜNÜN ÜZERİNDE
        gösterir (istenirse terminal loguna da düşürür)."""
        if hasattr(self, "camera_sim"):
            self.camera_sim.push_status(text, level, sticky=sticky)
        if terminal:
            self.add_terminal_log(text, "INFO" if level in ("INFO", "OK") else level)

    def toggle_video_yolo(self):
        """Kamera geçiş butonu: Gazebo simülasyon kamerası ile yerel drone.mp4
        video akışı arasında geçiş yapar. Geçiş esnasında hem görüntü kaynağı hem de
        altındaki tespit tablosu/logları ilgili ekrana (ve geçmiş kayıtlarına) özel filtreler.
        Geçiş logları görüntünün üzerindeki durum katmanında gösterilir."""
        if self.active_camera_mode == "video" or (getattr(self, "video_yolo_thread", None) is not None and self.video_yolo_thread.isRunning()):
            self.camera_sim.clear_status()
            self.camera_sim.set_source_label("GAZEBO", busy=True)
            self.camera_status("Video akışı durduruluyor, Gazebo kamerasına dönülüyor...", "WARN")
            if getattr(self, "video_yolo_thread", None) is not None:
                self.video_yolo_thread.stop()
                self.video_yolo_thread = None

            self.active_camera_mode = "gazebo"
            self.live_detections = self.gazebo_live_detections
            self.detection_log = self.gazebo_detection_log
            self._track_rows = self.gazebo_track_rows

            self.btn_video_toggle.setText("Video'ya Geç")
            self.btn_video_toggle.setStyleSheet(
                "font-size: 9px; padding: 0 10px; background-color: #10b981; color: white; border: none; font-weight: bold; border-radius: 2px;"
            )
            # Seçili görüşün O ANKİ topic'ine bağlan (YOLO düğümü kapandıysa ham
            # alt kamera akışına düşer; yayıncısı olmayan konuya asılı kalmaz).
            sel_key = self.camera_views[self.camera_view_index][0]
            self.switch_camera_source(self.camera_topic_for(sel_key))
            self.update_detections_table()
            self.update_vlm_from_log()
            self.camera_sim.set_source_label("GAZEBO", busy=False)
            self.camera_status("Aktif kamera: Gazebo Simülasyonu. Gazebo tespit logları yüklendi.", "OK")
            return

        video_file = os.path.join(BASE_DIR, "drone.mp4")
        if not os.path.exists(video_file):
            self.camera_status(f"Canlı video dosyası bulunamadı: {video_file}", "ERROR")
            return

        self.btn_video_toggle.setText("⏳ Video Kameraya Geçiliyor...")
        self.btn_video_toggle.setStyleSheet(
            "font-size: 9px; padding: 0 10px; background-color: #f59e0b; color: white; border: none; font-weight: bold; border-radius: 2px;"
        )
        # Yükleme bitene kadar butonu kilitle: art arda basınca thread.stop()
        # model yüklemesinin bitmesini beklediği için arayüz donuyordu.
        self.btn_video_toggle.setEnabled(False)
        self.camera_sim.clear_status()
        self.camera_sim.set_source_label("VİDEO", busy=True)
        self.camera_status("drone.mp4 video kaynağına geçiliyor, YOLOE modeli hazırlanıyor...", "WARN", sticky=True)
        # Gazebo ROS akışını kes: iki kaynak aynı widget'a kare basarsa görüntü
        # video ile Gazebo arasında titriyor.
        self.stop_camera_stream()

        self.video_yolo_thread = VideoYoloThread(video_path=video_file, parent=self)
        self.video_yolo_thread.frame_received.connect(self._on_video_frame)
        self.video_yolo_thread.stream_started.connect(self._on_video_yolo_started)
        self.video_yolo_thread.load_failed.connect(self._on_video_yolo_failed)
        self.video_yolo_thread.detections_ready.connect(self.handle_video_detections)
        # Thread'in yükleme/hata logları hem terminale hem görüntü üstündeki
        # durum katmanına düşer ki kullanıcı ilerlemeyi görüntü alanında görsün.
        self.video_yolo_thread.log_signal.connect(
            lambda msg, lvl: self.camera_status(msg, "WARN" if lvl == "WARN" else "INFO")
        )
        self.video_yolo_thread.start()

    def _on_video_frame(self, q_img):
        """Video karesini ekrana basar ve thread'in geri-basınç sayacını düşürür.
        Sayaç sayesinde arayüz yoğunken video thread'i kare yayınlamayı keser;
        aksi halde Qt kuyruğunda kare birikip görüntü giderek geriden geliyordu."""
        self.camera_sim.set_frame(q_img)
        th = getattr(self, "video_yolo_thread", None)
        if th is not None:
            th.frame_consumed()

    def _on_video_yolo_started(self):
        self.active_camera_mode = "video"
        self.live_detections = self.video_live_detections
        self.detection_log = self.video_detection_log
        self._track_rows = self.video_track_rows

        self.btn_video_toggle.setText("Gazebo'ya Geç")
        self.btn_video_toggle.setStyleSheet(
            "font-size: 9px; padding: 0 10px; background-color: #06b6d4; color: white; border: none; font-weight: bold; border-radius: 2px;"
        )
        self.update_detections_table()
        self.update_vlm_from_log()
        # Telemetriyi 1 sn'lik tik'i beklemeden hemen boşalt (video karesiyle
        # birlikte eski SITL değerleri bir an ekranda kalmasın).
        self._blank_telemetry_ui()
        self.btn_video_toggle.setEnabled(True)
        self.camera_sim.clear_status()
        self.camera_sim.set_source_label("VİDEO", busy=False)
        self.camera_status("Aktif kamera: Video Akışı (drone.mp4). Video tespit logları yüklendi.", "OK")

    def _on_video_yolo_failed(self, err_msg):
        self.active_camera_mode = "gazebo"
        self.live_detections = self.gazebo_live_detections
        self.detection_log = self.gazebo_detection_log
        self._track_rows = self.gazebo_track_rows

        self.btn_video_toggle.setText("🎥 Kamera: Gazebo (Video'ya Geç)")
        self.btn_video_toggle.setStyleSheet(
            "font-size: 9px; padding: 0 10px; background-color: #10b981; color: white; border: none; font-weight: bold; border-radius: 2px;"
        )
        self.update_detections_table()
        self.update_vlm_from_log()
        self.btn_video_toggle.setEnabled(True)
        self.camera_sim.clear_status()
        self.camera_sim.set_source_label("GAZEBO", busy=False)
        self.camera_status(f"Video kamerası başlatılamadı: {err_msg} — Gazebo'ya dönüldü.", "ERROR")
        self.video_yolo_thread = None
        # Kesilen Gazebo akışını geri bağla.
        self.switch_camera_source(self.current_camera_source)

    def handle_video_detections(self, data):
        """drone.mp4 canlı video YOLOE tespit verisini kendi video tespit logunda tutar ve aktifse tabloyu günceller."""
        raw_dets = data.get("detections", []) if isinstance(data, dict) else []
        if not raw_dets:
            return
        now_ts = time.time()
        now_str = datetime.now().strftime("%H:%M:%S")
        lat = self.telemetry["lat"]
        lon = self.telemetry["lon"]
        alt = self.telemetry.get("alt")
        changed = False

        for d in raw_dets:
            cls = d.get("cls", "nesne")
            conf = float(d.get("conf", 0.90))
            track_id = d.get("track_id")
            bbox = d.get("bbox")
            category = d.get("category") or cls
            type_tr = self._cls_to_tr(category)

            matched = None
            if track_id is not None and track_id in self.video_track_rows:
                matched = self.video_track_rows[track_id]
            if matched is None and bbox is not None:
                for e in self.video_live_detections:
                    if now_ts - e.get("_seen", 0.0) > self.DET_MERGE_WINDOW:
                        continue
                    if self._bbox_iou(bbox, e.get("bbox")) >= self.DET_MERGE_IOU:
                        matched = e
                        break

            if matched is not None:
                matched["_seen"] = now_ts
                matched["time"] = now_str
                matched["lat"], matched["lon"], matched["alt"] = lat, lon, alt
                matched["category"] = category
                if bbox is not None:
                    matched["bbox"] = bbox
                if track_id is not None:
                    self.video_track_rows[track_id] = matched
                if conf > matched["conf"]:
                    matched["conf"] = conf
                    matched["type"] = type_tr
                changed = True
                continue

            # Yeni video tespiti
            self.video_det_counter += 1
            target_id = f"V-{self.video_det_counter:02d}"
            entry = {
                "id": target_id,
                "type": type_tr,
                "category": category,
                "conf": conf,
                "time": now_str,
                "first_time": now_str,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "bbox": bbox,
                "hits": 1,
                "_seen": now_ts,
                "source": "video",
            }
            self.video_live_detections.append(entry)
            self.video_detection_log.append(entry)
            if len(self.video_detection_log) > 200:
                self.video_detection_log.pop(0)
            if track_id is not None:
                self.video_track_rows[track_id] = entry
            if len(self.video_live_detections) > 25:
                removed = self.video_live_detections.pop(0)
                for tid, e in list(self.video_track_rows.items()):
                    if e is removed:
                        del self.video_track_rows[tid]
            changed = True
            self.add_timeline_event("detection", f"Video YOLO tespiti: {type_tr} (%{int(conf*100)})")
            self.add_terminal_log(f"Video YOLO tespiti [{target_id}]: {type_tr} — güven %{int(conf*100)}", "INFO")

        if self.active_camera_mode == "video":
            self.live_detections = self.video_live_detections
            self.detection_log = self.video_detection_log
            self._track_rows = self.video_track_rows
            if changed:
                self.update_detections_table()
                QTimer.singleShot(100, self.update_vlm_from_log)

    def closeEvent(self, event):
        # Kamera yayınını durdur
        if hasattr(self, "camera_thread") and self.camera_thread is not None:
            self.camera_thread.stop()

        # Canlı video (drone.mp4) thread'ini durdur
        if getattr(self, "video_yolo_thread", None) is not None:
            self.video_yolo_thread.stop()

        # MCP araç sunucusunu durdur
        if getattr(self, "mcp_client", None) is not None:
            self.mcp_client.stop()

        # Sahne özeti veritabanını kapat
        if getattr(self, "scene_db", None) is not None:
            self.scene_db.close()
    

        if hasattr(self, "otonom_flight_process") and self.otonom_flight_process is not None:
            try:
                os.killpg(os.getpgid(self.otonom_flight_process.pid), signal.SIGTERM)
            except Exception:
                pass

        # YOLO tespit sürecini durdur
        if hasattr(self, "yolo_process") and self.yolo_process is not None:
            try:
                os.killpg(os.getpgid(self.yolo_process.pid), signal.SIGTERM)
            except Exception:
                pass

        # Telemetri alıcısını durdur
        if hasattr(self, "telemetry_thread") and self.telemetry_thread is not None:
            self.telemetry_thread.stop()

        # YOLO tespit alıcısını durdur
        if hasattr(self, "detection_thread") and self.detection_thread is not None:
            self.detection_thread.stop()
            
        # Gazebo sürecini ve alt süreçlerini sonlandır
        if hasattr(self, "gazebo_process") and self.gazebo_process is not None:
            try:
                self.add_terminal_log("Gazebo kapatılıyor...", "INFO")
                os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)
                self.gazebo_process.wait(timeout=3)
            except Exception as e:
                print(f"Gazebo sonlandırılamadı: {e}")

        # ArduPilot SITL sürecini ve alt süreçlerini (MAVProxy vb.) sonlandır
        if hasattr(self, "ardupilot_process") and self.ardupilot_process is not None:
            try:
                self.add_terminal_log("ArduPilot SITL kapatılıyor...", "INFO")
                os.killpg(os.getpgid(self.ardupilot_process.pid), signal.SIGTERM)
                self.ardupilot_process.wait(timeout=3)
            except Exception as e:
                print(f"ArduPilot sonlandırılamadı: {e}")

        # ROS-Gazebo köprüsü sürecini ve alt süreçlerini sonlandır
        if hasattr(self, "bridge_process") and self.bridge_process is not None:
            try:
                self.add_terminal_log("ROS-Gazebo Köprüsü kapatılıyor...", "INFO")
                os.killpg(os.getpgid(self.bridge_process.pid), signal.SIGTERM)
                self.bridge_process.wait(timeout=2)
            except Exception as e:
                print(f"ROS-Gazebo Köprüsü sonlandırılamadı: {e}")

        # MAVROS sürecini ve alt süreçlerini sonlandır
        if hasattr(self, "mavros_process") and self.mavros_process is not None:
            try:
                self.add_terminal_log("MAVROS kapatılıyor...", "INFO")
                os.killpg(os.getpgid(self.mavros_process.pid), signal.SIGTERM)
                self.mavros_process.wait(timeout=2)
            except Exception as e:
                print(f"MAVROS sonlandırılamadı: {e}")
                
        # Force-kill remaining processes to avoid port-binding issues for next launch
        try:
            subprocess.run("pkill -9 -f mavproxy || true", shell=True)
            subprocess.run("pkill -9 -f arduplane || true", shell=True)
            subprocess.run("pkill -9 -f sim_vehicle || true", shell=True)
            subprocess.run("pkill -9 -f ros_gz_bridge || true", shell=True)
            subprocess.run("pkill -9 -f gz-sim || true", shell=True)
            subprocess.run("pkill -9 -f apm.launch || true", shell=True)
            subprocess.run("pkill -9 -f siha_control.py || true", shell=True)
            subprocess.run("pkill -9 -f yolo.py || true", shell=True)
        except Exception:
            pass

        # Shutdown ROS 2 context
        try:
            import rclpy
            if rclpy.ok():
                rclpy.try_shutdown()
        except Exception:
            pass

        event.accept()


# ----------------------------------------------------------------------
# APPLICATION RUNTIME ENTRY POINT
# ----------------------------------------------------------------------
if __name__ == "__main__":
    # Initialize ROS 2
    if not rclpy.ok():
        rclpy.init()

    # Create Qt App instance
    app = QApplication(sys.argv)
    
    # Instantiate layout windows
    window = GroundControlStation()
    window.show()
    
    # Enable clean SIGINT (Ctrl+C) handling
    import signal
    def sigint_handler(sig, frame):
        logger.info("Terminalden Ctrl+C (SIGINT) sinyali alındı. Uygulama ve tüm arka plan süreçleri kapatılıyor...")
        window.close()
        QApplication.quit()
        
    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)
    
    # Dummy timer to regularly yield control to the Python interpreter to capture Ctrl+C
    sig_timer = QTimer()
    sig_timer.start(250)
    sig_timer.timeout.connect(lambda: None)
    
    # Run loop
    sys.exit(app.exec())
