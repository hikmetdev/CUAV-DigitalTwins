import math
import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROS_LOG_DIR = os.environ.get("ROS_LOG_DIR", os.path.join(BASE_DIR, "logs", "ros"))
os.makedirs(ROS_LOG_DIR, exist_ok=True)
os.environ.setdefault("ROS_LOG_DIR", ROS_LOG_DIR)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import UInt32
from PySide6.QtCore import QThread, Signal

# LiPo hücre gerilim eşikleri (V/hücre). Batarya yüzdesi otopilottan gelmediğinde
# (ArduPilot SITL sık sık percentage=-1 döndürür → eskiden %100'e sabit kalıyordu)
# gerilimden kestirmek için kullanılır. Renk tonlaması app.py'de aynı eşiklerle.
CELL_FULL_V = 4.20    # tam dolu hücre
CELL_EMPTY_V = 3.30   # boş/kesim hücre
# Hücre sayısını gerilimden kestirme yerine sabitlemek için (ör. 6S = 6): env.
_FORCED_CELLS = int(os.environ.get("BATTERY_CELLS", "0") or 0)

# SITL düz-uçak (ArduPlane) + Gazebo/JSON modelinde ölçülen gerilim SoC ile
# DÜŞMEZ — yalnız gaza bağlı sarkar (voltage = SIM_BATT_VOLTAGE - 0.7*gaz);
# kapasiteye bağlı gerilim düşüşü sadece Multicopter/QuadPlane'de var. Gerçekten
# zamanla düşen tek büyüklük YÜZDE'dir (consumed_mah/BATT_CAPACITY). Bu bayrak
# açıkken görüntülenen gerilim gerçek yüzdeden (SoC) LiPo eğrisiyle türetilir →
# gerilim de görünür düşer ve app.py'deki hücre-başına renk kademesi akar.
# Gerçek donanımda ham gerilimi korumak için BATTERY_VOLTAGE_FROM_SOC=0 yapın.
_VOLTAGE_FROM_SOC = os.environ.get("BATTERY_VOLTAGE_FROM_SOC", "1").strip().lower() \
    not in ("0", "false", "no", "off", "")


def percent_from_voltage(voltage, cells):
    """Paket gerilimi + hücre sayısından yaklaşık batarya yüzdesi (0-100).
    Doğrusal LiPo yaklaşımı (3.30 V=%0 .. 4.20 V=%100). Bilinmiyorsa None."""
    if not voltage or cells <= 0:
        return None
    per_cell = voltage / cells
    pct = (per_cell - CELL_EMPTY_V) / (CELL_FULL_V - CELL_EMPTY_V) * 100.0
    return max(0, min(100, int(round(pct))))


def voltage_from_percent(pct, cells):
    """percent_from_voltage'ın tersi: yüzde (0-100) + hücre sayısından yaklaşık
    paket gerilimi (V, 0.1 hassasiyet). Doğrusal LiPo (%0=3.30 V .. %100=4.20 V/
    hücre). SITL'de ham gerilim SoC ile düşmediğinden, gerilimi gerçek yüzdeden
    türetip görünür düşüş sağlamak için kullanılır. Bilinmiyorsa None."""
    if pct is None or cells <= 0:
        return None
    pct = max(0.0, min(100.0, pct))
    per_cell = CELL_EMPTY_V + (pct / 100.0) * (CELL_FULL_V - CELL_EMPTY_V)
    return round(per_cell * cells, 1)


class TelemetryStreamThread(QThread):
    telemetry_received = Signal(dict)
    log_signal = Signal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = False
        self.data = {
            "alt": 0.0,
            "speed": 0.0,
            "battery": 100,
            "voltage": 24.0,       # görüntülenen gerilim (SoC'den türetilebilir)
            "voltage_raw": 24.0,   # otopilottan gelen ham gerilim (hücre kestirimi)
            "battery_cells": _FORCED_CELLS,  # 0 = gerilimden otomatik kestirilir
            "mode": "DISCONNECTED",
            "lat": 39.920782,
            "lon": 32.854115,
            "heading": 90,
            "pitch": 0.0,
            "roll": 0.0,
            "sats": 0,
        }

    def run(self):
        self.running = True
        self.log_signal.emit("MAVROS Telemetri Alıcısı başlatılıyor...", "INFO")
        
        if not rclpy.ok():
            rclpy.init()
        
        node = Node("gcs_telemetry_listener")
        
        def state_cb(msg):
            # FCU bağlantısı kopmuşsa son modu "canlı" gibi göstermeyip DISCONNECTED
            # bildir — böylece mod alanı gerçekten güncel durumu yansıtır.
            connected = getattr(msg, "connected", True)
            self.data["mode"] = msg.mode if (connected and msg.mode) else "DISCONNECTED"
            self.telemetry_received.emit(self.data.copy())

        def gps_cb(msg):
            if msg.latitude != 0.0 and msg.longitude != 0.0 and not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
                self.data["lat"] = msg.latitude
                self.data["lon"] = msg.longitude
                self.telemetry_received.emit(self.data.copy())

        def sats_cb(msg):
            self.data["sats"] = int(msg.data)
            self.telemetry_received.emit(self.data.copy())

        def battery_cb(msg):
            raw_voltage = round(msg.voltage, 1) if not math.isnan(msg.voltage) else 0.0
            if raw_voltage > 0:
                self.data["voltage_raw"] = raw_voltage
                # Hücre sayısı sabitlenmemişse gözlenen EN YÜKSEK gerilimden kestir
                # (gerilim hücre×4.20'yi aşmaz; SITL/uçuş dolu başladığı için tutar).
                if not _FORCED_CELLS:
                    est_cells = max(1, round(raw_voltage / CELL_FULL_V))
                    if est_cells > self.data["battery_cells"]:
                        self.data["battery_cells"] = est_cells
            # Yüzde otopilottan geçerli geldiyse onu kullan; gelmezse (SITL'de sık
            # sık -1) HAM gerilimden kestir — eskiden %100'e sabit kalıp donuyordu.
            pct = msg.percentage
            have_pct = pct is not None and pct >= 0.0 and not math.isnan(pct)
            if have_pct:
                self.data["battery"] = int(pct * 100)
            else:
                est = percent_from_voltage(self.data["voltage_raw"], self.data["battery_cells"])
                if est is not None:
                    self.data["battery"] = est
            # Görüntülenen gerilim: SITL düz-uçak modelinde ham gerilim SoC ile
            # düşmediğinden, _VOLTAGE_FROM_SOC açık + geçerli yüzde varsa gerilimi
            # gerçek yüzdeden LiPo eğrisiyle türet (zamanla görünür düşer). Aksi
            # halde otopilotun ham gerilimini göster.
            derived = (voltage_from_percent(pct * 100.0, self.data["battery_cells"])
                       if (_VOLTAGE_FROM_SOC and have_pct) else None)
            if derived is not None:
                self.data["voltage"] = derived
            elif raw_voltage > 0:
                self.data["voltage"] = raw_voltage
            self.telemetry_received.emit(self.data.copy())
            
        def pose_cb(msg):
            self.data["alt"] = msg.pose.position.z
            q = msg.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            # MAVROS local_position/pose ENU'dur: yaw Doğu ekseninden CCW ölçülür
            # (kuzeye uçarken yaw=+90°). Tüketiciler (pusula, harita, etiket)
            # pusula açısı bekliyor: 0=Kuzey, saat yönünde artan. Dönüşüm
            # yapılmazsa semboller 90° yan duruyordu.
            heading = int(round(90.0 - math.degrees(yaw))) % 360
            self.data["heading"] = heading
            # Yönelim grafiği için gövde tutumu (derece): roll (x) ve pitch (y).
            sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            self.data["roll"] = round(math.degrees(math.atan2(sinr_cosp, cosr_cosp)), 1)
            sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            self.data["pitch"] = round(math.degrees(math.asin(sinp)), 1)
            self.telemetry_received.emit(self.data.copy())

        def velocity_cb(msg):
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            vz = msg.twist.linear.z
            self.data["speed"] = math.sqrt(vx*vx + vy*vy + vz*vz)
            self.telemetry_received.emit(self.data.copy())

        # Separate QoS profiles for reliable state topic and best-effort sensor streams
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # MAVROS Subscriptions
        node.create_subscription(State, "/mavros/state", state_cb, qos_reliable)
        node.create_subscription(NavSatFix, "/mavros/global_position/global", gps_cb, qos_best_effort)
        node.create_subscription(UInt32, "/mavros/global_position/raw/satellites", sats_cb, qos_best_effort)
        node.create_subscription(BatteryState, "/mavros/battery", battery_cb, qos_best_effort)
        node.create_subscription(PoseStamped, "/mavros/local_position/pose", pose_cb, qos_best_effort)
        node.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", velocity_cb, qos_best_effort)

        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)

        while self.running and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.05)
            except Exception:
                break

        self.executor.remove_node(node)
        node.destroy_node()

    def stop(self):
        self.running = False
        self.wait()
