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
from PySide6.QtCore import QThread, Signal

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
            "voltage": 24.0,
            "mode": "DISCONNECTED",
            "lat": 39.920782,
            "lon": 32.854115,
            "heading": 90
        }

    def run(self):
        self.running = True
        self.log_signal.emit("MAVROS Telemetri Alıcısı başlatılıyor...", "INFO")
        
        if not rclpy.ok():
            rclpy.init()
        
        node = Node("gcs_telemetry_listener")
        
        def state_cb(msg):
            self.data["mode"] = msg.mode
            self.telemetry_received.emit(self.data.copy())
            
        def gps_cb(msg):
            if msg.latitude != 0.0 and msg.longitude != 0.0 and not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
                self.data["lat"] = msg.latitude
                self.data["lon"] = msg.longitude
                self.telemetry_received.emit(self.data.copy())
            
        def battery_cb(msg):
            self.data["battery"] = int(msg.percentage * 100) if msg.percentage >= 0 else 100
            self.data["voltage"] = round(msg.voltage, 1)
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
