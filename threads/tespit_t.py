import os
import json

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROS_LOG_DIR = os.environ.get("ROS_LOG_DIR", os.path.join(BASE_DIR, "logs", "ros"))
os.makedirs(ROS_LOG_DIR, exist_ok=True)
os.environ.setdefault("ROS_LOG_DIR", ROS_LOG_DIR)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from PySide6.QtCore import QThread, Signal


class DetectionStreamThread(QThread):
    """yolo.py'nin /yolo/detections topic'ine yayınladığı tespit JSON'larını
    dinler ve her tespit paketini Qt sinyali ile GUI'ye iletir."""
    detection_received = Signal(dict)
    log_signal = Signal(str, str)

    def __init__(self, topic="/yolo/detections", parent=None):
        super().__init__(parent)
        self.topic = topic
        self.running = False

    def run(self):
        self.running = True
        self.log_signal.emit(f"YOLO tespit alıcısı başlatılıyor. Konu: {self.topic}", "INFO")

        if not rclpy.ok():
            rclpy.init()

        node = Node("gcs_detection_listener")

        def det_cb(msg):
            try:
                data = json.loads(msg.data)
                self.detection_received.emit(data)
            except Exception as e:
                self.log_signal.emit(f"Tespit mesajı çözümlenemedi: {e}", "WARN")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        node.create_subscription(String, self.topic, det_cb, qos_profile)

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
