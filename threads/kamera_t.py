import cv2
import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROS_LOG_DIR = os.environ.get("ROS_LOG_DIR", os.path.join(BASE_DIR, "logs", "ros"))
os.makedirs(ROS_LOG_DIR, exist_ok=True)
os.environ.setdefault("ROS_LOG_DIR", ROS_LOG_DIR)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

class CameraStreamThread(QThread):
    frame_received = Signal(QImage)
    log_signal = Signal(str, str)

    def __init__(self, topic="/camera/image", parent=None):
        super().__init__(parent)
        self.topic = topic
        self.running = False
        self.bridge = CvBridge()

    def run(self):
        self.running = True
        self.log_signal.emit(f"ROS 2 Kamera Alıcısı başlatılıyor. Konu: {self.topic}", "INFO")
        
        if not rclpy.ok():
            rclpy.init()
        
        node = Node("gcs_camera_listener")
        
        def listener_callback(msg):
            try:
                # ROS Image mesajını OpenCV BGR formatına dönüştür
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                # OpenCV BGR formatından Qt RGB formatına dönüştür
                rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                q_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
                self.frame_received.emit(q_img)
            except Exception as e:
                self.log_signal.emit(f"Görüntü dönüştürme hatası: {str(e)}", "WARN")

        # Best effort QoS profile for high-bandwidth image stream
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5
        )

        # ROS 2 aboneliğini oluştur
        node.create_subscription(ROSImage, self.topic, listener_callback, qos_profile)

        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)

        # Thread durdurulana kadar ROS 2 düğümünü döndür
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
