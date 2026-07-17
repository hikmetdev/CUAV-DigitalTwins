import time
import random
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QFont, QPen
from PySide6.QtCore import Qt

class CameraSimulatorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(240)
        self.heading = 0
        self.speed = 12.8
        self.fps = 0.0
        self.latency = 0
        self.lat = 39.920782
        self.lon = 32.854115
        self.current_frame = None

    def set_frame(self, q_image):
        self.current_frame = q_image
        if q_image is not None:
            self.fps = 30.0
            self.latency = random.randint(15, 35)
        else:
            self.fps = 0.0
            self.latency = 0
        self.update()

    def set_telemetry(self, lat, lon, heading, speed):
        self.lat = lat
        self.lon = lon
        self.heading = heading
        self.speed = speed

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        
        # 1. Draw camera frame or connection warning
        if self.current_frame is not None:
            scaled_img = self.current_frame.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            x = (width - scaled_img.width()) // 2
            y = (height - scaled_img.height()) // 2
            painter.drawImage(x, y, scaled_img)
            
            # Corner photo boundaries
            p_len = 15
            p_offset = 12
            painter.setPen(QPen(QColor(16, 185, 129, 150), 1)) # Green HUD
            # Top-left
            painter.drawLine(p_offset, p_offset, p_offset + p_len, p_offset)
            painter.drawLine(p_offset, p_offset, p_offset, p_offset + p_len)
            # Top-right
            painter.drawLine(width - p_offset, p_offset, width - p_offset - p_len, p_offset)
            painter.drawLine(width - p_offset, p_offset, width - p_offset, p_offset + p_len)
            # Bottom-left
            painter.drawLine(p_offset, height - p_offset, p_offset + p_len, height - p_offset)
            painter.drawLine(p_offset, height - p_offset, p_offset, height - p_offset - p_len)
            # Bottom-right
            painter.drawLine(width - p_offset, height - p_offset, width - p_offset - p_len, height - p_offset)
            painter.drawLine(width - p_offset, height - p_offset, width - p_offset, height - p_offset - p_len)
            
            # Center crosshair
            painter.setPen(QPen(QColor("#10b981"), 1.5))
            painter.drawLine(width//2 - 10, height//2, width//2 - 3, height//2)
            painter.drawLine(width//2 + 3, height//2, width//2 + 10, height//2)
            painter.drawLine(width//2, height//2 - 10, width//2, height//2 - 3)
            painter.drawLine(width//2, height//2 + 3, width//2, height//2 + 10)
            
            # Horizontal wings
            painter.setPen(QPen(QColor(16, 185, 129, 80), 1))
            painter.drawLine(width//2 - 40, height//2, width//2 - 20, height//2)
            painter.drawLine(width//2 + 20, height//2, width//2 + 40, height//2)
            
            # Text Overlays (FPS, Latency, GPS)
            painter.setPen(QColor("#10b981"))
            painter.setFont(QFont("Roboto Mono", 8))
            painter.drawText(p_offset + 5, p_offset + 20, "CAM-01 [GIMBAL]")
            painter.setPen(QColor("#e2e8f0"))
            painter.drawText(p_offset + 5, p_offset + 35, f"LAT: {self.lat:.6f}")
            painter.drawText(p_offset + 5, p_offset + 47, f"LON: {self.lon:.6f}")
            
            # Right values
            painter.setPen(QColor("#10b981"))
            painter.drawText(width - p_offset - 75, p_offset + 20, f"FPS: {self.fps:.1f}")
            painter.setPen(QColor("#f59e0b"))
            painter.drawText(width - p_offset - 85, p_offset + 35, f"LATENCY: {self.latency}ms")
            
        else:
            # Fallback to connection warning screen
            painter.fillRect(0, 0, width, height, QColor("#090d16"))
            
            # Draw a subtle border
            painter.setPen(QPen(QColor("#1e293b"), 1.5))
            painter.drawRect(0, 0, width - 1, height - 1)
            
            # Draw warning icon/text in the center
            painter.setPen(QColor("#ef4444")) # Red color
            painter.setFont(QFont("Outfit", 12, QFont.Bold))
            text = "BAĞLANTI YOK"
            font_metrics = painter.fontMetrics()
            text_width = font_metrics.horizontalAdvance(text)
            text_height = font_metrics.height()
            painter.drawText((width - text_width) // 2, (height + text_height) // 2 - 10, text)
            
            painter.setPen(QColor("#64748b")) # Gray secondary text
            painter.setFont(QFont("Roboto Mono", 8))
            subtext = "Görüntü akışı bekleniyor..."
            sub_metrics = painter.fontMetrics()
            sub_width = sub_metrics.horizontalAdvance(subtext)
            painter.drawText((width - sub_width) // 2, (height + text_height) // 2 + 15, subtext)
