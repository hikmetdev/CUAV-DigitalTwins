import time
import random
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QFont, QPen
from PySide6.QtCore import Qt, QTimer, QRectF

# Görüntü üstündeki durum katmanında satır başına kullanılan renkler.
STATUS_COLORS = {
    "INFO": "#38bdf8",
    "OK": "#10b981",
    "WARN": "#f59e0b",
    "ERROR": "#ef4444",
}

class CameraSimulatorWidget(QWidget):
    """Canlı görüntü alanı. Kare çizmenin yanında kamera kaynağı geçişlerine ait
    durum mesajlarını (ör. 'Video yükleniyor...') doğrudan görüntünün üzerinde
    gösterir; kullanıcı terminal loguna bakmak zorunda kalmaz."""

    # Kalıcı olmayan mesajların ekranda kalma süresi (saniye).
    STATUS_TTL = 6.0

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
        # Alınan kare sayacı (kaynak geçiş denetimi için).
        self.frame_count = 0
        # Görüntü üstü durum katmanı: [(metin, seviye, zaman, kalıcı_mı), ...]
        self.status_msgs = []
        # Sol üstte gösterilen aktif kaynak etiketi ("GAZEBO" / "VIDEO").
        self.source_label = "GAZEBO"
        # Kaynak geçişi sürerken dönen animasyon için sayaç.
        self.busy = False
        self._spin = 0
        self._anim = QTimer(self)
        self._anim.setInterval(120)
        self._anim.timeout.connect(self._tick)
        self._anim.start()

    # ------------------------------------------------------------------
    # Durum katmanı API'si
    # ------------------------------------------------------------------
    def push_status(self, text, level="INFO", sticky=False):
        """Görüntünün üzerine bir durum satırı ekler. sticky=True ise mesaj
        zaman aşımına uğramaz (yükleme sürerken sabit kalması için)."""
        self.status_msgs.append([str(text), level, time.time(), bool(sticky)])
        if len(self.status_msgs) > 5:
            self.status_msgs.pop(0)
        self.update()

    def clear_status(self):
        self.status_msgs = []
        self.update()

    def set_source_label(self, label, busy=False):
        """Aktif kamera kaynağını (ve geçiş sürüyor mu bilgisini) günceller."""
        self.source_label = label
        self.busy = busy
        self.update()

    def _tick(self):
        """Animasyon/zaman aşımı sayacı: süresi dolan mesajları düşürür."""
        self._spin = (self._spin + 1) % 8
        now = time.time()
        alive = [m for m in self.status_msgs if m[3] or (now - m[2]) < self.STATUS_TTL]
        changed = len(alive) != len(self.status_msgs)
        self.status_msgs = alive
        if changed or self.busy or self.status_msgs:
            self.update()

    def _draw_status_overlay(self, painter, width, height, on_frame):
        """Durum satırlarını görüntünün üzerine yarı saydam bir kutu içinde çizer."""
        if not self.status_msgs and not self.busy:
            return

        painter.setFont(QFont("Roboto Mono", 8))
        fm = painter.fontMetrics()
        lines = [(m[0], m[1]) for m in self.status_msgs]
        if self.busy:
            dots = "." * (1 + self._spin % 3)
            lines.append((f"Kamera kaynağı değiştiriliyor{dots}", "WARN"))

        line_h = fm.height() + 3
        box_w = min(width - 24, max(fm.horizontalAdvance(t) for t, _ in lines) + 22)
        box_h = line_h * len(lines) + 12
        x = 12
        y = height - box_h - 12 if on_frame else (height + 40) // 2

        painter.setBrush(QColor(4, 8, 16, 205))
        painter.setPen(QPen(QColor(56, 189, 248, 110), 1))
        painter.drawRoundedRect(QRectF(x, y, box_w, box_h), 4, 4)

        ty = y + fm.ascent() + 7
        for text, level in lines:
            painter.setPen(QColor(STATUS_COLORS.get(level, "#94a3b8")))
            painter.drawText(x + 9, ty, "▸")
            painter.setPen(QColor("#e2e8f0" if level == "INFO" else STATUS_COLORS.get(level, "#e2e8f0")))
            painter.drawText(x + 22, ty, fm.elidedText(text, Qt.ElideRight, box_w - 30))
            ty += line_h

    def set_frame(self, q_image):
        self.current_frame = q_image
        # Kaynak geçişi sonrası "yayın geliyor mu?" denetimi için sayaç.
        self.frame_count += 1
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
            painter.drawText(p_offset + 5, p_offset + 20, f"CAM-01 [{self.source_label}]")
            painter.setPen(QColor("#e2e8f0"))
            painter.drawText(p_offset + 5, p_offset + 35, f"LAT: {self.lat:.6f}")
            painter.drawText(p_offset + 5, p_offset + 47, f"LON: {self.lon:.6f}")
            
            # Right values
            painter.setPen(QColor("#10b981"))
            painter.drawText(width - p_offset - 75, p_offset + 20, f"FPS: {self.fps:.1f}")
            painter.setPen(QColor("#f59e0b"))
            painter.drawText(width - p_offset - 85, p_offset + 35, f"LATENCY: {self.latency}ms")

            self._draw_status_overlay(painter, width, height, on_frame=True)

        else:
            # Fallback to connection warning screen
            painter.fillRect(0, 0, width, height, QColor("#090d16"))
            
            # Draw a subtle border
            painter.setPen(QPen(QColor("#1e293b"), 1.5))
            painter.drawRect(0, 0, width - 1, height - 1)
            
            # Kaynak geçişi sürüyorsa "bağlantı yok" değil, yükleme ekranı göster.
            if self.busy:
                painter.setPen(QColor("#f59e0b"))
                text = f"{self.source_label} KAMERASI HAZIRLANIYOR"
                subtext = "Model ve görüntü akışı yükleniyor" + "." * (1 + self._spin % 3)
            else:
                painter.setPen(QColor("#ef4444")) # Red color
                text = "BAĞLANTI YOK"
                subtext = "Görüntü akışı bekleniyor..."
            painter.setFont(QFont("Outfit", 12, QFont.Bold))
            font_metrics = painter.fontMetrics()
            text_width = font_metrics.horizontalAdvance(text)
            text_height = font_metrics.height()
            painter.drawText((width - text_width) // 2, (height + text_height) // 2 - 40, text)

            painter.setPen(QColor("#64748b")) # Gray secondary text
            painter.setFont(QFont("Roboto Mono", 8))
            sub_metrics = painter.fontMetrics()
            sub_width = sub_metrics.horizontalAdvance(subtext)
            painter.drawText((width - sub_width) // 2, (height + text_height) // 2 - 15, subtext)

            self._draw_status_overlay(painter, width, height, on_frame=False)
