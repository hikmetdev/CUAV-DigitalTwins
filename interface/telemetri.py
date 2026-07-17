from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QFont, QPen, QBrush, QPolygonF
from PySide6.QtCore import Qt, QPointF

class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(60, 60)
        self.setMaximumSize(60, 60)
        self.heading = 0 # Degrees

    def set_heading(self, heading):
        self.heading = heading
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        radius = min(width, height) / 2 - 2
        
        # Center coordinates
        painter.translate(width / 2, height / 2)
        
        # Draw outer circle
        painter.setPen(QPen(QColor("#1e293b"), 1.5))
        painter.setBrush(QBrush(QColor("#030712")))
        painter.drawEllipse(-radius, -radius, radius * 2, radius * 2)
        
        # Rotate painter for heading needle
        painter.save()
        painter.rotate(self.heading)
        
        # Draw arrow / needle
        painter.setPen(Qt.NoPen)
        # North pointer (Cyan)
        painter.setBrush(QBrush(QColor("#06b6d4")))
        north_poly = QPolygonF([
            QPointF(0, -radius + 4),
            QPointF(5, 0),
            QPointF(-5, 0)
        ])
        painter.drawPolygon(north_poly)
        
        # South pointer (Red/Gray)
        painter.setBrush(QBrush(QColor("#334155")))
        south_poly = QPolygonF([
            QPointF(0, radius - 4),
            QPointF(5, 0),
            QPointF(-5, 0)
        ])
        painter.drawPolygon(south_poly)
        
        painter.restore()
        
        # Draw Cardinal labels (N, E, S, W) static on top
        painter.setPen(QColor("#10b981"))
        painter.setFont(QFont("Roboto Mono", 7, QFont.Bold))
        painter.drawText(-4, -radius + 12, "N")
        painter.setPen(QColor("#94a3b8"))
        painter.drawText(radius - 12, 3, "E")
        painter.drawText(-3, radius - 6, "S")
        painter.drawText(-radius + 6, 3, "W")
