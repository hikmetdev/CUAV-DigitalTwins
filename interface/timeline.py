from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QFont, QPen, QBrush
from PySide6.QtCore import Qt

class TimelineWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(45)
        self.events = []

    def set_events(self, events):
        self.events = events
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        
        # Horizontal Axis line
        painter.setPen(QPen(QColor("#1e293b"), 1.5))
        painter.drawLine(20, height // 2 - 2, width - 20, height // 2 - 2)
        
        # Vertical grid ticks
        painter.setPen(QPen(QColor("#1e293b"), 1, Qt.DashLine))
        count = 5
        for i in range(count):
            x = 20 + i * (width - 40) // (count - 1)
            painter.drawLine(x, 4, x, height - 12)
            
        # Draw ticks labels
        painter.setFont(QFont("Roboto Mono", 7))
        painter.setPen(QColor("#475569"))
        painter.drawText(15, height - 2, "-10 dk")
        painter.drawText(width // 2 - 15, height - 2, "-5 dk")
        painter.drawText(width - 45, height - 2, "Şimdi")
        
        # Draw Event nodes
        if not self.events:
            return
            
        num_events = len(self.events)
        for idx, ev in enumerate(self.events):
            # Calculate position on axis
            x = 35 + idx * (width - 70) // max(1, num_events - 1)
            
            # Select Node color
            color = QColor("#10b981") # default command Green
            if ev["type"] == "vehicle":
                color = QColor("#ef4444") # Red
            elif ev["type"] == "person":
                color = QColor("#f59e0b") # Orange
            elif ev["type"] == "waypoint":
                color = QColor("#06b6d4") # Cyan
                
            # Circle Node
            painter.setPen(QPen(QColor("#030712"), 1.5))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(x - 5, height // 2 - 7, 10, 10)
            
            # Event name tag (mini text)
            painter.setPen(color)
            painter.setFont(QFont("Roboto Mono", 6, QFont.Bold))
            painter.drawText(x - 12, height // 2 - 10, ev["time"][:5])
