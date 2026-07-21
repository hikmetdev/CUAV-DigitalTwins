from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QFont, QPen, QBrush, QPolygonF
from PySide6.QtCore import Qt, QPointF, QRect

class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(45, 45)
        self.setMaximumSize(45, 45)
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
            QPointF(4, 0),
            QPointF(-4, 0)
        ])
        painter.drawPolygon(north_poly)
        
        # South pointer (Red/Gray)
        painter.setBrush(QBrush(QColor("#334155")))
        south_poly = QPolygonF([
            QPointF(0, radius - 4),
            QPointF(4, 0),
            QPointF(-4, 0)
        ])
        painter.drawPolygon(south_poly)
        
        painter.restore()
        
        # Draw Cardinal labels (N, E, S, W) static on top
        painter.setPen(QColor("#10b981"))
        painter.setFont(QFont("Roboto Mono", 6, QFont.Bold))
        painter.drawText(-3, -radius + 10, "N")
        painter.setPen(QColor("#94a3b8"))
        painter.drawText(radius - 10, 3, "E")
        painter.drawText(-3, radius - 4, "S")
        painter.drawText(-radius + 4, 3, "W")


class TelemetryChartsPanel(QWidget):
    """SİHA 2x2 Canlı Telemetri Zaman Serisi Grafik Paneli.
    Grafik 1: İrtifa (m) - Mavi Çizgi (#38bdf8)
    Grafik 2: Hava Hızı (m/s) - Yeşil Çizgi (#10b981)
    Grafik 3: Batarya (% / V) - Turuncu Çizgi (#f59e0b)
    Grafik 4: Kat Edilen Mesafe (m) - Mor Çizgi (#a855f7)
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(200)
        self.history = []

    def set_history(self, history):
        self.history = list(history)[-60:] # Son 60 saniyelik veri
        self.update()

    def _draw_value_badge(self, painter, x_right, y_top, text, color):
        """Güncel değeri, çizginin/ızgaranın üstüne binip okunmaz hale gelmesin
        diye koyu zeminli bir rozet içinde yazar: dolu koyu arka plan + seri
        renginde ince kenarlık + yüksek kontrastlı açık gri metin.
        (x_right, y_top) rozetin SAĞ-ÜST köşesidir."""
        painter.setFont(QFont("Roboto Mono", 8, QFont.Bold))
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        th = fm.height()
        pad_x, pad_y = 5, 2
        bw = tw + pad_x * 2
        bh = th + pad_y * 2
        bx = int(x_right - bw)
        by = int(y_top)

        painter.setBrush(QBrush(QColor(3, 7, 18, 235)))   # #030712, neredeyse opak
        painter.setPen(QPen(QColor(color), 1))
        painter.drawRoundedRect(bx, by, bw, bh, 3, 3)

        painter.setBrush(Qt.NoBrush)
        painter.setPen(QColor("#f8fafc"))                  # kırık beyaz metin
        painter.drawText(bx + pad_x, by + pad_y + fm.ascent(), text)

    def _draw_sub_chart(self, painter, rect, title, values_or_series, color_spec, unit, default_max=10.0, min_y=0.0):
        x0, y0, w, h = rect.x(), rect.y(), rect.width(), rect.height()

        # Alt grafik arka plan ve kenarlık (Dark Navy #0d1522)
        painter.fillRect(x0, y0, w, h, QColor("#0d1522"))
        painter.setPen(QPen(QColor("rgba(16, 185, 129, 0.2)"), 1)) # Emerald green kenarlık
        painter.drawRoundedRect(x0, y0, w - 1, h - 1, 4, 4)

        is_multi = isinstance(color_spec, list)
        if is_multi:
            main_color = color_spec[0][2] if isinstance(color_spec[0], (tuple, list)) else color_spec[0]
        else:
            main_color = color_spec

        # Başlık
        painter.setFont(QFont("Roboto Mono", 8, QFont.Bold))
        painter.setPen(QColor(main_color))
        painter.drawText(x0 + 8, y0 + 16, title)

        if len(self.history) < 2:
            painter.setPen(QColor("#64748b"))
            painter.setFont(QFont("Roboto Mono", 7))
            painter.drawText(x0 + max(10, w // 2 - 35), y0 + h // 2, "Veri bekleniyor...")
            return

        # Izgara Çizgileri
        painter.setPen(QPen(QColor(255, 255, 255, 12), 1, Qt.DashLine))
        painter.drawLine(x0, y0 + h // 2, x0 + w, y0 + h // 2)

        n = len(self.history)
        dx = (w - 16) / max(1, n - 1)

        if not is_multi:
            values = values_or_series
            max_val = max(max(values), default_max)
            min_val = min(min_y, min(values))
            val_range = max(1.0, max_val - min_val)

            # Çizgi çizimi
            painter.setPen(QPen(QColor(main_color), 1.8))
            for i in range(n - 1):
                px1 = x0 + 8 + i * dx
                py1 = y0 + h - 6 - ((values[i] - min_val) / val_range) * (h - 26)
                px2 = x0 + 8 + (i + 1) * dx
                py2 = y0 + h - 6 - ((values[i + 1] - min_val) / val_range) * (h - 26)
                painter.drawLine(int(px1), int(py1), int(px2), int(py2))

            # Güncel değer rozeti EN SON çizilir ki veri çizgisi üstünden geçip
            # rakamları bölmesin (rozet zemini opak).
            last_val = values[-1]
            val_str = f"{last_val:.1f}{unit}" if isinstance(last_val, float) else f"{last_val}{unit}"
            self._draw_value_badge(painter, x0 + w - 6, y0 + 4, val_str, main_color)
        else:
            # Çok renkli seri
            all_vals = []
            for key, lbl, col in color_spec:
                vals = [float(s.get(key, 0.0)) for s in self.history]
                all_vals.extend(vals)

            max_val = max(max(all_vals) if all_vals else default_max, default_max)
            min_val = min(min(all_vals) if all_vals else min_y, min_y)
            val_range = max(1.0, max_val - min_val)

            for key, lbl, col in color_spec:
                vals = [float(s.get(key, 0.0)) for s in self.history]
                painter.setPen(QPen(QColor(col), 1.5))
                for i in range(n - 1):
                    px1 = x0 + 8 + i * dx
                    py1 = y0 + h - 6 - ((vals[i] - min_val) / val_range) * (h - 26)
                    px2 = x0 + 8 + (i + 1) * dx
                    py2 = y0 + h - 6 - ((vals[i + 1] - min_val) / val_range) * (h - 26)
                    painter.drawLine(int(px1), int(py1), int(px2), int(py2))

            # Güncel değer rozeti çizgilerin üstüne (opak zemin, okunur rakam).
            last_val = float(self.history[-1].get(color_spec[0][0], 0.0))
            self._draw_value_badge(painter, x0 + w - 6, y0 + 4, f"{last_val:.0f}{unit}", color_spec[0][2])

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Dark Navy arka plan (#030712)
        painter.fillRect(0, 0, w, h, QColor("#030712"))

        margin = 4
        sub_w = (w - margin * 3) // 2
        sub_h = (h - margin * 3) // 2

        r1 = QRect(margin, margin, sub_w, sub_h)
        r2 = QRect(margin * 2 + sub_w, margin, sub_w, sub_h)
        r3 = QRect(margin, margin * 2 + sub_h, sub_w, sub_h)
        r4 = QRect(margin * 2 + sub_w, margin * 2 + sub_h, sub_w, sub_h)

        alts = [float(s.get("alt", 0.0)) for s in self.history]
        speeds = [float(s.get("speed", 0.0)) for s in self.history]
        bats = [float(s.get("battery", 100.0)) for s in self.history]
        dists = [float(s.get("dist", 0.0)) for s in self.history]

        # Grafik 1: Zamana bağlı İrtifa (m) - Mavi çizgi (#38bdf8)
        self._draw_sub_chart(painter, r1, "İRTİFA (m)", alts, "#38bdf8", "m", default_max=20.0, min_y=0.0)

        # Grafik 2: Zamana bağlı Hava Hızı (m/s) - Yeşil çizgi (#10b981)
        self._draw_sub_chart(painter, r2, "HAVA HIZI (m/s)", speeds, "#10b981", "m/s", default_max=15.0, min_y=0.0)

        # Grafik 3: Zamana bağlı Batarya Seviyesi (% / V) - Turuncu çizgi (#f59e0b)
        self._draw_sub_chart(painter, r3, "BATARYA (%/V)", bats, "#f59e0b", "%", default_max=100.0, min_y=0.0)

        # Grafik 4: Zamana bağlı Kat Edilen Mesafe (m) - Mor çizgi (#a855f7)
        self._draw_sub_chart(painter, r4, "KAT EDİLEN MESAFE (m)", dists, "#a855f7", "m", default_max=50.0, min_y=0.0)
