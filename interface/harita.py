"""Yerli (QPainter) taktiksel takip haritası.

Önceki Leaflet/QtWebEngine haritasının yerine geçer. WebEngine haritası iki
nedenle "çalışmıyor" durumuna düşüyordu:
  * Leaflet + karo (tile) katmanı CDN'den yükleniyordu; ağ/proxy takıldığında
    veya Chromium renderer'ı (64 MB JS heap sınırı, GPU kapalı) çöktüğünde
    panel sessizce boş kalıyordu (stderr /dev/null'a yönlendirildiği için hata
    da görünmüyordu).
  * 8 GB RAM'li sistemde QtWebEngine zaten en büyük RAM/çökme kaynağıydı.

Bu widget hiçbir dış kaynak kullanmaz: lat/lon değerlerini kalkış noktasına
göre yerel metre düzlemine (equirectangular) çevirip rota çizgisi, waypoint'ler,
hedef işaretleri, İHA izi ve İHA sembolünü kendisi çizer. API'si app.py'nin
eski runJavaScript çağrılarıyla birebir eşleşen doğrudan metotlardır.
"""

import math

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QFont, QPen, QBrush, QPolygonF
from PySide6.QtCore import Qt, QPointF, Signal, QTimer

EARTH_R = 6371000.0


def _plane_polygon():
    """Burnu -y'ye (ekranda yukarı = Kuzey) bakan uçak silueti.

    Yarım profil (+x tarafı) tanımlanır, ikinci yarı aynalanır: burun -> gövde ->
    ok kanat -> gövde arkası -> kuyruk düzlemi.
    """
    half = [
        (0.0, -11.0),    # burun
        (1.3, -6.5),
        (1.6, -2.5),     # kanat kök hücum kenarı
        (10.0, 4.0),     # kanat ucu hücum kenarı
        (10.0, 6.0),     # kanat ucu firar kenarı
        (1.8, 3.0),      # kanat kök firar kenarı
        (1.5, 7.5),
        (4.8, 10.5),     # kuyruk düzlemi ucu
        (4.8, 12.0),
        (0.0, 10.0),     # kuyruk konisi (merkez hat)
    ]
    pts = [QPointF(x, y) for x, y in half]
    # Merkez hattaki iki uç (burun ve kuyruk) tekrarlanmasın.
    pts += [QPointF(-x, y) for x, y in reversed(half[1:-1])]
    return QPolygonF(pts)


_UAV_SHAPE = _plane_polygon()


class TacticalMapWidget(QWidget):
    # Eski QWebEngineView arayüzüyle uyum: app.py loadFinished'a bağlanıyor.
    loadFinished = Signal(bool)

    def __init__(self, home_lat, home_lon, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(160)
        self.home_lat = float(home_lat)
        self.home_lon = float(home_lon)
        self._cos_lat = math.cos(math.radians(self.home_lat))

        self.waypoints = []      # {"x","y","name"}
        self.targets = {}        # id -> {"x","y","type","conf","qr_text"}
        self.trail = []          # İHA izi [(x, y)]
        self.uav = None          # {"x","y","heading","mode","alt","speed"}

        # Görünüm: takip modunda İHA ortalanır; hedefe odaklanınca oraya kilitlenir.
        self.follow_uav = True
        self.center_xy = (0.0, 0.0)
        self.scale = 1.0         # px / metre (rota yüklenince hesaplanır)

        # Kurulum sırasını korumak için loadFinished'ı bir sonraki event-loop
        # turunda yayınla (app.py sinyali bağladıktan sonra tetiklensin).
        QTimer.singleShot(0, lambda: self.loadFinished.emit(True))

    # ------------------------------------------------------------------
    # Koordinat dönüşümü
    # ------------------------------------------------------------------
    def _to_xy(self, lat, lon):
        """lat/lon -> kalkış noktasına göre metre (x=Doğu, y=Kuzey)."""
        x = math.radians(lon - self.home_lon) * EARTH_R * self._cos_lat
        y = math.radians(lat - self.home_lat) * EARTH_R
        return x, y

    def _to_screen(self, x, y):
        cx, cy = self.center_xy
        sx = self.width() / 2 + (x - cx) * self.scale
        sy = self.height() / 2 - (y - cy) * self.scale
        return QPointF(sx, sy)

    # ------------------------------------------------------------------
    # app.py'nin çağırdığı API
    # ------------------------------------------------------------------
    def set_waypoints(self, wp_list):
        """[{lat, lon, name}] rota listesini yükler ve ölçeği rotaya sığdırır."""
        self.waypoints = []
        for wp in wp_list:
            x, y = self._to_xy(wp["lat"], wp["lon"])
            self.waypoints.append({"x": x, "y": y, "name": wp.get("name", "")})
        self._fit_route()
        self.update()

    def update_uav(self, lat, lon, heading, mode="", alt=0.0, speed=0.0):
        x, y = self._to_xy(lat, lon)
        self.uav = {"x": x, "y": y, "heading": float(heading),
                    "mode": mode, "alt": float(alt), "speed": float(speed)}
        # İzi yalnızca gerçekten hareket varsa uzat (0.5 m eşiği Leaflet'tekiyle aynı).
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.5:
            self.trail.append((x, y))
            if len(self.trail) > 2000:
                self.trail = self.trail[-1500:]
        if self.follow_uav:
            self.center_xy = (x, y)
        self.update()

    def add_target(self, target_id, target_type, lat, lon, conf, qr_text=None):
        x, y = self._to_xy(lat, lon)
        self.targets[target_id] = {
            "x": x, "y": y, "type": target_type,
            "conf": float(conf), "qr_text": qr_text or "",
        }
        self.update()

    def set_target_qr_text(self, target_id, qr_text):
        if target_id in self.targets:
            self.targets[target_id]["qr_text"] = qr_text
            self.update()

    def center_on(self, lat, lon):
        """Tablodan hedef seçilince oraya kilitlenir; takip modundan çıkar."""
        self.follow_uav = False
        self.center_xy = self._to_xy(lat, lon)
        self.update()

    def center_on_uav(self):
        self.follow_uav = True
        if self.uav:
            self.center_xy = (self.uav["x"], self.uav["y"])
        self.update()

    def clear_path(self):
        self.trail = []
        self.update()

    # Çift tık: takip moduna geri dön.
    def mouseDoubleClickEvent(self, event):
        self.center_on_uav()

    def _fit_route(self):
        """Ölçeği, rotanın uzun ekseni panele sığacak şekilde ayarlar."""
        if not self.waypoints:
            return
        xs = [w["x"] for w in self.waypoints]
        ys = [w["y"] for w in self.waypoints]
        span = max(max(xs) - min(xs), max(ys) - min(ys), 50.0)
        # Panel henüz yerleşmemişse makul varsayılan boyut kullan.
        w = self.width() if self.width() > 50 else 500
        h = self.height() if self.height() > 50 else 260
        self.scale = min(w, h) * 0.85 / span
        if self.uav is None:
            cx = (min(xs) + max(xs)) / 2
            cy = (min(ys) + max(ys)) / 2
            self.center_xy = (cx, cy)

    # ------------------------------------------------------------------
    # Çizim
    # ------------------------------------------------------------------
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Zemin
        p.fillRect(0, 0, w, h, QColor("#030712"))

        # Metrik ızgara (50 m adım; ölçeğe göre 100/200'e büyür)
        step = 50.0
        while step * self.scale < 28:
            step *= 2
        cx, cy = self.center_xy
        p.setPen(QPen(QColor("#0d1522"), 1))
        x0 = math.floor((cx - w / 2 / self.scale) / step) * step
        x1 = cx + w / 2 / self.scale
        gx = x0
        while gx <= x1:
            pt = self._to_screen(gx, cy)
            p.drawLine(int(pt.x()), 0, int(pt.x()), h)
            gx += step
        y0 = math.floor((cy - h / 2 / self.scale) / step) * step
        y1 = cy + h / 2 / self.scale
        gy = y0
        while gy <= y1:
            pt = self._to_screen(cx, gy)
            p.drawLine(0, int(pt.y()), w, int(pt.y()))
            gy += step

        # Planlanan rota (kesikli turuncu)
        if len(self.waypoints) >= 2:
            pen = QPen(QColor(245, 158, 11, 190), 2)
            pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            pts = [self._to_screen(wp["x"], wp["y"]) for wp in self.waypoints]
            for a, b in zip(pts, pts[1:]):
                p.drawLine(a, b)

        # Waypoint elmasları (yeşil)
        p.setFont(QFont("Roboto Mono", 7))
        for wp in self.waypoints:
            pt = self._to_screen(wp["x"], wp["y"])
            p.save()
            p.translate(pt)
            p.rotate(45)
            p.setPen(QPen(QColor("#10b981"), 1))
            p.setBrush(QBrush(QColor(16, 185, 129, 30)))
            p.drawRect(-4, -4, 8, 8)
            p.restore()

        # İHA izi (cyan)
        if len(self.trail) >= 2:
            pen = QPen(QColor(6, 182, 212, 160), 2)
            pen.setStyle(Qt.DotLine)
            p.setPen(pen)
            pts = [self._to_screen(x, y) for x, y in self.trail]
            for a, b in zip(pts, pts[1:]):
                p.drawLine(a, b)

        # Hedefler (turuncu kare + etiket + varsa QR metni)
        p.setFont(QFont("Roboto Mono", 7, QFont.Bold))
        for tid, t in self.targets.items():
            pt = self._to_screen(t["x"], t["y"])
            pen = QPen(QColor("#f59e0b"), 2)
            pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            p.setBrush(QBrush(QColor(245, 158, 11, 40)))
            p.drawRect(int(pt.x()) - 7, int(pt.y()) - 7, 14, 14)
            p.setPen(QColor("#fbbf24"))
            label = f"{tid} {t['type']} %{int(t['conf'] * 100)}"
            p.drawText(int(pt.x()) + 10, int(pt.y()) - 2, label)
            if t.get("qr_text"):
                p.setPen(QColor("#a7f3d0"))
                p.drawText(int(pt.x()) + 10, int(pt.y()) + 9, f"QR: {t['qr_text']}")

        # İHA sembolü (cyan uçak silueti, burnu heading yönünde)
        if self.uav:
            pt = self._to_screen(self.uav["x"], self.uav["y"])
            p.save()
            p.translate(pt)
            # heading pusula açısıdır (0=Kuzey, saat yönü); ekranda Kuzey yukarı
            # olduğu için silueti burnu -y'ye bakacak şekilde çizip aynı açıyla
            # döndürmek yeterli (QPainter.rotate de saat yönünde pozitif).
            p.rotate(self.uav["heading"])
            p.setPen(QPen(QColor("#06b6d4"), 1.2))
            p.setBrush(QBrush(QColor(6, 182, 212, 110)))
            p.drawPolygon(_UAV_SHAPE)
            p.restore()
            # Bilgi kutusu (sol üst)
            p.setFont(QFont("Roboto Mono", 8))
            p.setPen(QColor("#94a3b8"))
            info = (f"MOD: {self.uav['mode']}  ALT: {self.uav['alt']:.1f} m  "
                    f"HIZ: {self.uav['speed']:.1f} m/s  YÖN: {self.uav['heading']:.0f}°")
            p.drawText(8, 14, info)

        # Ölçek çubuğu (sağ alt)
        bar_m = step
        bar_px = int(bar_m * self.scale)
        p.setPen(QPen(QColor("#64748b"), 2))
        p.drawLine(w - 12 - bar_px, h - 12, w - 12, h - 12)
        p.setFont(QFont("Roboto Mono", 7))
        p.drawText(w - 12 - bar_px, h - 16, f"{bar_m:.0f} m")

        # Takip modu rozeti
        if not self.follow_uav:
            p.setPen(QColor("#f59e0b"))
            p.setFont(QFont("Roboto Mono", 7, QFont.Bold))
            p.drawText(8, h - 8, "SABİT GÖRÜNÜM — çift tık: İHA takibine dön")
