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
import time

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import (QPainter, QColor, QFont, QPen, QBrush, QPolygonF,
                           QRadialGradient, QLinearGradient)
from PySide6.QtCore import Qt, QPointF, QRectF, Signal, QTimer

EARTH_R = 6371000.0

# Hedef tipine göre renk + kısa kod (harita sembolünün içine yazılır).
TARGET_STYLE = {
    "Kırmızı Kutu":  ("#ef4444", "KK"),
    "Mavi Kutu":     ("#3b82f6", "MK"),
    "Kutu":          ("#f59e0b", "KT"),
    "Araç":          ("#a78bfa", "AR"),
    "Kişi":          ("#f472b6", "KŞ"),
    "Sırt Çantası":  ("#fb923c", "SÇ"),
    "Stop Tabelası": ("#f87171", "ST"),
    "Dama Paneli":   ("#e2e8f0", "DP"),
    "Nişan Tahtası": ("#22d3ee", "NT"),
}
DEFAULT_TARGET_STYLE = ("#f59e0b", "??")


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
        self.fit_scale = 1.0     # rotaya sığdırılmış temel ölçek (zoom referansı)
        self.zoom = 1.0          # kullanıcı yakınlaştırması (tekerlek)
        self._drag_origin = None # sürükleyerek kaydırma için son fare noktası

        # Nabız/tarama animasyonu (hedef halkaları, İHA halesi). 10 FPS yeterli.
        self._t0 = time.time()
        self._pulse = QTimer(self)
        self._pulse.setInterval(100)
        self._pulse.timeout.connect(self._pulse_tick)
        self._pulse.start()
        self.setMouseTracking(False)

        # Kurulum sırasını korumak için loadFinished'ı bir sonraki event-loop
        # turunda yayınla (app.py sinyali bağladıktan sonra tetiklensin).
        QTimer.singleShot(0, lambda: self.loadFinished.emit(True))

    def _pulse_tick(self):
        """Nabız animasyonu yalnızca gerekliyken yeniden çizer: panel görünür
        değilse veya ekranda hedef yoksa boşuna CPU harcamaz (8 GB/GTX1650
        sisteminde Gazebo ile kaynak paylaşılıyor)."""
        if self.targets and self.isVisible():
            self.update()

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

    # Çift tık: takip moduna geri dön (ve yakınlaştırmayı sıfırla).
    def mouseDoubleClickEvent(self, event):
        self.zoom = 1.0
        self.scale = self.fit_scale
        self.center_on_uav()

    # --- Fare ile kaydırma / tekerlekle yakınlaştırma --------------------
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag_origin = event.position()

    def mouseMoveEvent(self, event):
        if self._drag_origin is None:
            return
        delta = event.position() - self._drag_origin
        self._drag_origin = event.position()
        cx, cy = self.center_xy
        # Ekranda sağa sürüklemek haritayı sağa kaydırır → merkez sola gider.
        self.center_xy = (cx - delta.x() / self.scale, cy + delta.y() / self.scale)
        self.follow_uav = False
        self.update()

    def mouseReleaseEvent(self, event):
        self._drag_origin = None

    def wheelEvent(self, event):
        """Fare tekerleği: 0.4x - 12x arası yakınlaştırma (imleç konumu sabit kalır)."""
        steps = event.angleDelta().y() / 120.0
        if not steps:
            return
        old_scale = self.scale
        self.zoom = max(0.4, min(12.0, self.zoom * (1.15 ** steps)))
        self.scale = self.fit_scale * self.zoom
        # İmlecin gösterdiği metre noktası ekranda aynı yerde kalsın.
        pos = event.position()
        cx, cy = self.center_xy
        dx = (pos.x() - self.width() / 2)
        dy = (self.height() / 2 - pos.y())
        self.center_xy = (cx + dx * (1 / old_scale - 1 / self.scale),
                          cy + dy * (1 / old_scale - 1 / self.scale))
        self.follow_uav = False
        self.update()

    def _fit_route(self):
        """Temel ölçeği, rotanın uzun ekseni panele sığacak şekilde ayarlar."""
        if not self.waypoints:
            return
        xs = [w["x"] for w in self.waypoints]
        ys = [w["y"] for w in self.waypoints]
        span = max(max(xs) - min(xs), max(ys) - min(ys), 50.0)
        # Panel henüz yerleşmemişse makul varsayılan boyut kullan.
        w = self.width() if self.width() > 50 else 500
        h = self.height() if self.height() > 50 else 260
        self.fit_scale = min(w, h) * 0.85 / span
        self.scale = self.fit_scale * self.zoom
        if self.uav is None:
            cx = (min(xs) + max(xs)) / 2
            cy = (min(ys) + max(ys)) / 2
            self.center_xy = (cx, cy)

    # ------------------------------------------------------------------
    # Çizim yardımcıları
    # ------------------------------------------------------------------
    @staticmethod
    def _chip(p, x, y, text, fg, bg=(3, 7, 18, 215), pad=4):
        """Koyu zeminli küçük etiket kutusu: metin ızgara/rota üstünde okunur kalır.
        (x, y) kutunun sol-üst köşesidir; çizilen dikdörtgen geri döner."""
        fm = p.fontMetrics()
        rect = QRectF(x, y, fm.horizontalAdvance(text) + pad * 2, fm.height() + 2)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(*bg))
        p.drawRoundedRect(rect, 3, 3)
        p.setPen(QColor(fg))
        p.drawText(rect.adjusted(pad, 0, -pad, 0), Qt.AlignVCenter | Qt.AlignLeft, text)
        return rect

    def _draw_background(self, p, w, h):
        """Radyal degrade zemin + köşe karartması: düz siyah yerine derinlikli."""
        g = QRadialGradient(w / 2, h / 2, max(w, h) * 0.75)
        g.setColorAt(0.0, QColor("#0b1626"))
        g.setColorAt(0.55, QColor("#070e1a"))
        g.setColorAt(1.0, QColor("#02050b"))
        p.fillRect(0, 0, w, h, QBrush(g))

    def _draw_grid(self, p, w, h):
        """İnce/kalın metrik ızgara + kenarlarda metre etiketleri. Adımı döndürür."""
        step = 50.0
        while step * self.scale < 26:
            step *= 2
        while step * self.scale > 160:
            step /= 2
        cx, cy = self.center_xy
        major = step * 5

        p.setFont(QFont("Roboto Mono", 6))
        x0 = math.floor((cx - w / 2 / self.scale) / step) * step
        x1 = cx + w / 2 / self.scale
        gx = x0
        while gx <= x1:
            is_major = abs(gx % major) < step / 2 or abs(abs(gx % major) - major) < step / 2
            sx = int(self._to_screen(gx, cy).x())
            p.setPen(QPen(QColor("#16233a") if is_major else QColor("#0d1522"), 1))
            p.drawLine(sx, 0, sx, h)
            if is_major and abs(gx) > 1:
                p.setPen(QColor("#33507a"))
                p.drawText(sx + 3, 10, f"{gx:+.0f}m")
            gx += step

        y0 = math.floor((cy - h / 2 / self.scale) / step) * step
        y1 = cy + h / 2 / self.scale
        gy = y0
        while gy <= y1:
            is_major = abs(gy % major) < step / 2 or abs(abs(gy % major) - major) < step / 2
            sy = int(self._to_screen(cx, gy).y())
            p.setPen(QPen(QColor("#16233a") if is_major else QColor("#0d1522"), 1))
            p.drawLine(0, sy, w, sy)
            if is_major and abs(gy) > 1:
                p.setPen(QColor("#33507a"))
                p.drawText(4, sy - 3, f"{gy:+.0f}m")
            gy += step

        # Kalkış noktasından geçen eksenler (0 m çizgileri) belirgin olsun.
        p.setPen(QPen(QColor(56, 189, 248, 55), 1, Qt.DashLine))
        origin = self._to_screen(0.0, 0.0)
        p.drawLine(int(origin.x()), 0, int(origin.x()), h)
        p.drawLine(0, int(origin.y()), w, int(origin.y()))
        return step

    def _draw_route(self, p):
        """Planlanan rota: parlama + kesikli çizgi + yön okları + kalkış noktası.
        Rota üzerindeki cisim konumları BİLEREK çizilmez (bkz. aşağıdaki not)."""
        if len(self.waypoints) < 2:
            return
        pts = [self._to_screen(wp["x"], wp["y"]) for wp in self.waypoints]

        # Alt katman parlama (glow) — rota kalın turuncu bir koridor gibi görünür.
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(245, 158, 11, 38), 8, Qt.SolidLine, Qt.RoundCap))
        for a, b in zip(pts, pts[1:]):
            p.drawLine(a, b)
        pen = QPen(QColor(251, 191, 36, 220), 1.8)
        pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        for a, b in zip(pts, pts[1:]):
            p.drawLine(a, b)

        # Bacak ortasında uçuş yönü oku.
        p.setFont(QFont("Roboto Mono", 6))
        for a, b in zip(pts, pts[1:]):
            dx, dy = b.x() - a.x(), b.y() - a.y()
            seg = math.hypot(dx, dy)
            if seg < 24:
                continue
            ux, uy = dx / seg, dy / seg
            mx, my = (a.x() + b.x()) / 2, (a.y() + b.y()) / 2
            head = QPolygonF([
                QPointF(mx + ux * 6, my + uy * 6),
                QPointF(mx - ux * 4 + uy * 3.5, my - uy * 4 - ux * 3.5),
                QPointF(mx - ux * 4 - uy * 3.5, my - uy * 4 + ux * 3.5),
            ])
            p.setPen(Qt.NoPen)
            p.setBrush(QColor("#fbbf24"))
            p.drawPolygon(head)

        # Kalkış noktası (rotanın ilk noktası). Rotadaki DİĞER noktalar bilinen
        # cisim konumlarıdır; harita bunları önceden GÖSTERMEZ — cisimler ancak
        # YOLO tespit ettikçe add_target ile haritaya düşer.
        pt = pts[0]
        color = QColor("#22c55e")
        p.setPen(QPen(color, 1.6))
        p.setBrush(QColor(2, 24, 18, 200))
        p.drawEllipse(pt, 8, 8)
        p.setPen(Qt.NoPen)
        p.setBrush(color)
        p.drawEllipse(pt, 3.0, 3.0)
        p.setPen(QPen(QColor(34, 197, 94, 90), 1))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(pt, 13, 13)
        p.setFont(QFont("Roboto Mono", 6))
        tw = p.fontMetrics().horizontalAdvance("KALKIŞ") + 8
        self._chip(p, pt.x() - 11 - tw, pt.y() - 5, "KALKIŞ", "#34d399", bg=(3, 7, 18, 170))

    def _draw_trail(self, p):
        """İHA izi: eskiden yeniye doğru koyulaşan/kalınlaşan çizgi."""
        if len(self.trail) < 2:
            return
        pts = [self._to_screen(x, y) for x, y in self.trail]
        n = len(pts) - 1
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            f = (i + 1) / n            # 0 = en eski, 1 = en yeni
            alpha = int(25 + 200 * f ** 1.6)
            p.setPen(QPen(QColor(6, 182, 212, alpha), 1.0 + 1.8 * f, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(a, b)

    def _place_label(self, pt, bw, bh, placed):
        """Etiket bloğu için cismin çevresinde çakışmayan ilk konumu bulur.
        Adaylar yakından uzağa denenir; hiçbiri boş değilse en az çakışan seçilir.
        Böylece yan yana düşen cisimlerin etiketleri üst üste binmez."""
        cands = []
        for dy in (-14, 6, -34, 26, -54, 46):
            cands.append((pt.x() + 13, pt.y() + dy))          # sağ
            cands.append((pt.x() - 13 - bw, pt.y() + dy))     # sol
        best, best_overlap = None, None
        for x, y in cands:
            # Panel dışına taşmasın.
            x = max(2.0, min(self.width() - bw - 2.0, x))
            y = max(2.0, min(self.height() - bh - 2.0, y))
            rect = QRectF(x, y, bw, bh)
            overlap = sum(rect.intersected(r).width() * rect.intersected(r).height()
                          for r in placed if rect.intersects(r))
            if overlap == 0:
                return x, y
            if best_overlap is None or overlap < best_overlap:
                best, best_overlap = (x, y), overlap
        return best

    def _draw_offscreen_marker(self, p, sp, t):
        """Görüş alanı dışında kalan tespit için panel kenarında yön üçgeni."""
        color = QColor(TARGET_STYLE.get(t["type"], DEFAULT_TARGET_STYLE)[0])
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        dx, dy = sp.x() - cx, sp.y() - cy
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return
        # Merkezden hedefe giden ışının panel kenarını kestiği nokta.
        m = 9.0
        scale = min((cx - m) / abs(dx) if dx else 1e9, (cy - m) / abs(dy) if dy else 1e9)
        ex, ey = cx + dx * scale, cy + dy * scale
        ang = math.atan2(dy, dx)
        p.save()
        p.translate(ex, ey)
        p.rotate(math.degrees(ang))
        color.setAlpha(200)
        p.setPen(Qt.NoPen)
        p.setBrush(color)
        p.drawPolygon(QPolygonF([QPointF(6, 0), QPointF(-4, -4.5), QPointF(-4, 4.5)]))
        p.restore()

    def _draw_targets(self, p):
        """Tespit edilen cisimler: tipe göre renkli, nabız halkalı, etiketli.
        Etiketler birbiriyle ve sembollerle çakışmayacak şekilde yerleştirilir."""
        phase = (time.time() - self._t0)
        # Sembollerin kapladığı alanlar önce toplanır ki etiketler onların da
        # üstüne düşmesin; sıralama ekran y'sine göre (kararlı görünüm).
        items = sorted(self.targets.items(),
                       key=lambda kv: self._to_screen(kv[1]["x"], kv[1]["y"]).y())
        placed = []
        visible = []
        for tid, t in items:
            sp = self._to_screen(t["x"], t["y"])
            if -10 <= sp.x() <= self.width() + 10 and -10 <= sp.y() <= self.height() + 10:
                visible.append((tid, t))
                placed.append(QRectF(sp.x() - 12, sp.y() - 12, 24, 24))
            else:
                # Görüş alanı dışındaki tespit: etiketi panele sıkıştırıp
                # yığmak yerine kenarda küçük bir yön üçgeniyle işaretlenir.
                self._draw_offscreen_marker(p, sp, t)
        items = visible
        if self.uav:
            up = self._to_screen(self.uav["x"], self.uav["y"])
            placed.append(QRectF(up.x() - 16, up.y() - 16, 32, 32))

        for tid, t in items:
            pt = self._to_screen(t["x"], t["y"])
            color_hex, code = TARGET_STYLE.get(t["type"], DEFAULT_TARGET_STYLE)
            color = QColor(color_hex)

            # Nabız halkası: yeni bakılan hedef gözden kaçmasın.
            pr = 10 + 7 * (0.5 + 0.5 * math.sin(phase * 2.2))
            ring = QColor(color)
            ring.setAlpha(int(110 * (1.0 - (pr - 10) / 14)))
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(ring, 1.4))
            p.drawEllipse(pt, pr, pr)

            # Köşe parantezleri (nişan çerçevesi).
            p.setPen(QPen(color, 1.6))
            s, g = 9, 4
            for sx in (-1, 1):
                for sy in (-1, 1):
                    p.drawLine(QPointF(pt.x() + sx * s, pt.y() + sy * s),
                               QPointF(pt.x() + sx * (s - g), pt.y() + sy * s))
                    p.drawLine(QPointF(pt.x() + sx * s, pt.y() + sy * s),
                               QPointF(pt.x() + sx * s, pt.y() + sy * (s - g)))

            # Merkez sembol: tipin kısa kodunu taşıyan dolu daire.
            fill = QColor(color)
            fill.setAlpha(70)
            p.setBrush(fill)
            p.setPen(QPen(color, 1.2))
            p.drawEllipse(pt, 6.5, 6.5)
            p.setFont(QFont("Roboto Mono", 5, QFont.Bold))
            p.setPen(QColor("#f8fafc"))
            p.drawText(QRectF(pt.x() - 7, pt.y() - 7, 14, 14), Qt.AlignCenter, code)

            # --- Etiket bloğu: ID + tip + güven (+ varsa QR) ---
            # Etiketler cismin yanına SABİT konumda basılırsa yakın cisimlerde
            # üst üste biniyordu. Blok, çakışmayan ilk aday konuma yerleştirilir
            # ve cisme ince bir bağlantı çizgisiyle bağlanır.
            head_font = QFont("Roboto Mono", 6, QFont.Bold)
            qr_font = QFont("Roboto Mono", 6)
            head_txt = f"{tid} · {t['type']}  %{int(t['conf'] * 100)}"
            qr_txt = f"QR: {t['qr_text']}" if t.get("qr_text") else None

            p.setFont(head_font)
            fm = p.fontMetrics()
            bw = fm.horizontalAdvance(head_txt) + 8
            bh = fm.height() + 2 + 4          # başlık + güven çubuğu payı
            if qr_txt:
                p.setFont(qr_font)
                bw = max(bw, p.fontMetrics().horizontalAdvance(qr_txt) + 8)
                bh += p.fontMetrics().height() + 5

            bx, by = self._place_label(pt, bw, bh, placed)
            placed.append(QRectF(bx - 2, by - 2, bw + 4, bh + 4))

            # Bağlantı çizgisi (etiket cisimden uzağa düştüyse).
            anchor_x = bx if bx > pt.x() else bx + bw
            anchor_y = by + 6
            if math.hypot(anchor_x - pt.x(), anchor_y - pt.y()) > 16:
                lead = QColor(color)
                lead.setAlpha(120)
                p.setPen(QPen(lead, 1, Qt.DotLine))
                p.setBrush(Qt.NoBrush)
                p.drawLine(pt, QPointF(anchor_x, anchor_y))

            p.setFont(head_font)
            r1 = self._chip(p, bx, by, head_txt, color_hex)
            # Güven çubuğu (etiketin altında kısa şerit).
            bar_w = min(40.0, r1.width())
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(30, 41, 59, 200))
            p.drawRect(QRectF(r1.left(), r1.bottom() + 1, bar_w, 2.5))
            p.setBrush(color)
            p.drawRect(QRectF(r1.left(), r1.bottom() + 1, bar_w * max(0.03, min(1.0, t["conf"])), 2.5))
            if qr_txt:
                p.setFont(qr_font)
                self._chip(p, bx, r1.bottom() + 5, qr_txt, "#a7f3d0")

    def _draw_uav(self, p):
        """İHA sembolü: hale + gövde + yön ışını + hız vektörü."""
        if not self.uav:
            return
        pt = self._to_screen(self.uav["x"], self.uav["y"])
        # Hale (radyal degrade) — koyu zeminde konumu hemen yakalanır.
        halo = QRadialGradient(pt, 30)
        halo.setColorAt(0.0, QColor(6, 182, 212, 90))
        halo.setColorAt(1.0, QColor(6, 182, 212, 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(halo))
        p.drawEllipse(pt, 30, 30)

        hdg = math.radians(self.uav["heading"])
        ux, uy = math.sin(hdg), -math.cos(hdg)   # ekran koordinatında yön birimi

        # Yön ışını (kesikli) — nereye gittiği çizgi olarak görünür.
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(6, 182, 212, 120), 1, Qt.DashLine))
        p.drawLine(pt, QPointF(pt.x() + ux * 55, pt.y() + uy * 55))

        # Hız vektörü (hıza orantılı dolu çizgi, en fazla 45 px).
        vlen = min(45.0, self.uav["speed"] * 2.0)
        if vlen > 4:
            p.setPen(QPen(QColor("#22d3ee"), 2))
            p.drawLine(pt, QPointF(pt.x() + ux * vlen, pt.y() + uy * vlen))

        p.save()
        p.translate(pt)
        # heading pusula açısıdır (0=Kuzey, saat yönü); ekranda Kuzey yukarı
        # olduğu için silueti burnu -y'ye bakacak şekilde çizip aynı açıyla
        # döndürmek yeterli (QPainter.rotate de saat yönünde pozitif).
        p.rotate(self.uav["heading"])
        body = QLinearGradient(0, -12, 0, 12)
        body.setColorAt(0.0, QColor(103, 232, 249, 235))
        body.setColorAt(1.0, QColor(6, 148, 182, 190))
        p.setPen(QPen(QColor("#e0f2fe"), 1.1))
        p.setBrush(QBrush(body))
        p.drawPolygon(_UAV_SHAPE)
        p.restore()

    def _draw_compass(self, p, w, h):
        """Sağ üstte pusula gülü (K/D/G/B) — kuzey daima yukarıdadır."""
        cx, cy, r = w - 30, 30, 15
        c = QPointF(cx, cy)
        p.setPen(QPen(QColor(56, 189, 248, 70), 1))
        p.setBrush(QColor(3, 7, 18, 170))
        p.drawEllipse(c, r, r)
        p.setPen(QPen(QColor(148, 163, 184, 120), 1))
        for ang in range(0, 360, 45):
            a = math.radians(ang)
            p.drawLine(QPointF(cx + math.sin(a) * (r - 4), cy - math.cos(a) * (r - 4)),
                       QPointF(cx + math.sin(a) * r, cy - math.cos(a) * r))
        # Kuzey oku
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#ef4444"))
        p.drawPolygon(QPolygonF([QPointF(cx, cy - r + 2), QPointF(cx - 3.5, cy + 2), QPointF(cx + 3.5, cy + 2)]))
        p.setBrush(QColor("#64748b"))
        p.drawPolygon(QPolygonF([QPointF(cx, cy + r - 2), QPointF(cx - 3.5, cy + 2), QPointF(cx + 3.5, cy + 2)]))
        p.setPen(QColor("#e2e8f0"))
        p.setFont(QFont("Roboto Mono", 6, QFont.Bold))
        p.drawText(QRectF(cx - 6, cy - r - 12, 12, 10), Qt.AlignCenter, "K")

    def _draw_hud(self, p, w, h, step):
        """Sol üstte telemetri kutusu, sağ altta ölçek çubuğu, altta lejant/rozet."""
        # --- Telemetri kutusu ---
        if self.uav:
            rows = [
                ("MOD", self.uav["mode"] or "-", "#38bdf8"),
                ("ALT", f"{self.uav['alt']:.1f} m", "#10b981"),
                ("HIZ", f"{self.uav['speed']:.1f} m/s", "#f59e0b"),
                ("YÖN", f"{self.uav['heading']:.0f}°", "#e2e8f0"),
                # Haritadaki cisimler yalnızca tespit edildikçe eklenir; sayaç
                # kaç cismin haritaya düştüğünü gösterir.
                ("TSP", f"{len(self.targets)} cisim", "#f472b6"),
            ]
            p.setFont(QFont("Roboto Mono", 7, QFont.Bold))
            fm = p.fontMetrics()
            bw = max(fm.horizontalAdvance(f"{k} {v}") for k, v, _ in rows) + 22
            bh = len(rows) * (fm.height() + 1) + 10
            box = QRectF(8, 8, bw, bh)
            p.setPen(QPen(QColor(56, 189, 248, 80), 1))
            p.setBrush(QColor(3, 7, 18, 205))
            p.drawRoundedRect(box, 4, 4)
            ty = box.top() + 6
            for k, v, col in rows:
                p.setPen(QColor("#64748b"))
                p.drawText(QRectF(box.left() + 7, ty, 26, fm.height()), Qt.AlignLeft | Qt.AlignVCenter, k)
                p.setPen(QColor(col))
                p.drawText(QRectF(box.left() + 34, ty, bw - 40, fm.height()), Qt.AlignLeft | Qt.AlignVCenter, v)
                ty += fm.height() + 1

        # --- Ölçek çubuğu (sağ alt) ---
        bar_px = int(step * self.scale)
        y = h - 14
        p.setPen(QPen(QColor("#94a3b8"), 2))
        p.drawLine(w - 12 - bar_px, y, w - 12, y)
        p.drawLine(w - 12 - bar_px, y - 4, w - 12 - bar_px, y + 3)
        p.drawLine(w - 12, y - 4, w - 12, y + 3)
        p.setFont(QFont("Roboto Mono", 6))
        p.setPen(QColor("#cbd5e1"))
        p.drawText(w - 12 - bar_px, y - 7, f"{step:.0f} m   ×{self.zoom:.1f}")

        # --- Lejant (yer varsa) ---
        if h > 210:
            p.setFont(QFont("Roboto Mono", 6))
            items = [("#fbbf24", "rota"), ("#06b6d4", "iz"), ("#22c55e", "kalkış"),
                     ("#ef4444", "tespit")]
            x = 10
            ly = h - 26 if not self.follow_uav else h - 14
            for col, name in items:
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(col))
                p.drawEllipse(QPointF(x + 3, ly - 3), 3, 3)
                p.setPen(QColor("#94a3b8"))
                p.drawText(x + 10, ly, name)
                x += 14 + p.fontMetrics().horizontalAdvance(name)

        # --- Takip modu rozeti ---
        if not self.follow_uav:
            p.setFont(QFont("Roboto Mono", 6, QFont.Bold))
            self._chip(p, 8, h - 16, "SABİT GÖRÜNÜM — çift tık: İHA takibi + zoom sıfırla", "#f59e0b")

    # ------------------------------------------------------------------
    # Çizim
    # ------------------------------------------------------------------
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        w, h = self.width(), self.height()

        self._draw_background(p, w, h)
        step = self._draw_grid(p, w, h)
        self._draw_route(p)
        self._draw_trail(p)
        self._draw_targets(p)
        self._draw_uav(p)
        self._draw_compass(p, w, h)
        self._draw_hud(p, w, h, step)

        # İnce çerçeve — panel kenarı belirgin olsun.
        p.setPen(QPen(QColor("#1e293b"), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(0, 0, w - 1, h - 1)
