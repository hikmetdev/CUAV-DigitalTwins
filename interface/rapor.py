"""Taktik uçuş raporunun PDF çıktısı.

Arayüzdeki "Rapor Oluştur" düğmesi, düz metin raporun yanında bu modülle
görselleştirilmiş bir PDF üretir. Çizim tamamen QPainter + QPdfWriter ile
yapılır (reportlab/matplotlib gibi ek bağımlılık YOK, GUI süreci hafif kalır)
ve sayfa düzeni uygulamanın koyu taktik temasını birebir taşır.

Girdi, app.py'nin derlediği tek bir sözlüktür (bkz. MainWindow._build_report_data);
bu modül uygulamanın iç durumuna hiç dokunmaz, yalnızca verilen veriyi çizer.
"""

import math
from datetime import datetime

from PySide6.QtCore import QMarginsF, QPointF, QRectF, Qt
from PySide6.QtGui import (QBrush, QColor, QFont, QLinearGradient, QPageLayout,
                           QPageSize, QPainter, QPainterPath, QPdfWriter, QPen,
                           QPolygonF)

# --- Uygulama paleti (styles.py ile aynı tonlar) ---
BG = "#030712"        # sayfa zemini
PANEL = "#0d1522"     # kart zemini
LINE = "#1e293b"      # kenarlık
TEXT = "#e2e8f0"      # birincil metin
MUTED = "#64748b"     # ikincil metin
GREEN = "#10b981"
CYAN = "#06b6d4"
AMBER = "#f59e0b"
PURPLE = "#a855f7"
RED = "#ef4444"

FONT_SANS = "DejaVu Sans"
FONT_MONO = "DejaVu Sans Mono"

# Tespit tipine göre işaret rengi (harita + tablo + dağılım grafiği ortak).
TYPE_COLORS = (
    ("kişi", RED), ("insan", RED), ("person", RED),
    ("araç", CYAN), ("araba", CYAN), ("car", CYAN), ("truck", CYAN),
    ("kutu", AMBER), ("qr", AMBER),
    ("ekskavatör", PURPLE), ("excavator", PURPLE), ("iş makinesi", PURPLE),
)


def color_for_type(name):
    low = str(name).lower()
    for key, col in TYPE_COLORS:
        if key in low:
            return col
    return GREEN


class TacticalReportPdf:
    """Veri sözlüğünü çok sayfalı, görselleştirilmiş bir PDF'e dönüştürür."""

    def __init__(self, data):
        self.d = data or {}
        self.page_no = 0

    # ------------------------------------------------------------------
    # ANA AKIŞ
    # ------------------------------------------------------------------
    def save(self, path):
        """Raporu `path` dosyasına yazar. Üretilen sayfa sayısını döner."""
        writer = QPdfWriter(path)
        writer.setPageSize(QPageSize(QPageSize.A4))
        writer.setResolution(150)   # 150 dpi → A4 ≈ 1240x1754 piksel tuval
        # Kenar boşluğu sıfır: koyu zemin sayfanın tamamını kaplasın, iç
        # boşlukları çizim tarafında (self.M) yönetiyoruz.
        writer.setPageMargins(QMarginsF(0, 0, 0, 0), QPageLayout.Millimeter)
        writer.setTitle("SİHA Taktik Uçuş Raporu")
        writer.setCreator("Agentic Digital-Twin GCS")

        p = QPainter(writer)
        p.setRenderHint(QPainter.Antialiasing)
        try:
            self.W = writer.width()
            self.H = writer.height()
            self.M = int(self.W * 0.055)          # kenar boşluğu
            self.CW = self.W - 2 * self.M         # içerik genişliği

            self._page_bg(p)
            y = self._draw_header(p)
            y = self._draw_kpis(p, y + 26)
            y = self._draw_charts(p, y + 30)
            self._draw_route_map(p, y + 30, self.H - self.M - 30 - (y + 30))
            self._footer(p)

            writer.newPage()
            self._page_bg(p)
            y = self._draw_page_title(p, "TESPİT KAYITLARI VE OLAY AKIŞI")
            y = self._draw_detection_table(p, y + 24)
            y = self._draw_class_bars(p, y + 28)
            y = self._draw_timeline(p, y + 28)
            # Son iki panel içerik yüksekliğinde çizilir; sayfayı zorla doldurmak
            # yerine altta boşluk bırakmak baskıda daha temiz görünüyor.
            y = self._draw_vlm(p, y + 28, 170)
            self._draw_stats(p, y + 28, 186)
            self._footer(p)
        finally:
            p.end()
        return self.page_no

    # ------------------------------------------------------------------
    # ORTAK ÇİZİM YARDIMCILARI
    # ------------------------------------------------------------------
    def _page_bg(self, p):
        self.page_no += 1
        p.fillRect(0, 0, self.W, self.H, QColor(BG))
        # Çok hafif ızgara dokusu: "operasyon ekranı" hissi verir, baskıda
        # neredeyse görünmez kalacak kadar düşük kontrastlı.
        p.setPen(QPen(QColor(255, 255, 255, 6), 1))
        step = self.W // 24
        for i in range(1, 24):
            p.drawLine(i * step, 0, i * step, self.H)
        for i in range(1, 34):
            p.drawLine(0, i * step, self.W, i * step)

    def _font(self, size, bold=False, mono=False):
        f = QFont(FONT_MONO if mono else FONT_SANS)
        f.setPixelSize(size)
        f.setBold(bold)
        return f

    def _panel(self, p, rect, accent=None, title=None):
        """Koyu kart: yuvarlatılmış zemin + ince kenarlık + isteğe bağlı sol
        renk şeridi ve başlık. Başlık varsa içeriğin başlayacağı y'yi döner."""
        p.setPen(QPen(QColor(LINE), 1))
        p.setBrush(QBrush(QColor(PANEL)))
        p.drawRoundedRect(rect, 8, 8)
        if accent:
            path = QPainterPath()
            path.addRoundedRect(rect, 8, 8)
            p.save()
            p.setClipPath(path)
            p.fillRect(QRectF(rect.x(), rect.y(), 5, rect.height()), QColor(accent))
            p.restore()
        if title:
            p.setFont(self._font(15, bold=True, mono=True))
            p.setPen(QColor(accent or GREEN))
            p.drawText(QRectF(rect.x() + 18, rect.y() + 12, rect.width() - 36, 24),
                       Qt.AlignLeft | Qt.AlignVCenter, title)
            return rect.y() + 40
        return rect.y() + 12

    def _footer(self, p):
        y = self.H - self.M + 16
        p.setPen(QPen(QColor(LINE), 1))
        p.drawLine(self.M, y - 14, self.W - self.M, y - 14)
        p.setFont(self._font(13, mono=True))
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(self.M, y, self.CW / 2, 24), Qt.AlignLeft | Qt.AlignVCenter,
                   "Agentic Digital-Twin GCS · Otonom SİHA Yer Kontrol İstasyonu")
        p.drawText(QRectF(self.M + self.CW / 2, y, self.CW / 2, 24),
                   Qt.AlignRight | Qt.AlignVCenter, f"Sayfa {self.page_no}")

    def _draw_page_title(self, p, text):
        p.setFont(self._font(26, bold=True))
        p.setPen(QColor(TEXT))
        p.drawText(QRectF(self.M, self.M, self.CW, 40), Qt.AlignLeft | Qt.AlignVCenter, text)
        y = self.M + 46
        p.setPen(QPen(QColor(GREEN), 3))
        p.drawLine(self.M, y, self.M + 90, y)
        p.setPen(QPen(QColor(LINE), 1))
        p.drawLine(self.M + 96, y, self.W - self.M, y)
        return y

    # ------------------------------------------------------------------
    # SAYFA 1 — BAŞLIK
    # ------------------------------------------------------------------
    def _draw_header(self, p):
        h = 150
        rect = QRectF(self.M, self.M, self.CW, h)
        # Koyu lacivertten sayfa zeminine geçen bant.
        grad = QLinearGradient(rect.topLeft(), rect.topRight())
        grad.setColorAt(0.0, QColor("#0b2233"))
        grad.setColorAt(0.6, QColor("#0d1522"))
        grad.setColorAt(1.0, QColor("#0a1119"))
        path = QPainterPath()
        path.addRoundedRect(rect, 10, 10)
        p.setPen(Qt.NoPen)
        p.fillPath(path, QBrush(grad))
        p.setPen(QPen(QColor(GREEN), 2))
        p.drawLine(int(rect.x()), int(rect.y() + h), int(rect.x() + 150), int(rect.y() + h))
        p.setPen(QPen(QColor(LINE), 1))
        p.drawLine(int(rect.x() + 150), int(rect.y() + h), int(rect.right()), int(rect.y() + h))

        gen = self.d.get("generated_at") or datetime.now()

        p.setFont(self._font(13, bold=True, mono=True))
        p.setPen(QColor(GREEN))
        p.drawText(QRectF(rect.x() + 26, rect.y() + 22, rect.width() - 52, 20),
                   Qt.AlignLeft | Qt.AlignVCenter, "AGENTIC DIGITAL-TWIN GCS")

        p.setFont(self._font(34, bold=True))
        p.setPen(QColor(TEXT))
        p.drawText(QRectF(rect.x() + 26, rect.y() + 46, rect.width() - 52, 44),
                   Qt.AlignLeft | Qt.AlignVCenter, "TAKTİK UÇUŞ RAPORU")

        p.setFont(self._font(14, mono=True))
        p.setPen(QColor(MUTED))
        src = "VİDEO AKIŞI (drone.mp4)" if self.d.get("camera_mode") == "video" else "GAZEBO SİMÜLASYON KAMERASI"
        p.drawText(QRectF(rect.x() + 26, rect.y() + 94, rect.width() - 52, 22),
                   Qt.AlignLeft | Qt.AlignVCenter,
                   f"Oluşturma: {gen.strftime('%d.%m.%Y %H:%M:%S')}   ·   Görüntü kaynağı: {src}")

        # Sağ üstte durum rozeti
        mode = str(self.d.get("telemetry", {}).get("mode", "-"))
        badge_col = RED if mode.upper() == "DISCONNECTED" else GREEN
        bw, bh = 190, 34
        br = QRectF(rect.right() - bw - 26, rect.y() + 24, bw, bh)
        p.setPen(QPen(QColor(badge_col), 1))
        p.setBrush(QBrush(QColor(badge_col).darker(400)))
        p.drawRoundedRect(br, 6, 6)
        p.setFont(self._font(14, bold=True, mono=True))
        p.setPen(QColor(badge_col))
        p.drawText(br, Qt.AlignCenter, f"● UÇUŞ MODU: {mode}")

        return rect.y() + h

    # ------------------------------------------------------------------
    # SAYFA 1 — ÖZET KARTLARI
    # ------------------------------------------------------------------
    def _draw_kpis(self, p, y):
        hist = self.d.get("history", [])
        tel = self.d.get("telemetry", {})
        dets = self.d.get("detections", [])

        alts = [float(s.get("alt", 0.0)) for s in hist] or [0.0]
        spds = [float(s.get("speed", 0.0)) for s in hist] or [0.0]
        bats = [float(s.get("battery", 0.0)) for s in hist] or [float(tel.get("battery", 0.0))]

        mins = self.d.get("mission_min", 0.0)
        cards = [
            ("UÇUŞ SÜRESİ", f"{int(mins):02d}:{int((mins % 1) * 60):02d}", "dk:sn", CYAN),
            ("MAKS. İRTİFA", f"{max(alts):.0f}", "m", GREEN),
            ("ORT. HIZ", f"{sum(spds) / len(spds):.1f}", "m/s", AMBER),
            ("BATARYA", f"{bats[-1]:.0f}", "%", self._battery_color(bats[-1])),
            ("TESPİT", f"{len(dets)}", "hedef", PURPLE),
        ]

        gap = 14
        cw = (self.CW - gap * (len(cards) - 1)) / len(cards)
        ch = 96
        for i, (label, value, unit, col) in enumerate(cards):
            r = QRectF(self.M + i * (cw + gap), y, cw, ch)
            self._panel(p, r, accent=col)
            p.setFont(self._font(12, bold=True, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(r.x() + 18, r.y() + 14, r.width() - 26, 18),
                       Qt.AlignLeft | Qt.AlignVCenter, label)
            p.setFont(self._font(32, bold=True))
            p.setPen(QColor(col))
            p.drawText(QRectF(r.x() + 18, r.y() + 36, r.width() - 26, 40),
                       Qt.AlignLeft | Qt.AlignVCenter, value)
            p.setFont(self._font(12, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(r.x() + 18, r.y() + 72, r.width() - 26, 16),
                       Qt.AlignLeft | Qt.AlignVCenter, unit)
        return y + ch

    @staticmethod
    def _battery_color(pct):
        # app.py'deki BATTERY_TIERS ile aynı eşikler.
        if pct >= 70:
            return GREEN
        if pct >= 40:
            return "#facc15"
        if pct >= 20:
            return "#f97316"
        return RED

    # ------------------------------------------------------------------
    # SAYFA 1 — TELEMETRİ GRAFİKLERİ
    # ------------------------------------------------------------------
    def _draw_charts(self, p, y):
        hist = self.d.get("history", [])
        gap = 14
        cw = (self.CW - gap * 2) / 3
        ch = 220
        series = [
            ("İRTİFA", "alt", "m", CYAN),
            ("HIZ", "speed", "m/s", GREEN),
            ("BATARYA", "battery", "%", AMBER),
        ]
        for i, (title, key, unit, col) in enumerate(series):
            r = QRectF(self.M + i * (cw + gap), y, cw, ch)
            top = self._panel(p, r, accent=col, title=f"{title} ({unit})")
            vals = [float(s.get(key, 0.0)) for s in hist]
            self._line_chart(p, QRectF(r.x() + 16, top + 6, r.width() - 32, r.bottom() - top - 34),
                             vals, col, unit)
        return y + ch

    def _line_chart(self, p, rect, values, color, unit):
        """Alan dolgulu çizgi grafiği: ızgara, min/maks etiketleri, son değer
        rozeti. Veri yoksa bilgilendirici bir yazı basar."""
        if len(values) < 2:
            p.setFont(self._font(13, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(rect, Qt.AlignCenter, "Yeterli veri yok")
            return

        vmin, vmax = min(values), max(values)
        if vmax - vmin < 1e-6:      # düz seyir: çizgi ortada dursun
            vmax, vmin = vmax + 1.0, vmin - 1.0
        span = vmax - vmin

        # Yatay ızgara (4 bölme)
        p.setPen(QPen(QColor(255, 255, 255, 16), 1, Qt.DashLine))
        for i in range(1, 4):
            gy = rect.y() + rect.height() * i / 4.0
            p.drawLine(QPointF(rect.x(), gy), QPointF(rect.right(), gy))

        n = len(values)
        pts = [QPointF(rect.x() + rect.width() * i / (n - 1),
                       rect.bottom() - (values[i] - vmin) / span * rect.height())
               for i in range(n)]

        # Çizginin altını renkli degrade ile doldur (hacim hissi)
        area = QPolygonF(pts + [QPointF(rect.right(), rect.bottom()),
                                QPointF(rect.x(), rect.bottom())])
        grad = QLinearGradient(rect.topLeft(), rect.bottomLeft())
        c = QColor(color)
        grad.setColorAt(0.0, QColor(c.red(), c.green(), c.blue(), 110))
        grad.setColorAt(1.0, QColor(c.red(), c.green(), c.blue(), 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(grad))
        p.drawPolygon(area)

        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(color), 2.2))
        p.drawPolyline(QPolygonF(pts))

        # Son nokta işareti
        p.setBrush(QBrush(QColor(color)))
        p.setPen(QPen(QColor(BG), 2))
        p.drawEllipse(pts[-1], 5, 5)

        # Min / maks etiketleri (sol kenarda, koyu zeminde okunur)
        p.setFont(self._font(11, mono=True))
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(rect.x(), rect.y() - 4, rect.width(), 16),
                   Qt.AlignLeft | Qt.AlignVCenter, f"maks {vmax:.1f}")
        p.drawText(QRectF(rect.x(), rect.bottom() - 12, rect.width(), 16),
                   Qt.AlignLeft | Qt.AlignVCenter, f"min {vmin:.1f}")

        # Güncel değer rozeti (sağ üst, opak zemin → rakam her zaman okunur)
        self._badge(p, rect.right(), rect.y() - 6, f"{values[-1]:.1f}{unit}", color)

    def _badge(self, p, x_right, y_top, text, color):
        p.setFont(self._font(13, bold=True, mono=True))
        fm = p.fontMetrics()
        tw, th = fm.horizontalAdvance(text), fm.height()
        r = QRectF(x_right - tw - 16, y_top, tw + 16, th + 8)
        p.setPen(QPen(QColor(color), 1))
        p.setBrush(QBrush(QColor(3, 7, 18, 240)))
        p.drawRoundedRect(r, 4, 4)
        p.setBrush(Qt.NoBrush)
        p.setPen(QColor(TEXT))
        p.drawText(r, Qt.AlignCenter, text)

    # ------------------------------------------------------------------
    # SAYFA 1 — ROTA HARİTASI
    # ------------------------------------------------------------------
    def _draw_route_map(self, p, y, h):
        r = QRectF(self.M, y, self.CW, h)
        top = self._panel(p, r, accent=GREEN, title="UÇUŞ ROTASI VE HEDEF DAĞILIMI")
        plot = QRectF(r.x() + 20, top + 6, r.width() - 40, r.bottom() - top - 26)

        hist = self.d.get("history", [])
        wps = self.d.get("waypoints", [])
        dets = [d for d in self.d.get("detections", [])
                if d.get("lat") is not None and d.get("lon") is not None]

        pts_ll = [(float(s["lat"]), float(s["lon"])) for s in hist
                  if s.get("lat") and s.get("lon")]
        all_ll = pts_ll + [(float(w["lat"]), float(w["lon"])) for w in wps] \
                        + [(float(d["lat"]), float(d["lon"])) for d in dets]
        if not all_ll:
            p.setFont(self._font(14, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(plot, Qt.AlignCenter, "Konum verisi kaydedilmedi (video akışı veya GPS yok)")
            return

        lats = [a for a, _ in all_ll]
        lons = [b for _, b in all_ll]
        lat0 = sum(lats) / len(lats)
        # Coğrafi koordinatı metreye çevirip en-boy oranını koru; aksi halde
        # rota kuzey-güney yönünde ezilmiş görünür.
        mx = [(lo - min(lons)) * 111000.0 * math.cos(math.radians(lat0)) for lo in lons]
        my = [(la - min(lats)) * 111000.0 for la in lats]
        span = max(max(mx) - min(mx), max(my) - min(my), 20.0) * 1.15
        cx_m = (max(mx) + min(mx)) / 2.0
        cy_m = (max(my) + min(my)) / 2.0
        side = min(plot.width(), plot.height())
        ox = plot.x() + (plot.width() - side) / 2.0
        oy = plot.y() + (plot.height() - side) / 2.0

        def to_px(lat, lon):
            X = (lon - min(lons)) * 111000.0 * math.cos(math.radians(lat0))
            Y = (lat - min(lats)) * 111000.0
            return QPointF(ox + side / 2 + (X - cx_m) / span * side,
                           oy + side / 2 - (Y - cy_m) / span * side)   # kuzey yukarı

        # Harita çerçevesi + ızgara
        frame = QRectF(ox, oy, side, side)
        p.setPen(QPen(QColor(LINE), 1))
        p.setBrush(QBrush(QColor("#081019")))
        p.drawRect(frame)
        p.setPen(QPen(QColor(255, 255, 255, 14), 1, Qt.DashLine))
        for i in range(1, 6):
            gx = frame.x() + side * i / 6.0
            gy = frame.y() + side * i / 6.0
            p.drawLine(QPointF(gx, frame.y()), QPointF(gx, frame.bottom()))
            p.drawLine(QPointF(frame.x(), gy), QPointF(frame.right(), gy))

        # Rota izi
        if len(pts_ll) > 1:
            track = QPolygonF([to_px(a, b) for a, b in pts_ll])
            p.setPen(QPen(QColor(CYAN), 2.4))
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(track)

        # Görev noktaları (elmas) ve etiketleri
        p.setFont(self._font(11, mono=True))
        for w in wps:
            q = to_px(float(w["lat"]), float(w["lon"]))
            dia = QPolygonF([QPointF(q.x(), q.y() - 7), QPointF(q.x() + 7, q.y()),
                             QPointF(q.x(), q.y() + 7), QPointF(q.x() - 7, q.y())])
            p.setPen(QPen(QColor(BG), 1.5))
            p.setBrush(QBrush(QColor(AMBER)))
            p.drawPolygon(dia)
            p.setPen(QColor(MUTED))
            p.drawText(QPointF(q.x() + 11, q.y() + 4), str(w.get("name", "")))

        # Tespitler (renkli nokta + halka)
        for d in dets:
            q = to_px(float(d["lat"]), float(d["lon"]))
            col = QColor(color_for_type(d.get("type", "")))
            p.setPen(QPen(QColor(col.red(), col.green(), col.blue(), 120), 1))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(q, 11, 11)
            p.setPen(QPen(QColor(BG), 1))
            p.setBrush(QBrush(col))
            p.drawEllipse(q, 5, 5)

        # Başlangıç / güncel konum
        if pts_ll:
            s = to_px(*pts_ll[0])
            e = to_px(*pts_ll[-1])
            p.setPen(QPen(QColor(BG), 1.5))
            p.setBrush(QBrush(QColor(GREEN)))
            p.drawEllipse(s, 6, 6)
            p.setFont(self._font(11, bold=True, mono=True))
            p.setPen(QColor(GREEN))
            p.drawText(QPointF(s.x() + 10, s.y() - 8), "KALKIŞ")
            # Güncel konum: uçuş yönüne bakan üçgen
            hd = float(self.d.get("telemetry", {}).get("heading", 0.0))
            p.save()
            p.translate(e)
            p.rotate(hd)
            p.setPen(QPen(QColor(BG), 1.5))
            p.setBrush(QBrush(QColor(RED)))
            p.drawPolygon(QPolygonF([QPointF(0, -11), QPointF(7, 8), QPointF(0, 4), QPointF(-7, 8)]))
            p.restore()
            p.setPen(QColor(RED))
            # Etiket işaretin sol-altına: yukarıdaki görev noktası etiketleriyle
            # üst üste binmesin.
            p.drawText(QRectF(e.x() - 92, e.y() + 6, 84, 16),
                       Qt.AlignRight | Qt.AlignVCenter, "SON KONUM")

        # Kuzey oku
        nx, ny = frame.right() - 34, frame.y() + 34
        p.setPen(QPen(QColor(MUTED), 1))
        p.setBrush(QBrush(QColor(CYAN)))
        p.drawPolygon(QPolygonF([QPointF(nx, ny - 16), QPointF(nx + 6, ny + 4), QPointF(nx - 6, ny + 4)]))
        p.setFont(self._font(11, bold=True, mono=True))
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(nx - 12, ny + 6, 24, 14), Qt.AlignCenter, "K")

        # Ölçek çubuğu (span metreden türetilir)
        bar_m = self._nice_scale(span / 4.0)
        bar_px = bar_m / span * side
        bx, by = frame.x() + 16, frame.bottom() - 18
        p.setPen(QPen(QColor(TEXT), 2))
        p.drawLine(QPointF(bx, by), QPointF(bx + bar_px, by))
        p.drawLine(QPointF(bx, by - 4), QPointF(bx, by + 4))
        p.drawLine(QPointF(bx + bar_px, by - 4), QPointF(bx + bar_px, by + 4))
        p.setFont(self._font(11, mono=True))
        p.setPen(QColor(MUTED))
        p.drawText(QPointF(bx + bar_px + 8, by + 4), f"{bar_m:.0f} m")

    @staticmethod
    def _nice_scale(raw):
        """Ölçek çubuğu için yuvarlak bir uzunluk (10/25/50/100... m) seçer."""
        for step in (10, 25, 50, 100, 250, 500, 1000, 2500, 5000):
            if raw <= step:
                return float(step)
        return float(10000)

    # ------------------------------------------------------------------
    # SAYFA 2 — TESPİT TABLOSU
    # ------------------------------------------------------------------
    def _draw_detection_table(self, p, y):
        dets = self.d.get("detections", [])
        rows = dets[:18]                     # sayfaya sığan ilk 18 kayıt
        row_h = 30
        h = 52 + max(1, len(rows)) * row_h + 16
        r = QRectF(self.M, y, self.CW, h)
        top = self._panel(p, r, accent=CYAN,
                          title=f"TESPİT EDİLEN HEDEFLER ({len(dets)} kayıt)")

        cols = [("ID", 0.09), ("NESNE", 0.24), ("GÜVEN", 0.20),
                ("İLK GÖRÜLME", 0.15), ("SON", 0.12), ("KONUM", 0.20)]
        x0 = r.x() + 20
        tw = r.width() - 40

        p.setFont(self._font(11, bold=True, mono=True))
        p.setPen(QColor(MUTED))
        cx = x0
        for name, frac in cols:
            p.drawText(QRectF(cx, top, tw * frac, 18), Qt.AlignLeft | Qt.AlignVCenter, name)
            cx += tw * frac
        p.setPen(QPen(QColor(LINE), 1))
        p.drawLine(QPointF(x0, top + 20), QPointF(x0 + tw, top + 20))

        if not rows:
            p.setFont(self._font(13, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(x0, top + 26, tw, row_h), Qt.AlignLeft | Qt.AlignVCenter,
                       "Bu görevde hedef tespit edilmedi.")
            return r.bottom()

        ry = top + 26
        for i, d in enumerate(rows):
            if i % 2 == 0:      # zebra: satır takibi kolaylaşsın
                p.fillRect(QRectF(x0 - 8, ry, tw + 16, row_h), QColor(255, 255, 255, 8))
            col = QColor(color_for_type(d.get("type", "")))
            conf = float(d.get("conf", 0.0))
            cx = x0

            p.setFont(self._font(12, bold=True, mono=True))
            p.setPen(col)
            p.drawText(QRectF(cx, ry, tw * cols[0][1], row_h), Qt.AlignLeft | Qt.AlignVCenter,
                       str(d.get("id", "-")))
            cx += tw * cols[0][1]

            p.setFont(self._font(12))
            p.setPen(QColor(TEXT))
            name = str(d.get("type", "-"))
            if d.get("qr_text"):
                name += "  [QR]"
            p.drawText(QRectF(cx, ry, tw * cols[1][1] - 8, row_h),
                       Qt.AlignLeft | Qt.AlignVCenter, name)
            cx += tw * cols[1][1]

            # Güven: mini dolu çubuk + yüzde
            bw = tw * cols[2][1] - 60
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(255, 255, 255, 18)))
            p.drawRoundedRect(QRectF(cx, ry + row_h / 2 - 4, bw, 8), 4, 4)
            p.setBrush(QBrush(col))
            p.drawRoundedRect(QRectF(cx, ry + row_h / 2 - 4, max(3.0, bw * conf), 8), 4, 4)
            p.setBrush(Qt.NoBrush)
            p.setFont(self._font(12, bold=True, mono=True))
            p.setPen(col)
            p.drawText(QRectF(cx + bw + 8, ry, 52, row_h), Qt.AlignLeft | Qt.AlignVCenter,
                       f"%{int(conf * 100)}")
            cx += tw * cols[2][1]

            p.setFont(self._font(12, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(cx, ry, tw * cols[3][1], row_h), Qt.AlignLeft | Qt.AlignVCenter,
                       str(d.get("first_time", "-")))
            cx += tw * cols[3][1]
            p.drawText(QRectF(cx, ry, tw * cols[4][1], row_h), Qt.AlignLeft | Qt.AlignVCenter,
                       str(d.get("time", "-")))
            cx += tw * cols[4][1]

            lat, lon = d.get("lat"), d.get("lon")
            loc = f"{lat:.5f}, {lon:.5f}" if lat is not None and lon is not None else "video akışı"
            p.drawText(QRectF(cx, ry, tw * cols[5][1], row_h), Qt.AlignLeft | Qt.AlignVCenter, loc)
            ry += row_h

        if len(dets) > len(rows):
            p.setFont(self._font(11, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(x0, ry, tw, 16), Qt.AlignLeft | Qt.AlignVCenter,
                       f"... ve {len(dets) - len(rows)} kayıt daha (tam liste metin raporunda).")
        return r.bottom()

    # ------------------------------------------------------------------
    # SAYFA 2 — SINIF DAĞILIMI
    # ------------------------------------------------------------------
    def _draw_class_bars(self, p, y):
        dets = self.d.get("detections", [])
        counts = {}
        for d in dets:
            t = str(d.get("type", "?"))
            counts[t] = counts.get(t, 0) + 1
        items = sorted(counts.items(), key=lambda kv: -kv[1])[:6]

        h = 60 + max(1, len(items)) * 30
        r = QRectF(self.M, y, self.CW, h)
        top = self._panel(p, r, accent=PURPLE, title="HEDEF SINIF DAĞILIMI")
        if not items:
            p.setFont(self._font(13, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(r.x() + 20, top, r.width() - 40, 30),
                       Qt.AlignLeft | Qt.AlignVCenter, "Sınıflandırılacak tespit yok.")
            return r.bottom()

        mx = max(c for _, c in items)
        label_w = r.width() * 0.24
        bar_x = r.x() + 20 + label_w
        bar_max = r.width() - 40 - label_w - 70
        by = top + 4
        for name, cnt in items:
            col = QColor(color_for_type(name))
            p.setFont(self._font(12, mono=True))
            p.setPen(QColor(TEXT))
            p.drawText(QRectF(r.x() + 20, by, label_w - 10, 24),
                       Qt.AlignLeft | Qt.AlignVCenter, name)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(255, 255, 255, 14)))
            p.drawRoundedRect(QRectF(bar_x, by + 5, bar_max, 14), 7, 7)
            p.setBrush(QBrush(col))
            p.drawRoundedRect(QRectF(bar_x, by + 5, max(8.0, bar_max * cnt / mx), 14), 7, 7)
            p.setBrush(Qt.NoBrush)
            p.setFont(self._font(13, bold=True, mono=True))
            p.setPen(col)
            p.drawText(QRectF(bar_x + bar_max + 12, by, 60, 24),
                       Qt.AlignLeft | Qt.AlignVCenter, f"{cnt} adet")
            by += 30
        return r.bottom()

    # ------------------------------------------------------------------
    # SAYFA 2 — OLAY ZAMAN ÇİZELGESİ
    # ------------------------------------------------------------------
    def _draw_timeline(self, p, y):
        evs = self.d.get("timeline", [])[-6:]
        h = 130
        r = QRectF(self.M, y, self.CW, h)
        top = self._panel(p, r, accent=AMBER, title="GÖREV OLAY AKIŞI")
        if not evs:
            p.setFont(self._font(13, mono=True))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(r.x() + 20, top, r.width() - 40, 30),
                       Qt.AlignLeft | Qt.AlignVCenter, "Kaydedilmiş olay yok.")
            return r.bottom()

        # Uçlardaki olayların açıklama metni panel dışına taşmasın diye eksen
        # kenarlardan bir etiket yarısı kadar içeride başlatılır.
        axis_y = top + 34
        x0 = r.x() + 84
        x1 = r.right() - 84
        p.setPen(QPen(QColor(LINE), 2))
        p.drawLine(QPointF(x0, axis_y), QPointF(x1, axis_y))

        colors = {"detection": RED, "command": GREEN, "waypoint": CYAN, "person": AMBER}
        n = len(evs)
        for i, ev in enumerate(evs):
            ex = x0 + (x1 - x0) * (i / max(1, n - 1)) if n > 1 else (x0 + x1) / 2
            col = QColor(colors.get(ev.get("type"), GREEN))
            p.setPen(QPen(QColor(BG), 2))
            p.setBrush(QBrush(col))
            p.drawEllipse(QPointF(ex, axis_y), 7, 7)
            p.setFont(self._font(11, bold=True, mono=True))
            p.setPen(col)
            p.drawText(QRectF(ex - 60, axis_y - 28, 120, 16), Qt.AlignCenter, str(ev.get("time", "")))
            p.setFont(self._font(10))
            p.setPen(QColor(MUTED))
            desc = str(ev.get("desc", ""))
            p.drawText(QRectF(ex - 72, axis_y + 12, 144, 34),
                       Qt.AlignHCenter | Qt.AlignTop | Qt.TextWordWrap, desc)
        return r.bottom()

    # ------------------------------------------------------------------
    # SAYFA 2 — VLM SAHNE ÖZETİ
    # ------------------------------------------------------------------
    def _draw_vlm(self, p, y, h):
        h = max(110.0, h)
        r = QRectF(self.M, y, self.CW, h)
        top = self._panel(p, r, accent=GREEN, title="SAHNE ANALİZİ (VLM ÖZETİ)")
        text = (self.d.get("vlm") or "").strip() or "Sahne özeti üretilmedi."
        p.setFont(self._font(13))
        p.setPen(QColor(TEXT))
        p.drawText(QRectF(r.x() + 20, top, r.width() - 40, r.bottom() - top - 16),
                   Qt.AlignLeft | Qt.AlignTop | Qt.TextWordWrap, text)
        return r.bottom()

    # ------------------------------------------------------------------
    # SAYFA 2 — TELEMETRİ İSTATİSTİKLERİ
    # ------------------------------------------------------------------
    def _draw_stats(self, p, y, h):
        """Uçuş geçmişinden türetilen min/maks/ortalama tablosu. Metin raporunun
        'Telemetry History' bölümünün görsel karşılığıdır."""
        h = max(150.0, h)
        r = QRectF(self.M, y, self.CW, h)
        top = self._panel(p, r, accent=CYAN, title="UÇUŞ İSTATİSTİKLERİ")
        hist = self.d.get("history", [])
        tel = self.d.get("telemetry", {})

        def col(key):
            return [float(s.get(key, 0.0)) for s in hist] or [0.0]

        alts, spds, bats, dists = col("alt"), col("speed"), col("battery"), col("dist")
        rows = [
            ("İrtifa", f"{min(alts):.1f} m", f"{max(alts):.1f} m", f"{sum(alts)/len(alts):.1f} m", CYAN),
            ("Hava hızı", f"{min(spds):.1f} m/s", f"{max(spds):.1f} m/s", f"{sum(spds)/len(spds):.1f} m/s", GREEN),
            ("Batarya", f"{min(bats):.0f} %", f"{max(bats):.0f} %", f"{bats[-1]:.0f} % (son)", self._battery_color(bats[-1])),
            ("Kat edilen mesafe", "0 m", f"{max(dists):.0f} m", f"{tel.get('voltage', 0)} V (gerilim)", PURPLE),
        ]

        x0 = r.x() + 20
        tw = r.width() - 40
        fracs = (0.34, 0.22, 0.22, 0.22)
        heads = ("PARAMETRE", "EN DÜŞÜK", "EN YÜKSEK", "ORTALAMA")
        p.setFont(self._font(11, bold=True, mono=True))
        p.setPen(QColor(MUTED))
        cx = x0
        for name, fr in zip(heads, fracs):
            p.drawText(QRectF(cx, top, tw * fr, 18), Qt.AlignLeft | Qt.AlignVCenter, name)
            cx += tw * fr
        p.setPen(QPen(QColor(LINE), 1))
        p.drawLine(QPointF(x0, top + 20), QPointF(x0 + tw, top + 20))

        ry = top + 26
        for i, (name, lo, hi, avg, c) in enumerate(rows):
            if i % 2 == 0:
                p.fillRect(QRectF(x0 - 8, ry, tw + 16, 28), QColor(255, 255, 255, 8))
            cx = x0
            p.setFont(self._font(12))
            p.setPen(QColor(TEXT))
            p.drawText(QRectF(cx, ry, tw * fracs[0], 28), Qt.AlignLeft | Qt.AlignVCenter, name)
            cx += tw * fracs[0]
            p.setFont(self._font(12, bold=True, mono=True))
            for val, fr in zip((lo, hi, avg), fracs[1:]):
                p.setPen(QColor(c))
                p.drawText(QRectF(cx, ry, tw * fr, 28), Qt.AlignLeft | Qt.AlignVCenter, val)
                cx += tw * fr
            ry += 28

        p.setFont(self._font(11, mono=True))
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(x0, ry + 6, tw, 18), Qt.AlignLeft | Qt.AlignVCenter,
                   f"Kaynak: 1 Hz telemetri tamponu · {len(hist)} örnek · "
                   f"otopilot modu {tel.get('mode', '-')}")
        return r.bottom()
