import cv2
import os
import time

# ultralytics içe aktarılırken matplotlib'i de yükler. GUI sürecinde (bir
# QApplication zaten açıkken) matplotlib'in Qt arka ucu PySide6 ile çakışıp
# "int() ... KeyboardModifier" hatası veriyor. Etkileşimsiz 'Agg' arka ucunu
# import'tan ÖNCE zorlayarak bunu önle — uygulama matplotlib'i görüntü için
# kullanmaz (tüm çizim QPainter widget'larıyla). yolo.py ayrı alt süreç olduğu
# için bu ayardan etkilenmez.
os.environ.setdefault("MPLBACKEND", "Agg")

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

# threads/ klasörünün bir üstü = proje kökü. YOLOE model ağırlığı (yoloe-26s-seg.pt)
# ve text-encoder (mobileclip2_b.ts) burada durur; mutlak yolla yüklenir ki
# uygulamanın çalışma dizininden bağımsız çalışsın.
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class VideoYoloThread(QThread):
    """Yerel bir video dosyasını (ör. drone.mp4) OpenCV ile okuyup her karede
    hazır YOLOE modelini çalıştırır, kutuları/maskeleri çizip QImage olarak
    yayınlar. ROS/Gazebo akışından TAMAMEN bağımsızdır: mevcut canlı kamera
    sistemine dokunmaz, yalnızca 'Canlı Görüntüye Geç' butonuyla devreye girer.

    Ağır importlar (torch/ultralytics) ve model yüklemesi run() içinde, yani
    ayrı thread'de yapılır; böylece buton tıklanınca arayüz donmaz."""

    frame_received = Signal(QImage)
    log_signal = Signal(str, str)
    # Model ilk kez yüklendiğinde ana pencereye geri verilir; sonraki
    # etkinleştirmelerde yeniden yüklemek yerine önbellekten kullanılır (saniyeler
    # yerine anında açılır). object taşıyan sinyal thread'ler arası güvenli.
    model_ready = Signal(object)
    # İlk gerçek kare yayınlandığında (model hazır, video akıyor) tetiklenir;
    # ana pencere buton durumunu buna göre günceller.
    stream_started = Signal()
    # Model/video hazırlanamayınca (import, model yükleme, video açma hatası)
    # tetiklenir; ana pencere yükleme ekranını kapatıp Gazebo'ya döner.
    load_failed = Signal(str)
    # Her işlenen karenin tespitleri (cls/conf/track_id/bbox). Ana pencere bunu
    # tespit tablosuna + sahne analizine + AI Copilot snapshot'ına düşürür.
    # yolo.py'nin ROS /yolo/detections paketiyle aynı sözlük şeması kullanılır.
    detections_ready = Signal(dict)

    def __init__(self, video_path, model_name="yoloe-26s-seg.pt",
                 classes=None, loop=True, model=None, parent=None):
        super().__init__(parent)
        self.video_path = video_path
        self.model_name = model_name
        # Önceden yüklenmiş YOLOE modeli (önbellek). Verilirse tekrar yüklenmez.
        self.model = model
        # Gerçek hava aracı görüntüsü için sınıflar: araba galerisi + inşaat
        # sahnesindeki tipik cisimler. YOLOE açık-sözcük (open-vocab) olduğu için
        # bu metin komutları set_classes ile doğrudan verilebilir.
        self.classes = classes or [
            "car", "truck", "bus", "van", "person",
            "excavator", "bulldozer", "construction vehicle",
        ]
        self.loop = loop
        self.running = False
        # Gerçek görüntüde güven Gazebo'daki sentetik cisimlerden yüksek; 0.25
        # dengeli bir eşik. Çıkarım çözünürlüğü ve FPS sınırı env'den ayarlanabilir.
        self.conf = float(os.environ.get("VIDEO_YOLO_CONF", "0.25"))
        self.imgsz = int(os.environ.get("VIDEO_YOLO_IMGSZ", "640"))
        self.max_fps = float(os.environ.get("VIDEO_YOLO_MAX_FPS", "15"))
        # Oynatma hızı çarpanı (1.0 = gerçek zaman). 1.25 → video %25 hızlı akar.
        # CPU çıkarımı yavaş olsa bile oynatma duvar-saatine senkronlanıp gerektiğinde
        # kare atlanır; böylece hız çarpanı çıkarım hızından bağımsız korunur.
        self.speed = float(os.environ.get("VIDEO_YOLO_SPEED", "1.25"))

    def run(self):
        self.running = True
        # --- Ağır bağımlılıklar tembel yüklenir (app importunu hafif tutar) ---
        try:
            # matplotlib'i ultralytics'ten ÖNCE Agg'e sabitle (Qt arka ucu çakışması).
            import matplotlib
            matplotlib.use("Agg", force=True)
            import torch
            from ultralytics import YOLOE
        except Exception as e:
            self.log_signal.emit(f"YOLOE/torch yüklenemedi: {e}", "WARN")
            self.load_failed.emit(str(e))
            return

        # yolo.py ile aynı cihaz/thread mantığı: GPU varsa VRAM'e al, yoksa
        # CPU'da çalış ama tüm çekirdekleri kaplama (Gazebo/EKF aç kalmasın).
        if torch.cuda.is_available():
            device = os.environ.get("YOLO_DEVICE", "0")
            half = os.environ.get("YOLO_HALF", "1") == "1"
        else:
            device = "cpu"
            half = False
            try:
                torch.set_num_threads(int(os.environ.get("YOLO_CPU_THREADS", "4")))
            except Exception:
                pass

        # Model önbellekte varsa yeniden yükleme; yoksa bir kez yükleyip ana
        # pencereye geri ver (model_ready) ki sonraki açılışlar anında olsun.
        model = self.model
        if model is None:
            self.log_signal.emit(
                f"Video için YOLOE modeli yükleniyor: {self.model_name} "
                f"(cihaz={device}, ilk açılış birkaç saniye sürebilir)...", "INFO"
            )
            try:
                model = YOLOE(os.path.join(BASE_DIR, self.model_name))
                model.set_classes(self.classes)
            except Exception as e:
                self.log_signal.emit(f"YOLOE modeli hazırlanamadı: {e}", "WARN")
                self.load_failed.emit(str(e))
                return
            # Yükleme sırasında kullanıcı vazgeçtiyse (stop çağrıldı) boşuna devam etme.
            if not self.running:
                return
            self.model = model
            self.model_ready.emit(model)
            self.log_signal.emit(
                "YOLOE modeli hazır; video akışında nesne tespiti başladı.", "INFO"
            )

        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            self.log_signal.emit(f"Video açılamadı: {self.video_path}", "WARN")
            self.load_failed.emit("video açılamadı")
            return

        min_interval = (1.0 / self.max_fps) if self.max_fps > 0 else 0.0
        # Kaynak videonun kendi kare hızı — hız çarpanı bunun üstüne uygulanır.
        src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        first = True
        play_start = time.time()   # oynatma zaman referansı (duvar saati)
        frame_pos = 0.0            # şu anki kaynak kare indeksi
        while self.running:
            t0 = time.time()

            # --- Duvar-saati senkronu + hız çarpanı (kare atlama) ---
            # Geçen gerçek süre × kaynak_fps × hız = şu an gösterilmesi gereken
            # kaynak kare indeksi. Çıkarım geride kaldıysa aradaki kareler ucuz
            # grab() ile atlanır (decode etmeden) → video 1.25x akıcı akar.
            target_idx = (time.time() - play_start) * src_fps * self.speed
            skip = int(target_idx - frame_pos) - 1
            for _ in range(max(0, skip)):
                if not cap.grab():
                    break
                frame_pos += 1

            ok, frame = cap.read()
            frame_pos += 1
            self._frame_pos = frame_pos   # teşhis/gözlem için (oynatma konumu)
            if not ok:
                # Video bitti: baştan başlat (loop) ya da çık.
                if self.loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    frame_pos = 0.0
                    play_start = time.time()
                    continue
                break

            try:
                # persist=True ile ByteTrack takip ID'lerini kareler arası korur;
                # bu, kutuların titremeden stabil görünmesini sağlar.
                results = model.track(
                    frame, persist=True, conf=self.conf, imgsz=self.imgsz,
                    device=device, half=half, verbose=False,
                    tracker="bytetrack.yaml",
                )
                # Ultralytics'in yerleşik çizici'si: kutu + maske + etiket + güven.
                annotated = results[0].plot()
                # Tespitleri sözlük listesine çıkar (tablo/sahne analizi/chatbot için).
                self.detections_ready.emit({"detections": self._extract_dets(results[0])})
            except Exception as e:
                self.log_signal.emit(f"Video çıkarım hatası: {e}", "WARN")
                annotated = frame

            # OpenCV BGR -> Qt RGB. .copy() ile numpy tamponundan bağımsız kopya
            # alınır (CameraStreamThread ile aynı desen).
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            q_img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
            self.frame_received.emit(q_img)
            if first:
                first = False
                self.stream_started.emit()
                # Zaman referansını ilk GÖSTERİLEN kareye sıfırla: model ısınması
                # (ilk çıkarım ~1-2 sn) sırasında wall-clock ilerlerken frame_pos
                # beklediği için, sıfırlamazsak ısınma bitince video ileri sıçrar.
                # Başa sarmadan (frame_pos olduğu yerde) sadece saati sıfırla.
                play_start = time.time()

            # FPS sınırı: CPU'yu boğmadan sabit bir hızda oynat.
            dt = time.time() - t0
            if min_interval > dt:
                time.sleep(min_interval - dt)

        cap.release()

    @staticmethod
    def _extract_dets(result):
        """Ultralytics sonucundan (result[0]) tespit sözlük listesi çıkarır:
        her biri cls(ad)/conf/track_id/bbox içerir. Hata olursa boş liste döner
        (video akışını asla durdurmaz)."""
        dets = []
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return dets
        names = getattr(result, "names", {})
        for b in boxes:
            try:
                cls_name = names.get(int(b.cls[0]), str(int(b.cls[0])))
                conf = float(b.conf[0])
                tid = int(b.id[0]) if getattr(b, "id", None) is not None else None
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                dets.append({
                    "cls": cls_name,
                    "conf": round(conf, 3),
                    "track_id": tid,
                    "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                })
            except Exception:
                continue
        return dets

    def stop(self):
        self.running = False
        self.wait()
