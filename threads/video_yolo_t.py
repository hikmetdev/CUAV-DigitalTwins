import cv2
import numpy as np
import os
import threading
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
        # Gösterim (decode + çizim) kare hızı üst sınırı. Çıkarım artık ayrı
        # thread'de olduğu için bu sınır YALNIZCA görüntüleme hızını belirler;
        # CPU'da YOLO 2-3 FPS koşsa bile video bu hızda akıcı oynar.
        self.max_fps = float(os.environ.get("VIDEO_YOLO_MAX_FPS", "30"))
        # Oynatma hızı çarpanı (1.0 = gerçek zaman). 1.25 → video %25 hızlı akar.
        # CPU çıkarımı yavaş olsa bile oynatma duvar-saatine senkronlanıp gerektiğinde
        # kare atlanır; böylece hız çarpanı çıkarım hızından bağımsız korunur.
        self.speed = float(os.environ.get("VIDEO_YOLO_SPEED", "1.25"))

        # --- Asenkron çıkarım için paylaşılan durum ---
        # Çıkarım artık gösterim döngüsünü BLOKLAMAZ: oynatma thread'i en son
        # kareyi _pending_frame'e bırakır, işçi thread onu alıp YOLO'yu koşturur
        # ve sonucu _last_dets'e yazar. Oynatma her karede en son bilinen
        # kutuları çizer (birkaç yüz ms bayat olabilir ama video akıcı akar).
        self._infer_lock = threading.Lock()
        self._infer_evt = threading.Event()
        self._pending_frame = None    # işçinin işleyeceği en güncel kare
        self._pending_gray = None     # o karenin küçük gri kopyası (hareket referansı)
        self._last_dets = []          # işçinin ürettiği en son tespit listesi
        self._dets_seq = 0            # yeni sonuç sayacı (emit tetiklemek için)
        self._last_gray = None        # tespitlerin ait olduğu karenin gri kopyası
        # Kutular çıkarımın bittiği ANA ait; ekranda ise daha yeni bir kare var.
        # Aradaki kamera kaymasını ölçüp kutuları kaydırmak için küçük gri
        # görüntüler üzerinde faz korelasyonu kullanılır (kare başına ~0.2 ms).
        self.motion_comp = os.environ.get("VIDEO_YOLO_MOTION_COMP", "1") == "1"
        self._mc_w, self._mc_h = 160, 90
        self._mc_win = cv2.createHanningWindow((self._mc_w, self._mc_h), cv2.CV_32F)

        # Arayüze gönderilip henüz ekrana çizilmemiş kare sayısı. GUI thread'i
        # (Gazebo, harita, tablolar) yoğunken 30 FPS kare yayınlamak Qt kuyruğunu
        # şişiriyor ve video giderek GERİDEN geliyordu. Kuyrukta bu kadar kare
        # varken yeni kare atılır → ekranda hep EN GÜNCEL kare olur.
        self._inflight = 0
        self.max_inflight = int(os.environ.get("VIDEO_YOLO_MAX_INFLIGHT", "2"))
        # Gözcü: sayacı düşüren taraf (frame_consumed) herhangi bir nedenle
        # çağrılmazsa akış temelli kilitlenmesin diye, bu süre boyunca kare
        # yayınlanamadıysa sayaç sıfırlanır ve yayına devam edilir.
        self.inflight_reset_sec = float(os.environ.get("VIDEO_YOLO_INFLIGHT_RESET", "0.5"))
        self._stall_since = None

    def frame_consumed(self):
        """Ana pencere kareyi ekrana bastığında çağırır (geri-basınç sayacı)."""
        self._inflight = max(0, self._inflight - 1)

    def _gray_small(self, frame):
        """Hareket ölçümü için küçük (160x90) float32 gri kopya."""
        small = cv2.resize(frame, (self._mc_w, self._mc_h), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)

    def _motion_shift(self, ref_gray, cur_gray, frame_w, frame_h):
        """Tespit karesi ile şu anki kare arasındaki genel (kamera) kaymasını
        piksel cinsinden döner. Faz korelasyonu yalnızca ötelemeyi ölçer; hava
        aracı görüntüsünde baskın hareket kamera kaydırması olduğu için kutuları
        cisimlerin üstüne oturtmaya yeter. Güven düşükse veya kayma saçma
        büyüklükteyse (sahne kesmesi) telafi uygulanmaz."""
        if ref_gray is None or cur_gray is None:
            return 0.0, 0.0
        try:
            (dx, dy), response = cv2.phaseCorrelate(ref_gray, cur_gray, self._mc_win)
        except Exception:
            return 0.0, 0.0
        if response < 0.15:
            return 0.0, 0.0
        sx = dx * frame_w / self._mc_w
        sy = dy * frame_h / self._mc_h
        if abs(sx) > frame_w * 0.4 or abs(sy) > frame_h * 0.4:
            return 0.0, 0.0
        return sx, sy

    def _inference_worker(self, model, device, half):
        """Ayrı (Python) thread: en son bırakılan kare üzerinde YOLO çalıştırır.
        Kuyruk tutmaz — kare biriktirmek yerine hep EN GÜNCEL kareyi işler, böylece
        kutular videonun gerisinde kalmaz. Buradan Qt sinyali yayılmaz (log hariç);
        sonuçlar paylaşılan alana yazılır, oynatma thread'i yayınlar."""
        while self.running:
            if not self._infer_evt.wait(0.1):
                continue
            self._infer_evt.clear()
            with self._infer_lock:
                frame = self._pending_frame
                gray = self._pending_gray
                self._pending_frame = None
            if frame is None:
                continue
            try:
                # persist=True ile ByteTrack takip ID'leri kareler arası korunur.
                results = model.track(
                    frame, persist=True, conf=self.conf, imgsz=self.imgsz,
                    device=device, half=half, verbose=False,
                    tracker="bytetrack.yaml",
                )
                dets = self._extract_dets(results[0])
            except Exception as e:
                self.log_signal.emit(f"Video çıkarım hatası: {e}", "WARN")
                continue
            with self._infer_lock:
                self._last_dets = dets
                # Kutuların ait olduğu kareyi de sakla: oynatma bunu şu anki
                # kareyle karşılaştırıp kutuları kayma kadar öteler.
                self._last_gray = gray
                self._dets_seq += 1

    @staticmethod
    def _shift_dets(dets, sx, sy, frame_w, frame_h):
        """Tespit kutularını (sx, sy) kadar öteleyen KOPYA liste döner. Orijinal
        liste değiştirilmez: o liste ana pencereye (tablo/Copilot) gönderilen
        ham çıkarım sonucudur, yalnızca ÇİZİM telafi edilir."""
        out = []
        for d in dets:
            x1, y1, x2, y2 = d["bbox"]
            nx1 = max(0.0, min(frame_w - 1.0, x1 + sx))
            nx2 = max(0.0, min(frame_w - 1.0, x2 + sx))
            ny1 = max(0.0, min(frame_h - 1.0, y1 + sy))
            ny2 = max(0.0, min(frame_h - 1.0, y2 + sy))
            if nx2 - nx1 < 2 or ny2 - ny1 < 2:
                continue   # kare dışına tamamen çıkmış kutuyu çizme
            nd = dict(d)
            nd["bbox"] = [nx1, ny1, nx2, ny2]
            out.append(nd)
        return out

    @staticmethod
    def _cls_color(name):
        """Sınıf adından deterministik BGR renk (aynı sınıf hep aynı renkte)."""
        h = 0
        for ch in name:
            h = (h * 31 + ord(ch)) & 0xFFFFFF
        return (60 + h % 190, 60 + (h >> 8) % 190, 60 + (h >> 16) % 190)

    def _draw_dets(self, frame, dets):
        """Tespit kutularını + etiketlerini (sınıf, güven, takip ID) kareye çizer.
        Ultralytics'in results.plot() çağrısının yerini alır: segmentasyon maskesi
        harmanlaması ve sonuç nesnesi kopyalaması olmadığı için kare başına maliyet
        ~1 ms'dir (plot() CPU'da 30-60 ms). Kutu görünümü korunur."""
        for d in dets:
            x1, y1, x2, y2 = (int(v) for v in d["bbox"])
            color = self._cls_color(d["cls"])
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            tid = d.get("track_id")
            label = f"{d['cls']} {d['conf']:.2f}" + (f" #{tid}" if tid is not None else "")
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = max(th + 4, y1)
            cv2.rectangle(frame, (x1, ty - th - 4), (x1 + tw + 4, ty), color, -1)
            cv2.putText(frame, label, (x1 + 2, ty - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return frame

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

        # Çıkarım işçisini başlat: model yüklendikten SONRA, video döngüsüyle
        # paralel koşar. daemon=True → uygulama kapanırken asılı kalmaz.
        worker = threading.Thread(
            target=self._inference_worker, args=(model, device, half),
            name="VideoYoloInfer", daemon=True,
        )
        self._worker = worker
        worker.start()

        min_interval = (1.0 / self.max_fps) if self.max_fps > 0 else 0.0
        # Kaynak videonun kendi kare hızı — hız çarpanı bunun üstüne uygulanır.
        src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        first = True
        play_start = time.time()   # oynatma zaman referansı (duvar saati)
        frame_pos = 0.0            # şu anki kaynak kare indeksi
        last_seq = -1              # ana pencereye en son yayınlanan sonuç no'su
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

            # Hareket telafisi için bu karenin küçük gri kopyası (~0.2 ms).
            cur_gray = self._gray_small(frame) if self.motion_comp else None

            # --- Kareyi çıkarım işçisine devret (bloklamadan) ---
            # İşçi meşgulse kare basitçe üzerine yazılır: her zaman EN GÜNCEL
            # kare işlenir, kuyruk birikmez. Bu yüzden gösterim hiç beklemez.
            with self._infer_lock:
                self._pending_frame = frame
                self._pending_gray = cur_gray
                dets = self._last_dets
                ref_gray = self._last_gray
                seq = self._dets_seq
            self._infer_evt.set()

            # Yeni bir çıkarım sonucu geldiyse ana pencereye bir kez yayınla
            # (tespit tablosu / sahne analizi / Copilot snapshot).
            if seq != last_seq:
                last_seq = seq
                self.detections_ready.emit({"detections": dets})

            # --- Geri-basınç: arayüz geride kaldıysa bu kareyi ATLA ---
            # Kuyrukta zaten çizilmemiş kare varken yenisini göndermek gecikmeyi
            # büyütür (video "geç gelir"). Karenin çizimi/dönüşümü de yapılmaz,
            # ama kare çıkarım işçisine YUKARIDA zaten verildi: tespit hızı düşmez.
            if self._inflight >= self.max_inflight and not first:
                if self._stall_since is None:
                    self._stall_since = time.time()
                if time.time() - self._stall_since < self.inflight_reset_sec:
                    dt = time.time() - t0
                    if min_interval > dt:
                        time.sleep(min_interval - dt)
                    continue
                # Gözcü devrede: sayaç takılı kalmış, sıfırla ve yayına devam et.
                self._inflight = 0
            self._stall_since = None

            # Kutular çıkarımın yapıldığı (daha eski) kareye ait. CPU'da çıkarım
            # saniyede 1-3 kez koştuğu için kamera bu arada kayıyor ve kutular
            # cisimlerin gerisinde kalıyordu. Aradaki kaymayı ölçüp kutuları
            # ötele → kutular cisimlerin ÜSTÜNDE durur.
            if self.motion_comp and dets:
                h_f, w_f = frame.shape[:2]
                sx, sy = self._motion_shift(ref_gray, cur_gray, w_f, h_f)
                if sx or sy:
                    dets = self._shift_dets(dets, sx, sy, w_f, h_f)

            # Her karede en son bilinen kutular çizilir → video akıcı, kutular yerinde.
            # Çizim KOPYA üzerine yapılır: aynı tampon işçide çıkarıma girdiği için
            # üstüne çizersek model kendi kutularını görüntünün parçası sanabilir.
            annotated = self._draw_dets(frame.copy(), dets)

            # OpenCV BGR -> Qt RGB. .copy() ile numpy tamponundan bağımsız kopya
            # alınır (CameraStreamThread ile aynı desen).
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            q_img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
            self._inflight += 1
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
        # Çıkarım işçisi olay beklerken uyuyor olabilir; uyandır ki bayrağı görüp
        # en geç 0.1 sn içinde çıksın (daemon olduğu için ayrıca join gerekmez).
        self._infer_evt.set()
        self.wait()
        # İşçi torch içindeyken yorumlayıcı kapanırsa süreç "terminate called"
        # ile çöküyor; kapanmadan önce çıkarımın bitmesini kısa süre bekle.
        worker = getattr(self, "_worker", None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=3.0)
            self._worker = None
