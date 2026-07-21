# Kurulum — Repo Dışı Gerekli Dosyalar

Kaynak kodun tamamı GitHub'da (`hikmetdev/CUAV-DigitalTwins`). Ancak `.gitignore`
ile bilinçli olarak depoya **girmeyen** bazı dosyalar var: sırlar, büyük binary'ler
ve üretilen çıktılar. Yeni bir makineye klondan sonra bunları elle sağlamanız gerekir.

## 1. Gizli anahtarlar (zorunlu)

| Dosya | Nasıl elde edilir |
|---|---|
| `config/.env` | `cp config/.env.example config/.env` yapıp `GEMINI_API_KEY` değerini doldurun. Anahtar: https://aistudio.google.com/apikey (AIzaSy... formatında). |

Bu dosya olmadan Gemini AI Copilot çalışmaz.

## 2. Model ağırlıkları (zorunlu — büyük binary)

Proje kökünde bulunmalı (`yolo.py` göreli yolla yükler, ultralytics MobileCLIP'i
çalışma dizininden okur):

| Dosya | Boyut | Not |
|---|---|---|
| `yoloe-26s-seg.pt` | ~30 MB | YOLOE segmentasyon ağırlığı |
| `mobileclip2_b.ts` | ~243 MB | MobileCLIP text encoder |

Bunları ekibin paylaşımlı deposundan (ya da mevcut çalışan makineden) kopyalayın.

## 3. Medya (opsiyonel)

| Dosya | Boyut | Not |
|---|---|---|
| `drone.mp4` | ~6 MB | "Canlı Görüntüye Geç" butonu için yerel video kaynağı. Yoksa yalnızca bu özellik devre dışı kalır. |

## 4. Üretilen çıktılar (elle taşınmaz — çalıştıkça oluşur)

Bunlar uygulama çalıştıkça yeniden üretilir, kopyalamaya gerek yok:

- `database/` — uçuş başına SQLite sahne özetleri
- `reports/` — üretilen PDF/metin taktik raporlar
- `logs/` — çalışma logları (ROS dahil)
- `runs/` — YOLO eğitim çıktıları
- `dataset/images/`, `dataset/labels/` — veriseti içeriği (iskelet + `data.yaml` repoda)
- `__pycache__/`, `*.pyc` — Python cache

## Ayarlanabilir ortam değişkenleri (opsiyonel)

Kod, davranışı ortam değişkenleriyle ayarlanabilir kılar (varsayılanlar makul):
`YOLO_CONF_THRESHOLD`, `YOLO_DEVICE`, `YOLO_HALF`, `YOLO_IMGSZ`, `YOLO_MAX_FPS`,
`VIDEO_YOLO_CONF`, `VIDEO_YOLO_IMGSZ`, `VIDEO_YOLO_SPEED`, `ROS_LOG_DIR` vb.
Ayrıntı için `yolo.py` ve `threads/video_yolo_t.py` başlıklarına bakın.
