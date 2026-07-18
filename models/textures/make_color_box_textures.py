"""İnsan aktörlerinin yerine konan RENKLİ kutular için doku üretir.

İki kutu, iki ayırt edici renk: KIRMIZI ve MAVİ. Tespit stratejisi world
dosyasındaki ölçüm tablosuna dayanır: nadir'de YOLOE'yi ateşleyen şey düz renk
değil İÇ-DESEN KONTRASTIDIR (düz turuncu kutu 0.05 → görünmez, QR desenli kutu
0.60-0.63). Bu yüzden doku, ölçümle kanıtlanmış QR desenini korur; renk ayrımı
yolo.py tarafında bbox kırpıntısının HSV baskın rengiyle yapılır
(classify_box_colors → red_box / blue_box).

Doku düzeni:
  * Kalın renkli çerçeve (kutunun baskın rengi — HSV sınıflandırıcının ana
    kanıtı; kutu alanının ~%45'i saf renk).
  * Beyaz zemin üzerine KOYU RENKLİ modüllü QR (kırmızı/mavi). Modül rengi
    koyu tutulur ki gri tona çevrilince cv2.QRCodeDetector kontrastı korunsun
    (koyu kırmızı ≈ gri 45, koyu mavi ≈ gri 25) ve YOLOE deseni görmeye devam
    etsin. QR metni kutunun rengini söyler; yolo.py bunu da loga basar.

Çalıştır:  python3 models/textures/make_color_box_textures.py
Çıktı:     models/textures/box_red.png, models/textures/box_blue.png
"""

import os
import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

MODULE_PX = 24
QUIET_MODULES = 2
FRAME_MODULES = 4  # renkli çerçeve kalınlığı (modül cinsinden)

# (dosya adı, QR metni, koyu modül rengi BGR, parlak çerçeve rengi BGR)
BOXES = [
    ("box_red.png",  "KIRMIZI KUTU", (0, 0, 150), (30, 30, 230)),
    ("box_blue.png", "MAVI KUTU",    (150, 0, 0), (230, 80, 20)),
]


def make_texture(text, module_bgr, frame_bgr, out_path):
    enc = cv2.QRCodeEncoder_create()
    qr = enc.encode(text)  # 0=siyah modül, 255=beyaz
    big = cv2.resize(qr, None, fx=MODULE_PX, fy=MODULE_PX,
                     interpolation=cv2.INTER_NEAREST)
    pad = QUIET_MODULES * MODULE_PX
    big = cv2.copyMakeBorder(big, pad, pad, pad, pad,
                             cv2.BORDER_CONSTANT, value=255)

    # Gri QR'ı renklendir: modüller koyu renk, zemin beyaz.
    color = np.full((*big.shape, 3), 255, dtype=np.uint8)
    color[big == 0] = module_bgr

    # Kalın renkli çerçeve (baskın renk yüzeyi).
    fr = FRAME_MODULES * MODULE_PX
    color = cv2.copyMakeBorder(color, fr, fr, fr, fr,
                               cv2.BORDER_CONSTANT, value=frame_bgr)
    cv2.imwrite(out_path, color)

    # Kendi kendini doğrula: 1) QR geri okunabilmeli, 2) baskın renk doğru olmalı.
    img = cv2.imread(out_path)
    decoded, _, _ = cv2.QRCodeDetector().detectAndDecode(img)
    qr_ok = decoded == text

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    sat = (s > 70) & (v > 50)
    red_frac = float(np.mean(sat & ((h <= 10) | (h >= 170))))
    blue_frac = float(np.mean(sat & (h >= 100) & (h <= 130)))
    print(f"{os.path.basename(out_path)}: {img.shape[1]}px  '{text}'  "
          f"qr_dogrulama={'OK' if qr_ok else 'HATA (okunan: %r)' % decoded}  "
          f"red={red_frac:.2f} blue={blue_frac:.2f}")


if __name__ == "__main__":
    for fname, text, module_bgr, frame_bgr in BOXES:
        make_texture(text, module_bgr, frame_bgr, os.path.join(BASE_DIR, fname))
