"""Rota QR kutuları için METİN TAŞIYAN QR dokuları üretir.

Eski qr_code.png rastgele/anlamsız bir desendi; bu script her kutuya benzersiz,
OKUNABİLİR bir metin gömer (cv2.QRCodeEncoder — ek bağımlılık yok). yolo.py
uçuş sırasında cv2.QRCodeDetector ile bu metni çözüp loga basar.

Ölçüm notu (2026-07-17, scratchpad/texture_probe): 15 m irtifa + 640 px alt
kamerada 3 m kutu ~84 px görünür; bu boyuttaki QR, 4x cubic büyütme sonrası
cv2.QRCodeDetector ile güvenilir çözülüyor (60 px'e kadar test edildi).
Metinler kısa ve ASCII tutulur ki QR versiyonu (modül sayısı) küçük kalsın —
modül başına düşen piksel arttıkça havadan çözüm o kadar sağlam olur.

Çalıştır:  python3 models/textures/make_qr_text_textures.py
Çıktı:     models/textures/qr_text_N.png (world dosyası bunlara bağlanır)
"""

import os
import cv2

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Kutu sırası world'deki route_qr_box_N ile birebir eşleşir.
QR_TEXTS = {
    1: "KARGO-01 TIBBI MALZEME",
    2: "KARGO-02 GIDA PAKETI",
    3: "KARGO-03 ILK YARDIM",
    4: "KARGO-04 SU VE ERZAK",
}

# Modül başına piksel + sessiz bölge (quiet zone). Doku yüksek çözünürlüklü
# üretilir; kameradaki küçülme Gazebo/render tarafında olur.
MODULE_PX = 24
QUIET_MODULES = 2


def make_texture(text: str, out_path: str) -> None:
    enc = cv2.QRCodeEncoder_create()
    qr = enc.encode(text)  # küçük ikili matris (ör. 25x25)
    big = cv2.resize(qr, None, fx=MODULE_PX, fy=MODULE_PX,
                     interpolation=cv2.INTER_NEAREST)
    pad = QUIET_MODULES * MODULE_PX
    big = cv2.copyMakeBorder(big, pad, pad, pad, pad,
                             cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(out_path, big)

    # Kendi kendini doğrula: üretilen doku geri okunabilmeli.
    decoded, _, _ = cv2.QRCodeDetector().detectAndDecode(cv2.imread(out_path))
    status = "OK" if decoded == text else f"HATA (okunan: {decoded!r})"
    print(f"{os.path.basename(out_path)}: {qr.shape[0]}x{qr.shape[1]} modül, "
          f"{big.shape[1]}px  '{text}'  dogrulama={status}")


if __name__ == "__main__":
    for idx, text in QR_TEXTS.items():
        make_texture(text, os.path.join(BASE_DIR, f"qr_text_{idx}.png"))
