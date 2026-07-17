"""Sırt çantası dokusunu (backpack.png) üretir.

Düz renkli yüzeyler tepeden bakan kamerada YOLOE için gürültü tabanında kalıyor
(~0.06); model kenar/desen istiyor. Bu betik, tepeden görünen bir sırt çantasını
çizer (yere ön yüzü yukarı bakacak şekilde konmuş): ana gövde, ön cep, çevre
fermuarı, sıkıştırma kayışları ve taşıma sapı. orange_box.png ile aynı mantık —
üretilen PNG world SDF'inde albedo_map olarak bağlanır.

Çalıştırma:  python3 models/textures/make_backpack_texture.py
"""

import os
import cv2
import numpy as np

S = 512  # orange_box.png ile aynı çözünürlük

# BGR — koyu lacivert kumaş + turuncu aksan: yeşil çim üzerinde yüksek kontrast.
FABRIC = (78, 54, 32)
FABRIC_LT = (108, 80, 52)
FABRIC_DK = (52, 36, 20)
ACCENT = (40, 120, 235)
STITCH = (150, 132, 108)
ZIP = (190, 190, 195)
DARK = (28, 28, 30)


def rounded_rect(img, p1, p2, r, color, thickness=-1):
    """Köşeleri yuvarlatılmış dikdörtgen.

    Dolgu modunda dikdörtgen+daire birleşimi yeterli; çerçeve modunda ise daire
    çizmek köşelere tam halka bırakır — kenarlar çizgi, köşeler 90° yay olmalı.
    """
    x1, y1 = p1
    x2, y2 = p2
    if thickness < 0:
        cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), color, -1)
        cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r), color, -1)
        for cx, cy in ((x1 + r, y1 + r), (x2 - r, y1 + r), (x1 + r, y2 - r), (x2 - r, y2 - r)):
            cv2.circle(img, (cx, cy), r, color, -1)
        return
    cv2.line(img, (x1 + r, y1), (x2 - r, y1), color, thickness, cv2.LINE_AA)
    cv2.line(img, (x1 + r, y2), (x2 - r, y2), color, thickness, cv2.LINE_AA)
    cv2.line(img, (x1, y1 + r), (x1, y2 - r), color, thickness, cv2.LINE_AA)
    cv2.line(img, (x2, y1 + r), (x2, y2 - r), color, thickness, cv2.LINE_AA)
    for (cx, cy), ang in (((x1 + r, y1 + r), 180), ((x2 - r, y1 + r), 270),
                          ((x2 - r, y2 - r), 0), ((x1 + r, y2 - r), 90)):
        cv2.ellipse(img, (cx, cy), (r, r), ang, 0, 90, color, thickness, cv2.LINE_AA)


def zipper(img, pts, pull_at=None):
    """Fermuar: metalik hat + dişler; istenirse turuncu çekecek."""
    pts = np.array(pts, dtype=np.int32)
    cv2.polylines(img, [pts], False, ZIP, 3, cv2.LINE_AA)
    for i in range(len(pts) - 1):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        seg = max(int(np.hypot(bx - ax, by - ay)) // 8, 1)
        for t in np.linspace(0, 1, seg):
            x, y = int(ax + (bx - ax) * t), int(ay + (by - ay) * t)
            cv2.circle(img, (x, y), 2, (120, 120, 125), -1)
    if pull_at is not None:
        cv2.circle(img, pull_at, 10, ACCENT, -1)
        cv2.circle(img, pull_at, 10, DARK, 2)


def main():
    # Zemin: çantanın kendisi kutunun üst yüzünü kaplar.
    img = np.full((S, S, 3), FABRIC, dtype=np.uint8)
    noise = np.random.default_rng(7).normal(0, 6, (S, S, 1)).astype(np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # Ana gövde: yuvarlak köşeli silüet + çevre dikişi (kutu köşesini kırar).
    rounded_rect(img, (36, 26), (S - 36, S - 26), 52, FABRIC_LT)
    rounded_rect(img, (36, 26), (S - 36, S - 26), 52, STITCH, 2)
    rounded_rect(img, (52, 42), (S - 52, S - 42), 44, FABRIC_DK, 1)

    # Taşıma sapı: üstte kavisli bant.
    rounded_rect(img, (S // 2 - 52, 12), (S // 2 + 52, 52), 18, FABRIC_DK)
    rounded_rect(img, (S // 2 - 52, 12), (S // 2 + 52, 52), 18, STITCH, 2)

    # Üst kapak fermuarı: gövdenin üst üçte birini çevreleyen U hattı.
    zipper(img, [(64, 168), (64, 96), (150, 60), (S - 150, 60), (S - 64, 96), (S - 64, 168)],
           pull_at=(S - 64, 150))

    # Ön cep: alt-orta, kendi fermuarı ve dikişiyle — çantanın en ayırt edici öğesi.
    rounded_rect(img, (98, 250), (S - 98, S - 62), 34, FABRIC)
    rounded_rect(img, (98, 250), (S - 98, S - 62), 34, STITCH, 2)
    zipper(img, [(118, 276), (S - 118, 276)], pull_at=(S - 128, 276))

    # Cep üzerinde turuncu logo şeridi: net renk kontrastı.
    cv2.rectangle(img, (S // 2 - 44, 372), (S // 2 + 44, 396), ACCENT, -1)
    cv2.rectangle(img, (S // 2 - 44, 372), (S // 2 + 44, 396), DARK, 2)

    # Yan sıkıştırma kayışları: tokalarıyla birlikte ek kenar üretir.
    for y in (196, 452):
        cv2.rectangle(img, (40, y), (S - 40, y + 16), FABRIC_DK, -1)
        cv2.rectangle(img, (40, y), (S - 40, y + 16), DARK, 1)
        for bx in (86, S - 86):
            cv2.rectangle(img, (bx - 14, y - 5), (bx + 14, y + 21), ACCENT, -1)
            cv2.rectangle(img, (bx - 14, y - 5), (bx + 14, y + 21), DARK, 2)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "backpack.png")
    cv2.imwrite(out, img)
    print(f"Yazıldı: {out} ({img.shape[1]}x{img.shape[0]})")


if __name__ == "__main__":
    main()
