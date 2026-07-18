"""Rotadaki QR-dışı cisimlerin dokularını üretir: stop tabelası, dama paneli
ve nişan tahtası.

Cisim seçimi TAHMİNLE DEĞİL ÖLÇÜMLE yapıldı (2026-07-17, yoloe-26s ile 15 m
nadir simülasyonu, ~28 px/m):

    stop tabelası (3.5 m yatay) ....... stop sign:0.74      <- doğru etiket
    dama paneli (3.0 m, 8x8) .......... checkerboard:0.79   <- doğru etiket
    boyalı tepeden otomobil (4.4 m) ... TESPİT YOK (umbrella:0.04 gürültüsü)
    boyalı tepeden kamyon (6.0 m) ..... TESPİT YOK
    plaj şemsiyesi (3.2-4.5 m) ........ 0.09 / stop sign'a karışıyor
    futbol topu deseni (3.0 m) ........ qr code box'a karışıyor
    solar panel (4.0 m) ............... checkerboard'a karışıyor

İkinci tur ölçüm (2026-07-17, 3.2 m panel) — çeşitliliği artırmak için altı
aday denendi, YALNIZCA BİRİ (nişan tahtası) geçti. Adaylar önce ÇİMENDE
ölçüldü, sonra rotanın kendi asfaltı üzerinde, gerçek dünyada, üç ayrı bakış
konumundan DOĞRULANDI; elemelerin çoğu bu ikinci aşamada oldu:

    nişan tahtası (eş merkezli halka) .. çimen 0.635 / asfalt 0.57-0.67 <- ALINDI
    tehlike üçgeni (! işareti) ......... asfaltta 0.30-0.37 ateşliyor AMA
                                         'warning sign' promptu STOP TABELASINI
                                         da 0.37 ile yakalıyor (stop tabelası
                                         zaten bir uyarı levhası). Gerçek üçgen
                                         ile hayalet aynı bantta => hiçbir eşik
                                         ayıramaz. Denenen alternatif promptlar:
                                         'warning triangle' 0.20-0.28 (zayıf),
                                         'hazard sign' (nişan tahtasını 0.21 ile
                                         çalıyor), 'caution sign' (stop'u 0.51),
                                         'yield sign' (+6m'de 0.13'e düşüyor),
                                         'exclamation mark' (stop'u 0.23).
                                         => ELENDI.
    çapraz işaret (turuncu X) .......... çimen 'helicopter landing pad':0.397
                                         ama ASFALTTA 0.063 => ELENDI. Rota
                                         pist üzerinde; çimendeki ölçüm yanıltıcı.
    helipad 'H' ........................ target:0.518  (nişan tahtasıyla AYNI
                                         etikete düşüyor, ayırt edilemez)
    yaya geçidi (zebra) ................ qr code box:0.294 (QR kutuya karışıyor)
    barkod ............................. qr code box:0.300 (QR kutuya karışıyor)

Yani nadir'de yeni cisim eklemenin çalışan yolu, YOLOE'nin tepeden de tanıdığı
yüksek iç-kontrastlı düzlemsel desenler. Elenen adaylardan üç ders:
  1) Desenin güçlü ateşlemesi yetmiyor; ETİKETİ rotadaki başka bir cisimle
     çakışmamalı (zebra/barkod QR kutuyla, 'H' nişan tahtasıyla çakışıyor).
  2) ZEMİN önemli: aynı desen çimende 0.40, asfaltta 0.06 olabiliyor. Aday
     mutlaka duracağı zeminde ölçülmeli.
  3) Prompt kelimesi de ölçüm konusu: nişan tahtası için 'target' hem tahtayı
     (0.61) hem SIRT ÇANTASINI (0.23) yakalıyordu; 'concentric circles' tahtayı
     0.64 bulup çantada 0.02'de kalıyor. Bu yüzden yolo.py 'concentric circles'
     kullanır.

Çalıştır:  python3 models/textures/make_route_object_textures.py
Çıktı:     stop_sign.png, checker_panel.png, bullseye.png
"""

import os
import math

import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def make_stop_sign(s=768):
    """Tepeden stop tabelası: kırmızı sekizgen + beyaz çerçeve + STOP yazısı."""
    img = np.full((s, s, 3), (120, 120, 120), np.uint8)  # asfalt gri zemin
    c = s // 2
    r = s // 2 - 38
    pts = []
    for i in range(8):
        a = math.pi / 8 + i * math.pi / 4
        pts.append([c + int(r * math.cos(a)), c + int(r * math.sin(a))])
    pts = np.array(pts)
    cv2.fillPoly(img, [pts], (30, 30, 210))
    cv2.polylines(img, [pts], True, (240, 240, 240), 24)
    cv2.putText(img, "STOP", (c - 192, c + 58), cv2.FONT_HERSHEY_DUPLEX,
                4.9, (255, 255, 255), 28)
    return img


def make_checker_panel(s=768, n=8):
    """8x8 siyah-beyaz dama deseni paneli."""
    img = np.zeros((s, s, 3), np.uint8)
    q = s // n
    for i in range(n):
        for j in range(n):
            col = (240, 240, 240) if (i + j) % 2 == 0 else (25, 25, 25)
            img[i * q:(i + 1) * q, j * q:(j + 1) * q] = col
    return img


def make_bullseye(s=768, rings=6):
    """Nişan tahtası: kırmızı/beyaz eş merkezli halkalar."""
    img = np.full((s, s, 3), (245, 245, 245), np.uint8)
    step = (s // 2 - 30) // rings
    for i in range(rings):
        r = (s // 2 - 30) - i * step
        col = (40, 40, 220) if i % 2 == 0 else (250, 250, 250)
        cv2.circle(img, (s // 2, s // 2), r, col, -1)
    return img


def make_car_topview(s=768):
    """Kuş bakışı (nadir) otomobil dokusu: gövde, ön/arka camlar, tavan, dikiz aynaları, ön/arka farlar."""
    img = np.full((s, s, 3), (110, 110, 110), np.uint8)  # asfalt gri
    cx, cy = s // 2, s // 2
    w, h = 260, 580  # en x boy
    
    # Yan aynalar
    cv2.rectangle(img, (cx - w // 2 - 25, cy - 140), (cx - w // 2, cy - 90), (40, 40, 40), -1)
    cv2.rectangle(img, (cx + w // 2, cy - 140), (cx + w // 2 + 25, cy - 90), (40, 40, 40), -1)

    # Otomobil ana gövdesi (mavi-gri metalik)
    body_poly = np.array([
        [cx - w // 2 + 30, cy - h // 2],
        [cx + w // 2 - 30, cy - h // 2],
        [cx + w // 2, cy - h // 2 + 60],
        [cx + w // 2, cy + h // 2 - 60],
        [cx + w // 2 - 30, cy + h // 2],
        [cx - w // 2 + 30, cy + h // 2],
        [cx - w // 2, cy + h // 2 - 60],
        [cx - w // 2, cy - h // 2 + 60],
    ])
    cv2.fillPoly(img, [body_poly], (180, 80, 20))  # BGR lacivert-mavi
    cv2.polylines(img, [body_poly], True, (240, 240, 240), 6)

    # Ön kaput ızgarası + ön farlar
    cv2.rectangle(img, (cx - 70, cy - h // 2 + 10), (cx + 70, cy - h // 2 + 35), (30, 30, 30), -1)
    cv2.circle(img, (cx - w // 2 + 40, cy - h // 2 + 25), 20, (255, 255, 240), -1)
    cv2.circle(img, (cx + w // 2 - 40, cy - h // 2 + 25), 20, (255, 255, 240), -1)

    # Ön cam (trapezoid)
    ws_front = np.array([
        [cx - w // 2 + 25, cy - 130],
        [cx + w // 2 - 25, cy - 130],
        [cx + w // 2 - 40, cy - 40],
        [cx - w // 2 + 40, cy - 40],
    ])
    cv2.fillPoly(img, [ws_front], (35, 35, 35))
    cv2.polylines(img, [ws_front], True, (200, 200, 200), 4)

    # Tavan paneli
    roof_poly = np.array([
        [cx - w // 2 + 40, cy - 35],
        [cx + w // 2 - 40, cy - 35],
        [cx + w // 2 - 40, cy + 110],
        [cx - w // 2 + 40, cy + 110],
    ])
    cv2.fillPoly(img, [roof_poly], (150, 60, 15))

    # Arka cam
    ws_rear = np.array([
        [cx - w // 2 + 40, cy + 115],
        [cx + w // 2 - 40, cy + 115],
        [cx + w // 2 - 25, cy + 195],
        [cx - w // 2 + 25, cy + 195],
    ])
    cv2.fillPoly(img, [ws_rear], (35, 35, 35))
    cv2.polylines(img, [ws_rear], True, (200, 200, 200), 4)

    # Arka stop lambaları
    cv2.rectangle(img, (cx - w // 2 + 20, cy + h // 2 - 30), (cx - w // 2 + 70, cy + h // 2 - 10), (30, 30, 230), -1)
    cv2.rectangle(img, (cx + w // 2 - 70, cy + h // 2 - 30), (cx + w // 2 - 20, cy + h // 2 - 10), (30, 30, 230), -1)

    return img


def make_person_topview(s=768):
    """Kuş bakışı (nadir) insan dokusu: omuzlar, sarı reflektörlü yelek, kafa/kask."""
    img = np.full((s, s, 3), (110, 110, 110), np.uint8)  # asfalt gri
    cx, cy = s // 2, s // 2

    # Zemin kontrast zemin pedi (açık renk kare mat)
    cv2.rectangle(img, (cx - 180, cy - 180), (cx + 180, cy + 180), (220, 220, 220), -1)
    cv2.rectangle(img, (cx - 180, cy - 180), (cx + 180, cy + 180), (40, 40, 40), 8)

    # Omuzlar ve gövde (parlak sarı iş güvenliği yeleği)
    shoulders = np.array([
        [cx - 130, cy - 50],
        [cx + 130, cy - 50],
        [cx + 110, cy + 70],
        [cx - 110, cy + 70],
    ])
    cv2.fillPoly(img, [shoulders], (20, 220, 240))  # BGR fosforlu sarı
    cv2.polylines(img, [shoulders], True, (20, 20, 20), 6)

    # Yelek üzerindeki gümüş reflektör şeritler
    cv2.rectangle(img, (cx - 90, cy - 30), (cx - 50, cy + 50), (230, 230, 230), -1)
    cv2.rectangle(img, (cx + 50, cy - 30), (cx + 90, cy + 50), (230, 230, 230), -1)

    # Kafa / Kask (koyu renk daire)
    cv2.circle(img, (cx, cy - 10), 45, (40, 40, 40), -1)
    cv2.circle(img, (cx, cy - 10), 45, (230, 230, 230), 4)

    return img


if __name__ == "__main__":
    for name, img in (("stop_sign.png", make_stop_sign()),
                      ("checker_panel.png", make_checker_panel()),
                      ("bullseye.png", make_bullseye()),
                      ("car_topview.png", make_car_topview()),
                      ("person_topview.png", make_person_topview())):
        path = os.path.join(BASE_DIR, name)
        cv2.imwrite(path, img)
        print(f"{name}: {img.shape[1]}x{img.shape[0]} px yazıldı")

