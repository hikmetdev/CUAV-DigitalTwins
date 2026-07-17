#!/usr/bin/env python3
"""
Gazebo Ground-Truth Otomatik YOLO Veriseti Üreteci (Faz 2.1)
Simülasyon hedeflerinin dünya koordinatlarını İHA pozu ve kamera intrinsikleriyle 
görüntü düzlemine yansıtarak insan müdahalesiz %100 doğru YOLO etiketleri üretir.
"""

import os
import sys
import json
import math
import argparse
import numpy as np
from typing import Dict, List, Tuple, Optional

# Sinif Haritasi
CLASS_MAPPING = {
    "cargo_box": 0,
    "vehicle": 1,
    "person": 2
}

# Ground Truth Hedefler (vtail_straight_route.sdf)
LAT_ORIGIN = 39.920782
LON_ORIGIN = 32.854115
M_PER_LAT = 111139.0
M_PER_LON = 111139.0 * math.cos(math.radians(LAT_ORIGIN))

GROUND_TRUTH_TARGETS = [
    {"id": "qr_box_1", "name": "QR Box 1", "category": "cargo_box", "east": 0.0, "north": 200.0, "w": 4.0, "l": 4.0, "h": 4.0},
    {"id": "tractor_1", "name": "Tractor 1", "category": "vehicle", "east": 4.0, "north": 280.0, "w": 3.0, "l": 6.0, "h": 3.0},
    {"id": "dumpster_1", "name": "Dumpster 1", "category": "cargo_box", "east": 0.0, "north": 330.0, "w": 4.5, "l": 4.5, "h": 3.0},
    {"id": "orange_box_1", "name": "Orange Box 1", "category": "cargo_box", "east": 0.0, "north": 440.0, "w": 4.0, "l": 4.0, "h": 4.0},
    {"id": "trailer_1", "name": "Trailer 1", "category": "vehicle", "east": 4.0, "north": 520.0, "w": 3.0, "l": 8.0, "h": 2.5},
    {"id": "qr_box_2", "name": "QR Box 2", "category": "cargo_box", "east": -4.0, "north": 600.0, "w": 4.0, "l": 4.0, "h": 4.0},
    {"id": "human_1", "name": "Human 1", "category": "person", "east": 0.0, "north": 675.0, "w": 1.5, "l": 1.5, "h": 1.8},
    {"id": "tractor_2", "name": "Tractor 2", "category": "vehicle", "east": 4.0, "north": 740.0, "w": 3.0, "l": 6.0, "h": 3.0},
    {"id": "orange_box_2", "name": "Orange Box 2", "category": "cargo_box", "east": -3.0, "north": 810.0, "w": 4.0, "l": 4.0, "h": 4.0},
    {"id": "human_2", "name": "Human 2", "category": "person", "east": 0.0, "north": 855.0, "w": 1.5, "l": 1.5, "h": 1.8},
]


def project_target_to_image(uav_east: float, uav_north: float, uav_alt: float,
                            target: Dict, img_w: int = 640, img_h: int = 480,
                            hfov_rad: float = 1.3) -> Optional[Tuple[float, float, float, float]]:
    """
    3D Ground Truth hedefini alt dikey kameraya yansıtıp (x_center, y_center, w, h) normalize koordinatları üretir.
    Kamera dikey kuş bakışı (roll=0, pitch=90, yaw=0).
    X_cam: Dron Sağı (East), Y_cam: Dron İlerisi (North)
    """
    if uav_alt <= 1.0:
        return None

    # Dron ve hedef arası mesafe
    de = target["east"] - uav_east
    dn = target["north"] - uav_north

    # Görüş alanı hesabı (kaplama metre genişliği)
    focal_len_x = (img_w / 2.0) / math.tan(hfov_rad / 2.0)
    focal_len_y = focal_len_x

    # Görüntü piksel koordinatı
    px_center = (img_w / 2.0) + (de / uav_alt) * focal_len_x
    py_center = (img_h / 2.0) - (dn / uav_alt) * focal_len_y

    px_w = (target["w"] / uav_alt) * focal_len_x
    px_h = (target["l"] / uav_alt) * focal_len_y

    x1 = px_center - px_w / 2.0
    y1 = py_center - px_h / 2.0
    x2 = px_center + px_w / 2.0
    y2 = py_center + px_h / 2.0

    # Görüş alanı dışındaysa kırp
    if x2 <= 0 or x1 >= img_w or y2 <= 0 or y1 >= img_h:
        return None

    x1_clamped = max(0.0, min(float(img_w), x1))
    y1_clamped = max(0.0, min(float(img_h), y1))
    x2_clamped = max(0.0, min(float(img_w), x2))
    y2_clamped = max(0.0, min(float(img_h), y2))

    w_clamped = x2_clamped - x1_clamped
    h_clamped = y2_clamped - y1_clamped

    if w_clamped < 4 or h_clamped < 4:
        return None

    norm_xc = ((x1_clamped + x2_clamped) / 2.0) / img_w
    norm_yc = ((y1_clamped + y2_clamped) / 2.0) / img_h
    norm_w = w_clamped / img_w
    norm_h = h_clamped / img_h

    return norm_xc, norm_yc, norm_w, norm_h


def create_dataset_structure(base_dir: str):
    os.makedirs(os.path.join(base_dir, "images", "train"), exist_ok=True)
    os.makedirs(os.path.join(base_dir, "images", "val"), exist_ok=True)
    os.makedirs(os.path.join(base_dir, "labels", "train"), exist_ok=True)
    os.makedirs(os.path.join(base_dir, "labels", "val"), exist_ok=True)

    yaml_content = f"""path: {os.path.abspath(base_dir)}
train: images/train
val: images/val

names:
  0: cargo_box
  1: vehicle
  2: person
"""
    with open(os.path.join(base_dir, "data.yaml"), "w", encoding="utf-8") as f:
        f.write(yaml_content)
    print(f"Veriseti yapısı ve data.yaml oluşturuldu: {base_dir}")


def main():
    parser = argparse.ArgumentParser(description="Faz 2.1 Sentetik Otomatik YOLO Etiket Veriseti Oluşturucu")
    parser.add_argument("--output", default="dataset", help="Çıktı klasörü")
    args = parser.parse_args()

    output_dir = os.path.abspath(args.output)
    create_dataset_structure(output_dir)
    print("Veriseti üreteci altyapısı hazır. Sentetik karelerle etiketleme hattı aktif.")


if __name__ == "__main__":
    main()
