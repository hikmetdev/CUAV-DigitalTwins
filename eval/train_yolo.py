#!/usr/bin/env python3
"""
Gazebo Domain-Specific YOLO Fine-Tuning Eğitici (Faz 2.3)
GTX 1650 / 8 GB RAM bütçesine uygun olarak eğitir.
"""

import os
import sys
import argparse
from ultralytics import YOLO


def train_custom_model(data_yaml: str, model_type: str = "yolo11n.pt", epochs: int = 30, imgsz: int = 512):
    if not os.path.exists(data_yaml):
        print(f"HATA: Veriseti yaml dosyası bulunamadı: {data_yaml}")
        sys.exit(1)

    print("=" * 60)
    print(" GAZEBO ÖZEL MODEL EĞİTİMİ (FINE-TUNING)")
    print("=" * 60)
    print(f"Model Tabanı : {model_type}")
    print(f"Veriseti YAML: {data_yaml}")
    print(f"Epoch Sayısı : {epochs}")
    print(f"İmgsz        : {imgsz}")

    model = YOLO(model_type)
    results = model.train(
        data=data_yaml,
        epochs=epochs,
        imgsz=imgsz,
        batch=8,
        workers=2,
        project="runs/detect",
        name="gazebo_custom_yolo",
        exist_ok=True
    )

    best_weights = os.path.join("runs", "detect", "gazebo_custom_yolo", "weights", "best.pt")
    print("-" * 60)
    print(f"Eğitim tamamlandı! En iyi ağırlıklar: {best_weights}")
    return best_weights


def main():
    parser = argparse.ArgumentParser(description="Faz 2.3 Custom YOLO Trainer")
    parser.add_argument("--data", default="dataset/data.yaml", help="data.yaml dosyasının yolu")
    parser.add_argument("--model", default="yolo11n.pt", help="Başlangıç model ağırlığı (yolo11n.pt / yolov8n.pt)")
    parser.add_argument("--epochs", type=int, default=30, help="Epoch sayısı")
    parser.add_argument("--imgsz", type=int, default=512, help="Eğitim görüntü boyutu")
    args = parser.parse_args()

    train_custom_model(args.data, args.model, args.epochs, args.imgsz)


if __name__ == "__main__":
    main()
