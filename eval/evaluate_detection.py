#!/usr/bin/env python3
"""
Nesne Tespiti Değerlendirme & Metrik Betiği (Faz 0)
Simülasyon ground-truth hedefleri ile yolo.log verilerini eşleyip Precision, Recall, F1, TP/FP/FN hesaplar.
"""

import os
import sys
import json
import math
import argparse
from typing import Dict, List, Any

# Ground Truth Hedefler (vtail_straight_route.sdf)
# Spherical coordinates origin: (39.920782, 32.854115)
LAT_ORIGIN = 39.920782
LON_ORIGIN = 32.854115

# METERS_PER_DEGREE approximation
M_PER_LAT = 111139.0
M_PER_LON = 111139.0 * math.cos(math.radians(LAT_ORIGIN))

GROUND_TRUTH_TARGETS = [
    {"id": "qr_box_1", "name": "QR Box 1", "category": "cargo_box", "east": 0.0, "north": 200.0, "size": 4.0},
    {"id": "tractor_1", "name": "Tractor 1", "category": "vehicle", "east": 4.0, "north": 280.0, "size": 4.0},
    {"id": "dumpster_1", "name": "Dumpster 1", "category": "cargo_box", "east": 0.0, "north": 330.0, "size": 4.5},
    {"id": "orange_box_1", "name": "Orange Box 1", "category": "cargo_box", "east": 0.0, "north": 440.0, "size": 4.0},
    {"id": "trailer_1", "name": "Trailer 1", "category": "vehicle", "east": 4.0, "north": 520.0, "size": 6.0},
    {"id": "qr_box_2", "name": "QR Box 2", "category": "cargo_box", "east": -4.0, "north": 600.0, "size": 4.0},
    {"id": "human_1", "name": "Human 1", "category": "person", "east": 0.0, "north": 675.0, "size": 2.0},
    {"id": "tractor_2", "name": "Tractor 2", "category": "vehicle", "east": 4.0, "north": 740.0, "size": 4.0},
    {"id": "orange_box_2", "name": "Orange Box 2", "category": "cargo_box", "east": -3.0, "north": 810.0, "size": 4.0},
    {"id": "human_2", "name": "Human 2", "category": "person", "east": 0.0, "north": 855.0, "size": 2.0},
]


def latlon_to_enu(lat: float, lon: float):
    d_lat = lat - LAT_ORIGIN
    d_lon = lon - LON_ORIGIN
    north = d_lat * M_PER_LAT
    east = d_lon * M_PER_LON
    return east, north


def is_target_in_fov(uav_east: float, uav_north: float, alt_m: float, target: Dict[str, Any], hfov_rad: float = 1.3) -> bool:
    """Alt kamera (kuş bakışı) FOV hesabı."""
    if alt_m is None or alt_m <= 0:
        alt_m = 15.0  # varsayılan seyir irtifası
    
    # hfov_rad 1.3 rad (~74.5 deg). Kamera 4:3 ise vfov ~ 1.0 rad.
    # Kaplama genişliği W = 2 * alt * tan(hfov / 2)
    half_w = alt_m * math.tan(hfov_rad / 2.0)
    half_h = half_w * 0.75  # 4:3 aspect ratio

    de = abs(uav_east - target["east"])
    dn = abs(uav_north - target["north"])
    
    margin = target["size"] / 2.0
    return (de <= half_w + margin) and (dn <= half_h + margin)


def evaluate_log_file(log_path: str) -> Dict[str, Any]:
    if not os.path.exists(log_path):
        raise FileNotFoundError(f"Log dosyası bulunamadı: {log_path}")

    status_events = []
    detection_events = []
    metrics_events = []
    startup_event = None

    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                data = json.loads(line)
                ev = data.get("event_type")
                if ev == "startup":
                    startup_event = data
                elif ev == "status":
                    status_events.append(data)
                elif ev == "object_detection":
                    detection_events.append(data)
                elif ev == "metrics":
                    metrics_events.append(data)
            except Exception:
                pass

    # Ground truth hedef bazında görülme & tespit takibi
    target_fov_hits = {t["id"]: 0 for t in GROUND_TRUTH_TARGETS}
    target_detections = {t["id"]: [] for t in GROUND_TRUTH_TARGETS}
    
    tp_count = 0
    fp_count = 0
    confidences = []
    per_class = {"cargo_box": {"tp": 0, "fp": 0, "fn": 0, "conf_sum": 0.0},
                 "vehicle": {"tp": 0, "fp": 0, "fn": 0, "conf_sum": 0.0},
                 "person": {"tp": 0, "fp": 0, "fn": 0, "conf_sum": 0.0}}

    # Status loglarından dron pozisyonunu izleyip hangi hedeflerin FOV'a girdiğini bul
    for det in detection_events:
        lat = det.get("latitude")
        lon = det.get("longitude")
        alt = det.get("uav_altitude_m", 15.0)
        obj = det.get("object", {})
        category = obj.get("normalized_category", "unknown")
        conf = obj.get("confidence", 0.0)
        confidences.append(conf)

        matched_target = None
        if lat is not None and lon is not None:
            uav_e, uav_n = latlon_to_enu(lat, lon)
            
            # FOV içindeki ground truth hedeflerle eşleştir
            for t in GROUND_TRUTH_TARGETS:
                if is_target_in_fov(uav_e, uav_n, alt, t):
                    target_fov_hits[t["id"]] += 1
                    if t["category"] == category:
                        matched_target = t
                        break

        if matched_target:
            tp_count += 1
            target_detections[matched_target["id"]].append(det)
            if category in per_class:
                per_class[category]["tp"] += 1
                per_class[category]["conf_sum"] += conf
        else:
            fp_count += 1
            if category in per_class:
                per_class[category]["fp"] += 1

    # FN hesabı: Dronun FOV'undan en az 1 kez geçen ama hiç doğru tespit edilemeyen ground-truth hedefleri
    # (Veya total görülme potansiyeline göre)
    detected_target_ids = {tid for tid, dets in target_detections.items() if len(dets) > 0}
    fn_count = 0
    for t in GROUND_TRUTH_TARGETS:
        cat = t["category"]
        if t["id"] not in detected_target_ids:
            fn_count += 1
            if cat in per_class:
                per_class[cat]["fn"] += 1

    precision = tp_count / (tp_count + fp_count) if (tp_count + fp_count) > 0 else 0.0
    recall = tp_count / (tp_count + fn_count) if (tp_count + fn_count) > 0 else 0.0
    f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0
    avg_conf = (sum(confidences) / len(confidences)) if confidences else 0.0

    report = {
        "log_path": log_path,
        "startup_config": startup_event,
        "total_detections": len(detection_events),
        "metrics": {
            "precision": round(precision, 4),
            "recall": round(recall, 4),
            "f1_score": round(f1, 4),
            "avg_confidence": round(avg_conf, 4),
            "tp": tp_count,
            "fp": fp_count,
            "fn": fn_count,
        },
        "per_class": per_class,
        "target_breakdown": {
            t["id"]: {
                "name": t["name"],
                "category": t["category"],
                "detected": t["id"] in detected_target_ids,
                "det_count": len(target_detections[t["id"]]),
                "max_conf": max([d["object"]["confidence"] for d in target_detections[t["id"]]], default=0.0)
            }
            for t in GROUND_TRUTH_TARGETS
        }
    }

    return report


def print_report_summary(report: Dict[str, Any]):
    m = report["metrics"]
    print("=" * 60)
    print(" NESNE TESPİTİ PERFORMANS VE ÖLÇÜM RAPORU (FAZ 0)")
    print("=" * 60)
    print(f"Log Dosyası    : {report['log_path']}")
    print(f"Toplam Tespit  : {report['total_detections']}")
    print(f"Precision      : %{m['precision'] * 100:.2f}  (TP: {m['tp']}, FP: {m['fp']})")
    print(f"Recall         : %{m['recall'] * 100:.2f}  (TP: {m['tp']}, FN: {m['fn']})")
    print(f"F1-Score       : {m['f1_score']:.4f}")
    print(f"Ortalama Güven : %{m['avg_confidence'] * 100:.2f}")
    print("-" * 60)
    print("Sınıf Kırılımı:")
    for cls, cdata in report["per_class"].items():
        cp = cdata["tp"] / (cdata["tp"] + cdata["fp"]) if (cdata["tp"] + cdata["fp"]) > 0 else 0.0
        cr = cdata["tp"] / (cdata["tp"] + cdata["fn"]) if (cdata["tp"] + cdata["fn"]) > 0 else 0.0
        print(f" - {cls:10s} | TP: {cdata['tp']:2d} | FP: {cdata['fp']:2d} | FN: {cdata['fn']:2d} | Precision: %{cp*100:5.1f} | Recall: %{cr*100:5.1f}")
    print("-" * 60)
    print("Hedef BAZLI Detay:")
    for tid, tinfo in report["target_breakdown"].items():
        status = "✅ TESPİT EDİLDİ" if tinfo["detected"] else "❌ KAÇIRILDI"
        print(f" - [{tinfo['name']:15s}] ({tinfo['category']:10s}): {status} (Adet: {tinfo['det_count']}, Tepe Güven: %{tinfo['max_conf']*100:.1f})")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Faz 0 Nesne Tespiti Evaluator")
    parser.add_argument("log_path", nargs="?", default=None, help="yolo.log dosyasının yolu")
    args = parser.parse_args()

    log_file = args.log_path
    if not log_file:
        # En son log klasörünü bul
        logs_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
        build_dirs = [os.path.join(logs_dir, d) for d in os.listdir(logs_dir) if d.startswith("build_")]
        if not build_dirs:
            print("HATA: Hiçbir build_ log klasörü bulunamadı.")
            sys.exit(1)
        build_dirs.sort(key=os.path.getmtime, reverse=True)
        log_file = os.path.join(build_dirs[0], "yolo.log")

    report = evaluate_log_file(log_file)
    print_report_summary(report)

    # Raporu json olarak kaydet
    out_dir = os.path.dirname(log_file)
    out_json = os.path.join(out_dir, "eval_report.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"Değerlendirme raporu yazıldı: {out_json}")


if __name__ == "__main__":
    main()
