"""Arayüzde gösterilen sahne özetinin (VLM paneli) SQLite kaydı.

GCS'deki "SAHNE ANALİZİ (CANLI)" kutusunda gösterilen her özet, uçuşa özel bir
SQLite dosyasına da yazılır: database/build_YYYYMMDD_HHMMSS.sqlite — dosya adı o
koşunun log klasörüyle (logs/build_...) birebir aynıdır, böylece log ile
veritabanı eşlenir.

Tasarım notları:
  * Özet panel her 16 sn'de bir ve her yeni tespitte yenilenir ama metin çoğu
    kez AYNIDIR; her yenilemede satır eklemek tabloyu şişirir. Bu yüzden kayıt
    yalnızca içerik DEĞİŞTİĞİNDE atılır (son metnin karması tutulur) — tablo,
    sahne özetinin zaman içindeki evrimini verir.
  * Tüm çağrılar GUI (ana) thread'inden gelir; sqlite3 bağlantısı tek thread
    varsayımıyla açılır.
  * Veritabanı hatası uçuş arayüzünü asla düşürmemeli: record() hatayı loglayıp
    False döner, uygulama akışı sürer.
"""

import os
import sqlite3
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

_SCHEMA = """
CREATE TABLE IF NOT EXISTS scene_summary (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    created_at TEXT NOT NULL,          -- ISO 8601 yerel zaman
    mission_time_s REAL,               -- uçuş başından geçen süre
    source TEXT NOT NULL,              -- 'detection' (yolo.log) | 'placeholder'
    summary TEXT NOT NULL,             -- panelde gösterilen düz metin özet
    message_count INTEGER NOT NULL,    -- özetteki tespit cümlesi sayısı
    uav_lat REAL,
    uav_lon REAL,
    uav_alt_m REAL,
    uav_mode TEXT
);
"""


class SceneDatabase:
    """Tek uçuşa (build) ait sahne özeti veritabanı."""

    def __init__(self, database_dir, build_name):
        os.makedirs(database_dir, exist_ok=True)
        self.path = os.path.join(database_dir, f"{build_name}.sqlite")
        self.conn = sqlite3.connect(self.path)
        self.conn.execute(_SCHEMA)
        self.conn.commit()
        self._last_summary = None

    def record(self, summary, source, telemetry=None, mission_time_s=None,
               message_count=None):
        """Özeti kaydeder; içerik son kayıttan farksızsa atlar.

        Döndürür: True = yeni satır yazıldı, False = değişiklik yok / hata."""
        text = (summary or "").strip()
        if not text or text == self._last_summary:
            return False
        tel = telemetry or {}
        if message_count is None:
            message_count = sum(1 for ln in text.split("\n") if ln.strip())
        try:
            self.conn.execute(
                "INSERT INTO scene_summary (created_at, mission_time_s, source, "
                "summary, message_count, uav_lat, uav_lon, uav_alt_m, uav_mode) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    datetime.now().isoformat(timespec="seconds"),
                    round(mission_time_s, 1) if mission_time_s is not None else None,
                    source,
                    text,
                    message_count,
                    tel.get("lat"),
                    tel.get("lon"),
                    tel.get("alt"),
                    tel.get("mode"),
                ),
            )
            self.conn.commit()
        except sqlite3.Error as exc:
            logger.warning(f"Sahne özeti veritabanına yazılamadı: {exc}")
            return False
        self._last_summary = text
        return True

    def close(self):
        try:
            self.conn.close()
        except sqlite3.Error:
            pass
