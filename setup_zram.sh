#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# zram tabanlı sıkıştırmalı swap kurulumu (8 GB RAM sistemde OOM çökmelerine karşı)
#
#   - Disk KULLANMAZ: RAM'i zstd ile sıkıştırır (~3-4:1), etkin swap ekler.
#   - Kalıcıdır: systemd servisi ile her açılışta otomatik kurulur.
#   - apt/paket gerektirmez: yalnızca çekirdek zram modülü + zramctl + systemd.
#   - Idempotent: tekrar çalıştırılabilir.
#
# Kullanım:   sudo bash setup_zram.sh
# Boyut değiştirme:   sudo ZRAM_SIZE=6G bash setup_zram.sh
# Geri alma:  sudo systemctl disable --now zram-swap.service
# ---------------------------------------------------------------------------
set -euo pipefail

ZRAM_SIZE="${ZRAM_SIZE:-4G}"     # 8 GB RAM için güvenli varsayılan
ZRAM_ALGO="${ZRAM_ALGO:-zstd}"   # zstd: yüksek sıkıştırma, düşük CPU

if [[ ${EUID} -ne 0 ]]; then
  echo "Bu betik root gerektirir. Şöyle çalıştır:  sudo bash $0" >&2
  exit 1
fi

HELPER=/usr/local/sbin/zram-swap.sh
UNIT=/etc/systemd/system/zram-swap.service
SYSCTL=/etc/sysctl.d/99-zram.conf

echo "[1/5] zram çekirdek modülü yükleniyor..."
modprobe zram

echo "[2/5] Yardımcı betik yazılıyor: ${HELPER}"
cat > "${HELPER}" <<'HELPER_EOF'
#!/usr/bin/env bash
# zram swap başlat/durdur yardımcısı (systemd tarafından çağrılır).
set -euo pipefail
STATE=/run/zram-swap.dev
SIZE="${ZRAM_SIZE:-4G}"
ALGO="${ZRAM_ALGO:-zstd}"

case "${1:-}" in
  start)
    modprobe zram || true
    # Algoritma destekleniyorsa onunla, değilse varsayılanla oluştur.
    if DEV=$(zramctl --find --size "${SIZE}" --algorithm "${ALGO}" 2>/dev/null); then :; else
      DEV=$(zramctl --find --size "${SIZE}")
    fi
    mkswap "${DEV}" >/dev/null
    swapon --priority 100 "${DEV}"
    echo "${DEV}" > "${STATE}"
    echo "zram swap etkin: ${DEV} (${SIZE}, ${ALGO})"
    ;;
  stop)
    if [[ -f "${STATE}" ]]; then
      DEV=$(cat "${STATE}")
      swapoff "${DEV}" 2>/dev/null || true
      zramctl --reset "${DEV}" 2>/dev/null || true
      rm -f "${STATE}"
    fi
    ;;
  *)
    echo "kullanım: $0 {start|stop}" >&2; exit 1 ;;
esac
HELPER_EOF
chmod +x "${HELPER}"

echo "[3/5] systemd servisi yazılıyor: ${UNIT}"
cat > "${UNIT}" <<UNIT_EOF
[Unit]
Description=zram sıkıştırmalı swap (8GB RAM OOM koruması)
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
Environment=ZRAM_SIZE=${ZRAM_SIZE}
Environment=ZRAM_ALGO=${ZRAM_ALGO}
ExecStart=${HELPER} start
ExecStop=${HELPER} stop

[Install]
WantedBy=multi-user.target
UNIT_EOF

echo "[4/5] sysctl ayarları (zram için swappiness=100, page-cluster=0)..."
cat > "${SYSCTL}" <<'SYSCTL_EOF'
# zram hızlı olduğu için takasa isteklilik artırılır; page-cluster=0 rastgele erişime uygun.
vm.swappiness=100
vm.page-cluster=0
SYSCTL_EOF
sysctl --quiet -p "${SYSCTL}" || true

echo "[5/5] servis etkinleştiriliyor ve başlatılıyor..."
systemctl daemon-reload
systemctl enable zram-swap.service >/dev/null
# Zaten çalışıyorsa temiz yeniden kurulum.
systemctl restart zram-swap.service

echo
echo "================= SONUÇ ================="
zramctl || true
echo
swapon --show
echo
free -h
echo "========================================="
echo "Tamam. zram swap aktif ve her açılışta otomatik kurulacak."
