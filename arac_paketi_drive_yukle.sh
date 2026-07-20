#!/bin/bash
# ============================================================
#  TALOS ARAÇ PAKETİ → GOOGLE DRIVE YÜKLEYİCİ
#
#  ~/talos-sim/arac_paketi/ içeriğini (tarlar + MANIFEST + SHA256 + dokuman/)
#  Drive'daki TALOS/arac_paketi/ klasörüne yükler. rclone kullanır:
#   - YALNIZ yeni/değişen dosyalar gider (boyut+mtime; aynı olan atlanır)
#   - büyük tarlar parçalı yüklenir, kopan yükleme dosya başında devam eder
#   - yükleme sonrası md5 doğrulaması yapılır (Drive'ın kendi md5'iyle)
#   - hiçbir şey SİLİNMEZ ve paylaşım linki AÇILMAZ (copy, sync değil)
#
#  BİR KERELİK KURULUM (tarayıcı ister, elle yapılır):
#    rclone config create gdrive drive scope=drive.file
#    (scope=drive.file → rclone yalnız KENDİ yüklediği dosyaları görür;
#     Drive'ın geri kalanına erişimi olmaz.)
#
#  Kullanım:
#    ./arac_paketi_drive_yukle.sh              # yükle + doğrula
#    ./arac_paketi_drive_yukle.sh --kontrol    # yükleme YOK; yerel↔Drive farkını göster
#
#  Ortam değişkenleri: TALOS_DRIVE_REMOTE (vars: gdrive),
#    TALOS_DRIVE_KLASOR (vars: TALOS/arac_paketi), TALOS_PAKET_DIR
# ============================================================

set -eu -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PAKET_DIR="${TALOS_PAKET_DIR:-$PROJECT_ROOT/arac_paketi}"
REMOTE="${TALOS_DRIVE_REMOTE:-gdrive}"
HEDEF="${TALOS_DRIVE_KLASOR:-TALOS/arac_paketi}"

# rclone: PATH'te yoksa ~/.local/bin fallback (sudo'suz kurulum yeri)
RCLONE=$(command -v rclone || true)
[ -z "$RCLONE" ] && [ -x "$HOME/.local/bin/rclone" ] && RCLONE="$HOME/.local/bin/rclone"
[ -z "$RCLONE" ] && {
    echo "HATA: rclone yok. Kur: https://rclone.org/install/ (sudo'suz: binary'yi ~/.local/bin'e aç)." >&2
    exit 1
}

"$RCLONE" listremotes | grep -q "^${REMOTE}:" || {
    echo "HATA: '${REMOTE}' remote'u tanımlı değil. BİR KERELİK (tarayıcı açılır):" >&2
    echo "  rclone config create ${REMOTE} drive scope=drive.file" >&2
    exit 1
}

[ -d "$PAKET_DIR" ] || { echo "HATA: paket dizini yok: $PAKET_DIR" >&2; exit 1; }

# --kontrol: yükleme yapmadan yerel↔Drive farkını raporla (md5 tabanlı) ve çık
if [ "${1:-}" = "--kontrol" ]; then
    echo "Yerel ↔ ${REMOTE}:${HEDEF} karşılaştırması (yükleme YOK):"
    "$RCLONE" check "$PAKET_DIR" "${REMOTE}:${HEDEF}" --exclude "*.kismi" --one-way 2>&1 | tail -8
    exit 0
fi

# Kota kontrolü: değişmeyen dosyalar atlanacağından bu ÜST sınırdır; yine de
# Drive'da paket toplamı kadar yer yoksa baştan yüksek sesle uyar.
PAKET_BAYT=$(du -sb --exclude='*.kismi' "$PAKET_DIR" | cut -f1)
BOS_BAYT=$("$RCLONE" about "${REMOTE}:" --json 2>/dev/null \
    | python3 -c "import sys,json; print(json.load(sys.stdin).get('free',-1))" || echo -1)
gb() { python3 -c "print(round($1/2**30,1))"; }
echo "Paket: $(gb "$PAKET_BAYT") GB  |  Drive boş alan: $([ "$BOS_BAYT" -ge 0 ] && gb "$BOS_BAYT" || echo '?') GB"
if [ "$BOS_BAYT" -ge 0 ] && [ "$BOS_BAYT" -lt "$PAKET_BAYT" ]; then
    echo "UYARI: Drive'daki boş alan paket toplamından KÜÇÜK. Değişmeyen dosyalar"
    echo "       atlanacağı için yine de sığabilir; sığmazsa rclone kota hatasıyla durur."
fi

echo "Yükleniyor → ${REMOTE}:${HEDEF} (yalnız yeni/değişen; hiçbir şey silinmez)..."
"$RCLONE" copy "$PAKET_DIR" "${REMOTE}:${HEDEF}" \
    --exclude "*.kismi" \
    --transfers 2 --checkers 4 --drive-chunk-size 128M \
    --progress --stats-one-line

echo "Doğrulanıyor (md5, Drive tarafıyla)..."
"$RCLONE" check "$PAKET_DIR" "${REMOTE}:${HEDEF}" --exclude "*.kismi" --one-way \
    || { echo "HATA: doğrulama BAŞARISIZ — yüklemeyi tekrar çalıştır." >&2; exit 1; }

echo ""
echo "================ DRIVE GÜNCEL ================"
"$RCLONE" lsl "${REMOTE}:${HEDEF}" --exclude "dokuman/**" | sort -k4
echo "Klasör: Drive'da '${HEDEF}' (paylaşım linki bilerek AÇILMADI — gerekirse Drive arayüzünden paylaş)."
