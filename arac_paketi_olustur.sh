#!/bin/bash
# ============================================================
#  TALOS ARAÇ PAKETİ OLUŞTURUCU
#  Üç artefakt + doküman kopyası üretir (hedef: ~/talos-sim/arac_paketi/):
#
#  1) talos-all-imaj_<tarih>.tar.gz    — RUNTIME İMAJ (mount modeli; araçta kalır)
#     docker save talos-all:latest. KOD YOKTUR; kod bind-mount ile bilgisayardan
#     gelir (docker-compose.yml). Kod değişikliği = docker compose restart.
#
#  2) talos-yedek-imaj_<tarih>.tar.gz  — YEDEK İMAJ (kendi kendine yeten)
#     talos-all + KOD + devel + missions + maps GÖMÜLÜ (Dockerfile.yedek).
#     Host'ta kod ağacı GEREKMEZ: sorun çıkarsa docker load +
#     docker-compose.yedek.yml ile mount'suz ayağa kalkar.
#
#  3) talos-eskisurucu-imaj_<tarih>.tar.gz — ESKİ-SÜRÜCÜ YEDEK İMAJI
#     (2) ile aynı model (kod gömülü, mount'suz) ama torch CUDA 11.8 —
#     NVIDIA sürücüsü >= 450.80 yeter (cu121 imajları >= 525 ister). Araç
#     laptopunun sürücüsü eskiyse: docker load + retag (Dockerfile.eskisurucu).
#
#  4) talos-kod_<tarih>.tar.gz         — KOD ANLIK GÖRÜNTÜSÜ (küçük)
#     ~/talos-sim ağacının kodu (talos26_ws git dahil, slalom, devel, src
#     modelsiz, doc). Yeni makineye (laptop) mount modelini kurmak veya tek
#     dosya geri almak için.
#
#  dokuman/ — saha dokümanlarının kopyası (paket, laptopa taşınırken
#  dokümanlar da yanında gitsin diye).
#
#  Kullanım:
#    ./arac_paketi_olustur.sh                       # hepsini üret
#    ./arac_paketi_olustur.sh --sadece-imaj         # yalnız (1)
#    ./arac_paketi_olustur.sh --sadece-yedek        # yalnız (2)
#    ./arac_paketi_olustur.sh --sadece-eskisurucu   # yalnız (3)
#    ./arac_paketi_olustur.sh --sadece-kod          # yalnız (4)
#
#  NOT: docker save, daemon tarafında /var/lib/docker/tmp'de imaj boyu kadar
#  geçici alan kullanır — imaj başına ~14 GB boş disk gerekir.
#  Rehber: doc/saha_hazirlik/07_arac_paket_ve_mount_rehberi.md
# ============================================================

set -eu -o pipefail   # pipefail ŞART: docker save|gzip zincirinde save hatası yutulursa
                      # boş-ama-geçerli bir gzip "OK" diye sha256'lanıp araca gidebilir

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"          # ~/talos-sim
OUT_DIR="${TALOS_PAKET_DIR:-$PROJECT_ROOT/arac_paketi}"
TARIH="$(date +%Y%m%d)"

IMAJ_TAR="$OUT_DIR/talos-all-imaj_${TARIH}.tar.gz"
YEDEK_TAR="$OUT_DIR/talos-yedek-imaj_${TARIH}.tar.gz"
ESKI_TAR="$OUT_DIR/talos-eskisurucu-imaj_${TARIH}.tar.gz"
KOD_TAR="$OUT_DIR/talos-kod_${TARIH}.tar.gz"

YAP_IMAJ=1; YAP_YEDEK=1; YAP_ESKI=1; YAP_KOD=1
case "${1:-}" in
    --sadece-imaj)       YAP_YEDEK=0; YAP_ESKI=0; YAP_KOD=0 ;;
    --sadece-yedek)      YAP_IMAJ=0;  YAP_ESKI=0; YAP_KOD=0 ;;
    --sadece-eskisurucu) YAP_IMAJ=0;  YAP_YEDEK=0; YAP_KOD=0 ;;
    --sadece-kod)        YAP_IMAJ=0;  YAP_YEDEK=0; YAP_ESKI=0 ;;
    "") ;;
    *) echo "Bilinmeyen argüman: $1 (--sadece-imaj | --sadece-yedek | --sadece-eskisurucu | --sadece-kod)"; exit 1 ;;
esac

mkdir -p "$OUT_DIR"

if command -v pigz >/dev/null 2>&1; then GZ="pigz"; else GZ="gzip"; fi

_bos_dogrula() {  # $1=dizin  $2=gereken GB
    local bos
    bos=$(df -BG --output=avail "$1" | tail -1 | tr -dc '0-9')
    if [ "$bos" -lt "$2" ]; then
        echo "HATA: $1 üzerinde ${bos} GB boş yer var; bu adım için en az $2 GB gerekli." >&2
        exit 1
    fi
}

disk_kontrol() {  # $1 = gereken GB
    # Üç ayrı dolum noktası var ve farklı bölümlerde olabilirler:
    #  - OUT_DIR: tar çıktısı
    #  - DockerRootDir (tipik /var/lib/docker): docker save imaj boyu kadar tmp kullanır
    #  - TMPDIR (/tmp): yedek/eskisurucu staging'i (~3 GB yeter)
    # Yalnız OUT_DIR'ı ölçmek bu makinede işe yarar görünür (hepsi aynı bölüm) ama
    # farklı bölümlü bir makinede save ortasında ENOSPC'ye döner.
    local docker_kok
    docker_kok=$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo /var/lib/docker)
    _bos_dogrula "$OUT_DIR" "$1"
    [ -d "$docker_kok" ] && _bos_dogrula "$docker_kok" "$1"
    _bos_dogrula "${TMPDIR:-/tmp}" 3
}

imaj_kaydet() {  # $1=imaj adı  $2=çıktı dosyası
    echo "      docker save $1 → $2 ($GZ, birkaç dakika)..."
    docker save "$1" | $GZ -1 > "$2.kismi" \
        || { rm -f "$2.kismi"; echo "HATA: docker save/$GZ başarısız ($1) — yarım dosya silindi." >&2; exit 1; }
    mv "$2.kismi" "$2"
    $GZ -t "$2"
    echo "      OK: $(du -h "$2" | cut -f1)"
}

# ------------------------------------------------------------
# 1) RUNTIME İMAJ TARI (mount modeli)
# ------------------------------------------------------------
if [ "$YAP_IMAJ" = "1" ]; then
    docker image inspect talos-all:latest >/dev/null 2>&1 || {
        echo "HATA: talos-all:latest yok. Önce: docker build -t talos-all:latest -f Dockerfile.all ." >&2; exit 1; }
    IMAJ_ID=$(docker image inspect talos-all:latest --format '{{.Id}}')
    disk_kontrol 14
    echo "[1/4] Runtime imaj tarı..."
    imaj_kaydet talos-all:latest "$IMAJ_TAR"
fi

# ------------------------------------------------------------
# 2+3) YEDEK ve ESKİ-SÜRÜCÜ İMAJ TARLARI (kod gömülü, mount'suz)
#      Ortak staging: kod (logs/.git/__pycache__ hariç) + devel → build context
# ------------------------------------------------------------
gomulu_imaj_dogrula() {  # $1=imaj adı  — Dockerfile.yedek/eskisurucu ortak içerik kontrolü
    docker run --rm "$1" bash -c "
        source /opt/ros/noetic/setup.bash &&
        PYTHONPATH=/talos-devel/lib/python3/dist-packages:\$PYTHONPATH python3 -c 'import cart_sim.msg, smart_can_msgs.msg' &&
        test -f /talos-kod/control/control.py &&
        test -f /talos-kod/docker-compose.yedek.yml &&
        test -f /missions/gorev_gercek.geojson &&
        test -f /maps/my_map.yaml &&
        test -f /root/catkin_ws/src/yolov8_ros/scripts/best.pt &&
        echo GOMULU_IMAJ_DOGRULANDI"
}

if [ "$YAP_YEDEK" = "1" ] || [ "$YAP_ESKI" = "1" ]; then
    docker image inspect talos-all:latest >/dev/null 2>&1 || {
        echo "HATA: yedek/eskisurucu imajları talos-all:latest üzerine kurulur — önce onu build et." >&2; exit 1; }
    STAGE=$(mktemp -d)
    trap 'rm -rf "$STAGE"' EXIT
    mkdir -p "$STAGE"
    tar -C "$PROJECT_ROOT/scripts" -cf - \
        --exclude='talos26_ws/logs' \
        --exclude='talos26_ws/control/logs' \
        --exclude='talos26_ws/hedef/logs' \
        --exclude='talos26_ws/.git' \
        --exclude='__pycache__' --exclude='*.pyc' \
        talos26_ws | tar -C "$STAGE" -xf -
    cp -a "$PROJECT_ROOT/devel" "$STAGE/devel"
fi

if [ "$YAP_YEDEK" = "1" ]; then
    disk_kontrol 15
    echo "[2/4] Yedek imaj build (kod gömülü)..."
    docker build -t talos-yedek:latest -f "$SCRIPT_DIR/Dockerfile.yedek" "$STAGE" \
        || { echo "HATA: talos-yedek build başarısız" >&2; exit 1; }
    YEDEK_ID=$(docker image inspect talos-yedek:latest --format '{{.Id}}')
    echo "      imaj-içi doğrulama..."
    gomulu_imaj_dogrula talos-yedek:latest \
        || { echo "HATA: yedek imaj doğrulaması başarısız" >&2; exit 1; }
    imaj_kaydet talos-yedek:latest "$YEDEK_TAR"
fi

if [ "$YAP_ESKI" = "1" ]; then
    disk_kontrol 25   # cu118 katmanı imajı ~15 GB'a büyütür; save tmp + tar payı
    echo "[3/4] Eski-sürücü imaj build (kod gömülü + torch cu118 — ilk seferde ~2.7 GB pip indirmesi)..."
    docker build -t talos-eskisurucu:latest -f "$SCRIPT_DIR/Dockerfile.eskisurucu" "$STAGE" \
        || { echo "HATA: talos-eskisurucu build başarısız" >&2; exit 1; }
    ESKI_ID=$(docker image inspect talos-eskisurucu:latest --format '{{.Id}}')
    echo "      imaj-içi doğrulama (içerik + torch cu118)..."
    gomulu_imaj_dogrula talos-eskisurucu:latest \
        || { echo "HATA: eskisurucu imaj doğrulaması başarısız" >&2; exit 1; }
    docker run --rm talos-eskisurucu:latest python3 -c \
        "import torch; assert torch.version.cuda == '11.8', 'torch CUDA %s != 11.8' % torch.version.cuda" \
        || { echo "HATA: eskisurucu imajında torch cu118 değil" >&2; exit 1; }
    if command -v nvidia-smi >/dev/null 2>&1; then
        docker run --rm --gpus all talos-eskisurucu:latest python3 -c \
            "import torch; assert torch.cuda.is_available(), 'GPU gorulemedi'" \
            && echo "      GPU erişimi doğrulandı (cu118, bu makinenin sürücüsüyle)" \
            || { echo "HATA: eskisurucu imajı bu makinede GPU göremedi" >&2; exit 1; }
    else
        echo "      UYARI: bu makinede nvidia-smi yok — GPU erişim testi atlandı"
    fi
    imaj_kaydet talos-eskisurucu:latest "$ESKI_TAR"
fi

if [ "$YAP_YEDEK" = "1" ] || [ "$YAP_ESKI" = "1" ]; then
    rm -rf "$STAGE"; trap - EXIT
fi

# ------------------------------------------------------------
# 4) KOD ANLIK GÖRÜNTÜSÜ (~/talos-sim köküne GÖRELİ — symlink'ler korunur)
# ------------------------------------------------------------
if [ "$YAP_KOD" = "1" ]; then
    echo "[4/4] Kod anlık görüntüsü → $KOD_TAR ..."
    tar -C "$PROJECT_ROOT" -czf "$KOD_TAR.kismi" \
        --exclude='scripts/talos26_ws/logs' \
        --exclude='scripts/talos26_ws/control/logs' \
        --exclude='scripts/talos26_ws/hedef/logs' \
        --exclude='scripts/slalom/video' \
        --exclude='src/cart_sim/models' \
        --exclude='__pycache__' \
        --exclude='*.pyc' \
        scripts/talos26_ws \
        scripts/slalom \
        $(cd "$PROJECT_ROOT" && ls scripts/*.py scripts/*.sh 2>/dev/null) \
        devel \
        src \
        doc \
        CLAUDE.md 2>/dev/null || {
            rc=$?
            [ "$rc" -ge 2 ] && { echo "HATA: kod tarı oluşturulamadı (tar rc=$rc)" >&2; rm -f "$KOD_TAR.kismi"; exit "$rc"; }
        }
    mv "$KOD_TAR.kismi" "$KOD_TAR"
    tar -tzf "$KOD_TAR" >/dev/null
    echo "      OK: $(du -h "$KOD_TAR" | cut -f1)  ($(tar -tzf "$KOD_TAR" | wc -l) dosya)"
fi

# ------------------------------------------------------------
# DOKÜMAN KOPYASI (paket laptopa taşınırken dokümanlar yanında gitsin)
# ------------------------------------------------------------
mkdir -p "$OUT_DIR/dokuman"
cp "$PROJECT_ROOT"/doc/saha_hazirlik/*.md "$OUT_DIR/dokuman/" 2>/dev/null || true
cp "$SCRIPT_DIR/README.md" "$OUT_DIR/dokuman/talos26_ws_README.md" 2>/dev/null || true
cp "$SCRIPT_DIR/KURULUM_VE_SORUN_GIDERME.md" "$OUT_DIR/dokuman/" 2>/dev/null || true

# ------------------------------------------------------------
# MANIFEST + SHA256 (provenance)
# ------------------------------------------------------------
STACK_SHA=$(git -C "$SCRIPT_DIR" rev-parse --short HEAD 2>/dev/null || echo "no-git")
STACK_BRANCH=$(git -C "$SCRIPT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "?")
STACK_KIRLI=$(git -C "$SCRIPT_DIR" status --porcelain 2>/dev/null | wc -l || echo "?")
SLALOM_SHA=$(git -C "$PROJECT_ROOT/scripts/slalom" rev-parse --short HEAD 2>/dev/null || echo "no-git")
{
    echo "olusturma_tarihi: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)  kullanici: ${USER:-?}"
    echo "talos26_ws: $STACK_BRANCH @ $STACK_SHA (uncommitted dosya: $STACK_KIRLI)"
    echo "slalom: $SLALOM_SHA"
    [ "$YAP_IMAJ" = "1" ]  && echo "talos-all imaj id: ${IMAJ_ID:-?}"
    [ "$YAP_YEDEK" = "1" ] && echo "talos-yedek imaj id: ${YEDEK_ID:-?}"
    [ "$YAP_ESKI" = "1" ]  && echo "talos-eskisurucu imaj id: ${ESKI_ID:-?}"
    echo "not: talos-all imajında KOD YOK (bind-mount); talos-yedek imajında kod GÖMÜLÜ (mount'suz)."
    echo "not: talos-eskisurucu = talos-yedek modeli + torch cu118 — NVIDIA sürücüsü 450.80-524 olan"
    echo "     makine için (cu121 imajları sürücü >= 525 ister). Kullanım: docker load sonrası"
    echo "     'docker tag talos-eskisurucu:latest talos-yedek:latest' + 07 rehber §4 yedek akışı."
    echo "rehber: dokuman/07_arac_paket_ve_mount_rehberi.md"
} > "$OUT_DIR/MANIFEST_${TARIH}.txt"

echo "      sha256 hesaplanıyor..."
(
    cd "$OUT_DIR"
    LISTE=""
    for f in "$(basename "$IMAJ_TAR")" "$(basename "$YEDEK_TAR")" "$(basename "$ESKI_TAR")" "$(basename "$KOD_TAR")"; do
        [ -f "$f" ] && LISTE="$LISTE $f"
    done
    [ -n "$LISTE" ] && sha256sum $LISTE > "SHA256SUMS_${TARIH}.txt"
)

echo ""
echo "================ PAKET HAZIR ================"
ls -lh "$OUT_DIR" | tail -n +2
echo ""
echo "Laptopa/araca taşı: $OUT_DIR içindeki HER ŞEY (dokuman/ dahil)."
echo "Drive'a yükle:  ./arac_paketi_drive_yukle.sh   (yalnız yeni/değişen dosyalar gider, md5 doğrulamalı)"
echo "İmaj yükleme:  docker load -i talos-all-imaj_${TARIH}.tar.gz   (normal, mount modeli)"
echo "Acil durum:    docker load -i talos-yedek-imaj_${TARIH}.tar.gz (mount'suz — rehber §4)"
echo "Eski sürücü:   docker load -i talos-eskisurucu-imaj_${TARIH}.tar.gz && docker tag talos-eskisurucu:latest talos-yedek:latest  (nvidia-smi sürücü < 525 ise — rehber §4)"
