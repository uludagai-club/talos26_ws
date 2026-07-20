#!/bin/bash
# ============================================================
#  TALOS MANUEL SÜRÜŞ Başlatıcı (direksiyon seti ile)
#  - Otonom sürüş (talos-controller / control.py) HARİÇ tüm sistem ayağa kalkar.
#  - Aracı, bu bilgisayara takılı USB direksiyon setiyle vcan0 üzerinden sürersin.
#  - Diğer tüm modüller (konum, harita, hedef, engel, trafik, şerit, yaya,
#    park, karar, can/state köprüleri, can-visualizer) çalışır -> manuel
#    sürerken algı/karar çıktısını izleyebilirsin.
#
#  Zincir:
#    USB direksiyon -> direksiyon_teleop.py -> vcan0
#                  -> can-bridge -> /cart -> Gazebo aracı
#
#  Kullanım:
#    cd ~/talos-sim/scripts/talos26_ws && ./manuel_baslat.sh
#  Otonom controller'ı yine de istersen:  AUTO=on ./manuel_baslat.sh
#  Gazebo'yu bu script açmasın (zaten açıksa):  otomatik atlanır
# ============================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"   # ~/talos-sim

GREEN='\033[0;32m'; BLUE='\033[0;34m'; RED='\033[0;31m'
YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

AUTO="${AUTO:-off}"                 # on => kontrolcü release modu OLMADAN başlar (hemen sürer, çakışır)
GUI="${GUI:-on}"                    # on => can-visualizer (gui profile) dahil
WHEEL="${WHEEL:-local}"             # local => bu makinedeki USB direksiyon; ssh => sadece talimat bas
WHEEL_DEV="${WHEEL_DEV:-/dev/input/js0}"
WHEEL_ARGS="${WHEEL_ARGS:-}"
# talos-controller HER ZAMAN açılır ama varsayılan olarak "release" modunda:
# bus'a dokunmaz, direksiyon setinde BUTON 3 (0x500=1) ile otonom devralır,
# tekrar buton 3 (0x500=0) ile manuel devralır. AUTO=on eski davranışa döner.
if [ "$AUTO" = on ]; then
    export TALOS_BUS_RELEASE_ON_START=0
else
    export TALOS_BUS_RELEASE_ON_START=1
fi
# Manuel sürüş gaz gücü. ÖLÇEKLEME TEK NOKTADA yapılır: direksiyon_teleop.py'nin
# --throttle-scale'i (POWER). Köprüye (can-bridge/can_to_talos_cart.py) giden
# TALOS_POWER_LIMIT BİLEREK 1.0'da sabitlenir — aksi halde POWER hem teleop'ta
# hem köprüde çarpılır ve efektif güç POWER^2 olurdu (ör. 0.6 -> 0.36); çift
# ölçekleme düzeltmesi 2026-07-04. İstersen: POWER=1.0 ./manuel_baslat.sh
POWER="${POWER:-0.6}"
# ölçekleme teleop'ta --throttle-scale ile tek noktada; köprü limiti 1.0 —
# çift çarpım düzeltmesi 2026-07-04
export TALOS_POWER_LIMIT=1.0
# Buton 3 ile otonom devralınca köprü bu daha düşük/güvenli güç limitine döner
# (ölçekleme can_to_talos_cart.py'de). Manuel POWER gaz otonomda araca geçmez.
export TALOS_POWER_LIMIT_AUTO="${POWER_AUTO:-0.1}"
export TALOS_RAMP_UP="${RAMP_UP:-0.05}"
export TALOS_RAMP_DOWN="${RAMP_DOWN:-0.05}"
# Her manuel oturum kendi RUN_ID alt dizinine loglasin (yoksa hepsi dev/'e birikir)
export RUN_ID="${RUN_ID:-manuel_$(date -u +%Y%m%dT%H%M%SZ)}"
STARTED_GAZEBO=0
WHEEL_PID=""

echo -e "${CYAN}======================================================${NC}"
echo -e "${CYAN}  TALOS MANUEL SÜRÜŞ (direksiyon seti)${NC}"
echo -e "${CYAN}  Otonom kontrolcü: $([ "$AUTO" = on ] && echo "AÇIK (release yok, çakışır)" || echo "HAZIR — buton 3 ile devralır")${NC}"
echo -e "${CYAN}======================================================${NC}"

# -----------------------------------------------------------------
# Kapanış: compose down + (biz açtıysak) gazebo'yu durdur
# -----------------------------------------------------------------
cleanup() {
    echo -e "\n${BLUE}[*] Manuel sistem kapatılıyor...${NC}"
    if [ -n "${WHEEL_PID:-}" ]; then
        kill "$WHEEL_PID" 2>/dev/null || true
        wait "$WHEEL_PID" 2>/dev/null || true
        echo -e "${GREEN}[+] Yerel direksiyon teleop durduruldu${NC}"
    fi
    (cd "$SCRIPT_DIR" && docker compose down --remove-orphans 2>/dev/null) || true
    # Konteyner root olusturdugu tani loglarini kullaniciya geri ver. TUM moduller
    # (karar, hedef, engel, konum, system, control) artik birlesik logs/$RUN_ID/
    # altina yaziyor; baslat.sh ile ayni sekilde bu agaci chown'la (eski hedef/logs
    # yolu da guvenlik icin kalsin).
    for _logdir in "$SCRIPT_DIR/logs/$RUN_ID" "$SCRIPT_DIR/hedef/logs"; do
        if [ -d "$_logdir" ] && command -v sudo >/dev/null; then
            if [ "$(stat -c '%U' "$_logdir" 2>/dev/null)" = "root" ]; then
                sudo chown -R "$(id -u):$(id -g)" "$_logdir" 2>/dev/null || true
            fi
        fi
    done
    if [ "$STARTED_GAZEBO" = "1" ]; then
        pkill -f "gzserver" 2>/dev/null
        pkill -f "gzclient" 2>/dev/null
        [ -n "${SIM_PID:-}" ] && kill "$SIM_PID" 2>/dev/null
        echo -e "${GREEN}[+] Gazebo (script tarafından açılan) durduruldu${NC}"
    fi
    stty sane 2>/dev/null
    echo -e "${GREEN}[+] Kapandı${NC}"
}
trap cleanup EXIT

# -----------------------------------------------------------------
# 1) ROS ortamı (set +u: ROS profile.d nounset altında çöker)
# -----------------------------------------------------------------
echo -e "${BLUE}[1/6] ROS ortamı...${NC}"
set +u
if [ -f "$PROJECT_ROOT/devel/setup.bash" ]; then
    source "$PROJECT_ROOT/devel/setup.bash"; set -u
    echo -e "${GREEN}[+] devel yüklendi${NC}"
else
    source /opt/ros/noetic/setup.bash; set -u
    echo -e "${YELLOW}[!] devel yok, sistem ROS'u — önce catkin_make önerilir${NC}"
fi

if ! pgrep -f rosmaster >/dev/null; then
    echo -e "${YELLOW}[!] roscore yok — başlatılıyor...${NC}"
    nohup roscore >/tmp/manuel_roscore.log 2>&1 &
    sleep 3
fi
echo -e "${GREEN}[+] roscore aktif${NC}"

# -----------------------------------------------------------------
# 2) vcan0 + X11
# -----------------------------------------------------------------
echo -e "${BLUE}[2/6] vcan0...${NC}"
if ! ip link show vcan0 >/dev/null 2>&1; then
    echo -e "${YELLOW}[!] vcan0 yok — oluşturuluyor (sudo)...${NC}"
    sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0 \
        || { echo -e "${RED}[X] vcan0 oluşturulamadı${NC}"; exit 1; }
else
    sudo ip link set up vcan0 2>/dev/null
fi
echo -e "${GREEN}[+] vcan0 aktif${NC}"
xhost +local:docker 2>/dev/null

# -----------------------------------------------------------------
# 2.5) Yerel USB direksiyon -> CAN teleop
# -----------------------------------------------------------------
if [ "$WHEEL" = "local" ]; then
    echo -e "${BLUE}[2.5/6] Yerel USB direksiyon...${NC}"
    if [ ! -e "$WHEEL_DEV" ]; then
        echo -e "${RED}[X] $WHEEL_DEV bulunamadı. Direksiyonu bu bilgisayara tak veya WHEEL_DEV=/dev/input/jsX ver.${NC}"
        echo -e "${YELLOW}    Cihazları görmek için: ls /dev/input/js*${NC}"
        exit 1
    fi
    if ! python3 - <<'PY' >/dev/null 2>&1
import can
PY
    then
        echo -e "${RED}[X] Host Python'da python-can yok. Kur: sudo apt install python3-can veya pip3 install python-can${NC}"
        exit 1
    fi
    nohup python3 "$SCRIPT_DIR/control/direksiyon_teleop.py" \
        --dev "$WHEEL_DEV" \
        --channel vcan0 \
        --throttle-scale "$POWER" \
        --steer-axis 0 \
        --invert-steer \
        --pedal-mode split \
        --pedal-axis 1 \
        --gear-fwd-btn 0 \
        --gear-rev-btn 1 \
        --gear-neutral-btn -1 \
        --handbrake-btn 3 \
        --estop-btn -1 \
        --auto-toggle-btn 2 \
        --verbose \
        $WHEEL_ARGS >/tmp/manuel_direksiyon_teleop.log 2>&1 &
    WHEEL_PID=$!
    sleep 1
    if ! kill -0 "$WHEEL_PID" 2>/dev/null; then
        echo -e "${RED}[X] direksiyon_teleop.py başlatılamadı. Log: /tmp/manuel_direksiyon_teleop.log${NC}"
        tail -20 /tmp/manuel_direksiyon_teleop.log 2>/dev/null || true
        exit 1
    fi
    echo -e "${GREEN}[+] Yerel direksiyon teleop aktif: $WHEEL_DEV -> vcan0${NC}"
    echo -e "${YELLOW}    Not: Linux js tarafında fiziksel buton 3 genelde b2'dir; auto-toggle varsayılanı budur.${NC}"
    echo -e "${YELLOW}    Mapping gerekirse ayrı kalibre et: python3 control/direksiyon_teleop.py --kalibre${NC}"
fi

# -----------------------------------------------------------------
# 3) Gazebo sim (zaten çalışmıyorsa başlat)
# -----------------------------------------------------------------
echo -e "${BLUE}[3/6] Gazebo sim...${NC}"
if pgrep -f gzserver >/dev/null; then
    echo -e "${GREEN}[+] Gazebo zaten çalışıyor — yeniden başlatılmadı${NC}"
else
    echo -e "${YELLOW}[!] Gazebo başlatılıyor (cart_sim.launch)...${NC}"
    nohup roslaunch cart_sim cart_sim.launch >/tmp/manuel_gazebo.log 2>&1 &
    SIM_PID=$!
    STARTED_GAZEBO=1
    # /cart subscriber (gazebo cart plugin) gelene kadar bekle
    for i in $(seq 1 30); do
        rostopic info /cart 2>/dev/null | grep -q "/gazebo" && break
        sleep 1
    done
    echo -e "${GREEN}[+] Gazebo hazır${NC}"
fi

# Host'ta elde başlatılmış can_to_talos_cart varsa kapat (çift köprü olmasın)
pkill -f "can_to_talos_cart.py" 2>/dev/null && \
    echo -e "${YELLOW}[!] Host'taki manuel can_to_talos_cart kapatıldı (container köprüsü kullanılacak)${NC}"

# cart_sim.launch joy teleop node'u (/controller) /cart'a 20Hz basar; joystick
# olmasa bile timer son komutu (genelde sıfır) sonsuza dek tekrar yayınlar ->
# can-bridge'in direksiyon/gaz frame'leriyle aynı topic'te yarışıp ~%50 ezer.
# Manuelde tek /cart sahibi can-bridge olmalı -> joy teleop'u durdur.
if rosnode list 2>/dev/null | grep -q '^/controller$'; then
    rosnode kill /controller 2>/dev/null && \
        echo -e "${YELLOW}[!] cart_sim joy teleop (/controller) durduruldu — /cart'ta can-bridge ile çakışmıyor${NC}"
fi

# -----------------------------------------------------------------
# 4) talos-all imajı
# -----------------------------------------------------------------
echo -e "${BLUE}[4/6] Docker imajı...${NC}"
if ! docker image inspect talos-all:latest >/dev/null 2>&1; then
    echo -e "${YELLOW}  [*] talos-all build ediliyor...${NC}"
    docker build -t talos-all:latest -f "$SCRIPT_DIR/Dockerfile.all" "$SCRIPT_DIR" \
        || { echo -e "${RED}  [X] build başarısız${NC}"; exit 1; }
fi
echo -e "${GREEN}[+] talos-all hazır${NC}"

# -----------------------------------------------------------------
# 5) docker compose up — otonom controller HARİÇ servis listesi
# -----------------------------------------------------------------
echo -e "${BLUE}[5/6] Modüller başlatılıyor (otonom kontrolcü hariç)...${NC}"
cd "$SCRIPT_DIR" || exit 1

# Eski container kalıntıları
docker rm -f konum-server talos-map-server hedef_teslimi engel-node \
    traffic-node park-durak-node lane-follower yaya-gecidi-node karar-node \
    talos-can-bridge talos-state-bridge talos-controller talos-can-visualizer 2>/dev/null

# Manuel modda çalışacak servisler. talos-controller release modunda DAHİL:
# idle bekler, buton 3 ile devralır (TALOS_BUS_RELEASE_ON_START yukarıda set edildi).
SERVICES=(
    konum-server talos-map-server hedef-teslimi
    engel-node traffic-node park-durak-node
    lane-follower yaya-gecidi-node karar-node
    can-bridge state-bridge talos-controller
)
[ "$GUI" = on ] && SERVICES+=(can-visualizer)
[ "$AUTO" = on ] && \
    echo -e "${RED}[!] AUTO=on: kontrolcü release OLMADAN başlıyor — başından itibaren direksiyonla ÇAKIŞIR!${NC}"

COMPOSE_ARGS=(up -d)
[ "$GUI" = on ] && COMPOSE_ARGS=(--profile gui "${COMPOSE_ARGS[@]}")

docker compose "${COMPOSE_ARGS[@]}" "${SERVICES[@]}" 2>&1 | tail -20
sleep 3
docker compose ps

# -----------------------------------------------------------------
# 6) Durum + direksiyon bağlantı talimatı
# -----------------------------------------------------------------
HOST_IP=$(ip -4 addr show scope global 2>/dev/null | grep -oP 'inet \K[\d.]+' | head -1)
echo ""
echo -e "${GREEN}[+] MANUEL SİSTEM HAZIR — otonom kontrolcü idle bekliyor${NC}"
echo -e "${YELLOW}    Direksiyon setinde BUTON 3: otonom devral <-> manuel devral (toggle)${NC}"
echo -e "${CYAN}======================================================${NC}"
if [ "$WHEEL" = "local" ]; then
    echo -e "${CYAN}  Yerel direksiyon aktif: ${WHEEL_DEV}${NC}"
    echo -e "${YELLOW}  Teleop logu: /tmp/manuel_direksiyon_teleop.log${NC}"
else
    echo -e "${CYAN}  Uzak Windows PC'den sürmek için (cmd.exe):${NC}"
    echo -e "${YELLOW}  powershell -ExecutionPolicy Bypass -File direksiyon_oku.ps1 | \\${NC}"
    echo -e "${YELLOW}    ssh hilmi@${HOST_IP:-<bu-makine-ip>} \"python3 ~/talos-sim/scripts/talos26_ws/control/direksiyon_can_server.py --verbose --invert-steer\"${NC}"
fi
echo -e "${CYAN}------------------------------------------------------${NC}"
echo -e "${CYAN}  Çalışan modüller: konum, harita, hedef, engel, trafik,${NC}"
echo -e "${CYAN}  şerit, yaya, park, karar + can/state köprüleri${NC}"
echo -e "${CYAN}  can-bridge   => vcan0 (direksiyon) -> /cart -> Gazebo${NC}"
echo -e "${CYAN}  Gaz gücü     => teleop --throttle-scale=${POWER} (POWER=1.0 ile tam güç; köprü limiti sabit 1.0 — tek nokta ölçekleme)${NC}"
echo -e "${YELLOW}  Ctrl+C => modülleri kapatır (compose down)${NC}"
echo -e "${CYAN}======================================================${NC}"

# Foreground: algı/karar loglarını akıt (Ctrl+C => cleanup)
echo -e "${BLUE}[6/6] Modül logları (Ctrl+C => kapanış)${NC}"
docker compose logs -f --no-color talos-controller karar-node engel-node lane-follower \
    yaya-gecidi-node park-durak-node traffic-node 2>&1 || true
