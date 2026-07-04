#!/bin/bash
# ============================================================
#  TALOS Tam Sistem Baslatici (compose tabanli, post-ULTRAREVIEW)
#  - Tum konteynerler `docker compose up -d` ile baslatilir
#  - x-log-defaults, RUN_ID, talos_common mount'lari otomatik uygulanir
#  - rosbag (hafif profil) + candump arka planda calisir
#  - Cikista manifest mühürlenir, log sahipligi host kullanicisina chown'lanir
# ============================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# =============================================================
# COMPOSE DOSYA SEÇİMİ — beemobs (Bee1 araç arayüzü) modu opsiyonel katman
# TALOS_BEEMOBS=1 ./baslat.sh -> can-bridge/state-bridge beemobs_*.py'ye döner
# (bkz. docker-compose.beemobs.yml). TEK LAUNCHER kuralı: elle çift -f
# komutu yalnız hata ayıklama içindir, normal akış bu script üzerindendir.
# =============================================================
COMPOSE_FILES="-f docker-compose.yml"
if [ "${TALOS_BEEMOBS:-0}" = "1" ]; then
    COMPOSE_FILES="$COMPOSE_FILES -f docker-compose.beemobs.yml"
    echo -e "${CYAN}[beemobs modu] docker-compose.beemobs.yml katmanı devrede (can-bridge/state-bridge -> beemobs_*).${NC}"
fi

# =============================================================
# RUN ID + LOG DIZINI (KTR §9.14.3 + new plan §2.2)
# =============================================================
export RUN_ID="${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
RUN_DIR="$SCRIPT_DIR/logs/$RUN_ID"
mkdir -p "$RUN_DIR/rosbag" "$RUN_DIR/can" "$RUN_DIR/system"

echo -e "${CYAN}======================================================${NC}"
echo -e "${CYAN}  TALOS Otonom Surus - Tam Sistem (compose)${NC}"
echo -e "${CYAN}  RUN_ID  = $RUN_ID${NC}"
echo -e "${CYAN}  RUN_DIR = $RUN_DIR${NC}"
echo -e "${CYAN}======================================================${NC}"

# =============================================================
# TEMIZLIK FONKSIYONU
# =============================================================
cleanup() {
    echo -e "\n${BLUE}[*] Sistem kapatiliyor...${NC}"

    # Background python köprüleri (geriye dönük uyumluluk için)
    [ -n "${BRIDGE_PID:-}" ]  && kill "$BRIDGE_PID"  2>/dev/null
    [ -n "${STATE_PID:-}"  ]  && kill "$STATE_PID"   2>/dev/null
    [ -n "${VIS_PID:-}"    ]  && kill "$VIS_PID"     2>/dev/null

    # candump — yumusak kapat
    if [ -n "${CANDUMP_PID:-}" ]; then
        kill "$CANDUMP_PID" 2>/dev/null
        wait "$CANDUMP_PID" 2>/dev/null
    fi

    # rosbag — SIGINT, finalize edebilmesi icin wait
    if [ -n "${ROSBAG_PID:-}" ]; then
        kill -INT "$ROSBAG_PID" 2>/dev/null
        wait "$ROSBAG_PID" 2>/dev/null
    fi

    # Engel algilama host node'lari (kerem detector + ground_filter)
    [ -n "${DETECTOR_PID:-}" ]     && kill -INT "$DETECTOR_PID"     2>/dev/null
    [ -n "${GROUNDFILTER_PID:-}" ] && kill "$GROUNDFILTER_PID" 2>/dev/null

    # Compose down (degişiklik yapilmiş volume mount'lari da kapsar)
    (cd "$SCRIPT_DIR" && docker compose $COMPOSE_FILES down --remove-orphans 2>/dev/null) || true

    # Manifest end-of-run mühür
    if [ -f "$RUN_DIR/manifest.json" ]; then
        END_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)
        sed -i "s/\"ended_at\": *null/\"ended_at\": \"$END_AT\"/" "$RUN_DIR/manifest.json" 2>/dev/null
    fi

    # B2 fix: konteyner root olusturdugu dosyalari kullaniciya geri ver
    # RUN_DIR + hedef-teslimi'nin tani loglari (hedef/logs) chown'lanir
    for _logdir in "$RUN_DIR" "$SCRIPT_DIR/hedef/logs"; do
        if [ -d "$_logdir" ] && command -v sudo >/dev/null; then
            if [ "$(stat -c '%U' "$_logdir" 2>/dev/null)" = "root" ]; then
                sudo chown -R "$(id -u):$(id -g)" "$_logdir" 2>/dev/null || true
            fi
        fi
    done

    stty sane 2>/dev/null
    echo -e "${GREEN}[+] Tum konteynerler ve islemler durduruldu${NC}"
}
trap cleanup EXIT

# =============================================================
# MANIFEST (KTR §9.14.6 + new plan §2.4)
# =============================================================
write_manifest() {
    local sim_repo="$PROJECT_ROOT"
    local stack_repo="$SCRIPT_DIR"
    local sim_sha=$(cd "$sim_repo" 2>/dev/null && git rev-parse --short HEAD 2>/dev/null || echo "no-git")
    local stack_sha=$(cd "$stack_repo" 2>/dev/null && git rev-parse --short HEAD 2>/dev/null || echo "no-git")
    local stack_branch=$(cd "$stack_repo" 2>/dev/null && git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
    cat > "$RUN_DIR/manifest.json" <<EOF
{
  "run_id": "$RUN_ID",
  "started_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "ended_at": null,
  "operator": "${USER:-unknown}",
  "host": "$(hostname)",
  "sim_git_sha": "$sim_sha",
  "stack_git_sha": "$stack_sha",
  "stack_branch": "$stack_branch",
  "ros_distro": "noetic",
  "log_root": "$RUN_DIR"
}
EOF
}
write_manifest

# =============================================================
# 1) ROS ORTAMI
# =============================================================
echo -e "${BLUE}[1/8] ROS ortami yukleniyor...${NC}"
# ROS setup.bash, set -u (nounset) altinda cokuyor: catkin profile.d scriptleri
# ROS_DISTRO/CMAKE_PREFIX_PATH gibi degiskenleri default'suz expand ediyor → temiz
# bir shell'de (ROS_DISTRO .bashrc'de export edilmemisse) "unbound variable" ile
# non-interactive shell aninda EXIT eder. Bu yuzden source'lari set +u ile sariyoruz.
set +u
if [ -f "$PROJECT_ROOT/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$PROJECT_ROOT/devel/setup.bash"
    set -u
    echo -e "${GREEN}[+] ROS ortami yuklendi (devel)${NC}"
else
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    set -u
    echo -e "${YELLOW}[!] devel/setup.bash bulunamadi, sistem ROS kullaniliyor${NC}"
fi

# roscore yoksa baslat
if ! pgrep -f rosmaster >/dev/null; then
    echo -e "${YELLOW}[!] roscore calismiyor — host'ta baslatiliyor...${NC}"
    nohup roscore > "$RUN_DIR/system/roscore.log" 2>&1 &
    sleep 3
fi

# =============================================================
# 2) VCAN0
# =============================================================
echo -e "${BLUE}[2/8] Virtual CAN kontrol ediliyor...${NC}"
if ! ip link show vcan0 > /dev/null 2>&1; then
    echo -e "${YELLOW}[!] vcan0 bulunamadi. Olusturuluyor (sudo)...${NC}"
    if sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0; then
        echo -e "${GREEN}[+] vcan0 olusturuldu${NC}"
    else
        echo -e "${RED}[X] vcan0 olusturulamadi!${NC}"
        exit 1
    fi
else
    sudo ip link set up vcan0 2>/dev/null
    echo -e "${GREEN}[+] vcan0 aktif${NC}"
fi
xhost +local:docker 2>/dev/null

# =============================================================
# 3) HAM YEDEK: candump + rosbag (KTR §9.14.1 L3/L4)
# =============================================================
echo -e "${BLUE}[3/8] Ham yedek kayitlari baslatiliyor...${NC}"

if command -v candump >/dev/null 2>&1; then
    candump -L vcan0 > "$RUN_DIR/can/vcan0.log" 2>/dev/null &
    CANDUMP_PID=$!
    echo -e "${GREEN}[+] candump aktif (PID=$CANDUMP_PID)${NC}"
else
    echo -e "${YELLOW}[!] candump yok (can-utils paketi gerekli) — atlandi${NC}"
fi

if command -v rosbag >/dev/null 2>&1; then
    ROSBAG_TOPICS=(
        /cart /konum /imu /base_pose_ground_truth
        /hedef /waypoint /gorev_durumu /map_metadata
        /karar /karar_decision
        /trafik_levha /yaya_gecidi /park_alani /durak_alani /yaya_gecidi/model
        /engel /engel_distance /engel_angle /engel_sol_mesafe /engel_sag_mesafe
        /obstacles/poses
        /battery_state /steer_angle /line
        /lane_offset /lane/turn_type /lane/confidence
    )
    rosbag record --lz4 -O "$RUN_DIR/rosbag/light_${RUN_ID}" \
        --split --duration=15m --max-splits=8 \
        "${ROSBAG_TOPICS[@]}" > "$RUN_DIR/rosbag/rosbag_record.log" 2>&1 &
    ROSBAG_PID=$!
    echo -e "${GREEN}[+] rosbag (hafif profil) aktif (PID=$ROSBAG_PID)${NC}"
else
    echo -e "${YELLOW}[!] rosbag yok — atlandi${NC}"
fi

# =============================================================
# 3b) ENGEL ALGILAMA (kerem talos_obstacle_detector) — HOST node'lari
# C++ binary host catkin'inde (devel) derli; talos-all imajinda pcl/jsk runtime
# olmadigindan container yerine host'ta calistirilir. Girdi /cart/points_noground
# icin minimal zemin-ayiklama (ground_filter.py) onunde kosar. Binary yoksa
# (kerem branch build edilmemis) sessizce atlanir — baslat.sh akisi bozulmaz.
# Kapatmak: OBSTACLE_DETECTOR=off ./baslat.sh
# =============================================================
OBSTACLE_DETECTOR="${OBSTACLE_DETECTOR:-auto}"
DET_BIN="$PROJECT_ROOT/devel/lib/talos_obstacle_detector/obstacle_detector_node"
if [ "$OBSTACLE_DETECTOR" != "off" ] && [ -x "$DET_BIN" ] && [ -f "$SCRIPT_DIR/lidar/ground_filter.py" ]; then
    nohup python3 "$SCRIPT_DIR/lidar/ground_filter.py" \
        > "$RUN_DIR/system/ground_filter.log" 2>&1 &
    GROUNDFILTER_PID=$!
    nohup roslaunch talos_obstacle_detector obstacle_detector.launch \
        > "$RUN_DIR/system/obstacle_detector.log" 2>&1 &
    DETECTOR_PID=$!
    echo -e "${GREEN}[+] engel algilama aktif — ground_filter (PID=$GROUNDFILTER_PID) + obstacle_detector (PID=$DETECTOR_PID)${NC}"
    echo -e "${CYAN}    /cart/center_laser/scan → /cart/points_noground → /obstacles/poses${NC}"
else
    echo -e "${YELLOW}[!] talos_obstacle_detector binary yok veya OBSTACLE_DETECTOR=off — engel algilama atlandi (legacy /engel* kullanilir)${NC}"
fi

# =============================================================
# 4) DOCKER IMAGE KONTROL / YUKLEME
# =============================================================
echo -e "${BLUE}[4/8] Docker imajlari kontrol ediliyor...${NC}"

# talos-all:latest — TEK runtime imaji. TUM servisler (konum, map-server, hedef,
# engel, karar, traffic, safe-zone, can-bridge, state-bridge, talos-controller,
# can-visualizer) bunu kullanir. Eksikse Dockerfile.all'dan build edilir; repo
# kendi kendine yeter, harici .tar gerektirmez. Tum Python kodu compose ile bind-mount'lu.
if ! docker image inspect talos-all:latest > /dev/null 2>&1; then
    echo -e "${YELLOW}  [*] talos-all build ediliyor (Dockerfile.all)...${NC}"
    docker build -t talos-all:latest -f "$SCRIPT_DIR/Dockerfile.all" "$SCRIPT_DIR" \
        || { echo -e "${RED}  [X] talos-all build basarisiz, cikiliyor${NC}"; exit 1; }
else
    echo -e "${GREEN}  [+] talos-all mevcut${NC}"
fi

# =============================================================
# 5) DOCKER COMPOSE UP (tüm servisler)
# =============================================================
echo -e "${BLUE}[5/8] Docker compose up...${NC}"
cd "$SCRIPT_DIR" || exit 1

# Eski container kalintilari (manuel docker run'dan) — temizle
docker rm -f konum-server talos-map-server hedef_teslimi engel-node \
              traffic-node park-durak-node lane-follower yaya-gecidi-node karar-node \
              talos-can-bridge talos-state-bridge talos-controller \
              talos-can-visualizer 2>/dev/null

# Karar/engel/talos-controller dahil tum servisler ayaga kalkar.
# can-visualizer 'gui' profile altinda; --profile gui ile dahil ediliyor.
docker compose $COMPOSE_FILES --profile gui up -d 2>&1 | tail -20

sleep 3
docker compose $COMPOSE_FILES ps

# =============================================================
# 6) DURUM
# =============================================================
echo ""
echo -e "${GREEN}[+] TUM BILESENLER AKTIF${NC}"
echo -e "${CYAN}======================================================${NC}"
echo -e "${CYAN}  RUN_ID = $RUN_ID${NC}"
echo -e "${CYAN}  konum-server       => /konum (Pose2D)${NC}"
echo -e "${CYAN}  talos-map-server   => /map + /waypoint${NC}"
echo -e "${CYAN}  hedef_teslimi      => /hedef (String: x,y)${NC}"
echo -e "${CYAN}  engel-node         => /engel + obstacles.csv${NC}"
echo -e "${CYAN}  traffic-node       => /trafik_levha${NC}"
echo -e "${CYAN}  park-durak-node    => /park_alani + /durak_alani${NC}"
echo -e "${CYAN}  lane-follower      => /line + /lane/* (serit takip)${NC}"
echo -e "${CYAN}  yaya-gecidi-node   => /yaya_gecidi/model + image_annotated (adanmis)${NC}"
echo -e "${CYAN}  karar-node         => /karar + /karar_decision (decision_id)${NC}"
echo -e "${CYAN}  can-bridge         => CAN -> Gazebo${NC}"
echo -e "${CYAN}  state-bridge       => Gazebo -> CAN${NC}"
echo -e "${CYAN}  talos-controller   => Ana kontrolcu${NC}"
echo -e "${CYAN}  can-visualizer     => CAN GUI${NC}"
echo -e "${CYAN}------------------------------------------------------${NC}"
echo -e "${YELLOW}  Loglar: $RUN_DIR${NC}"
echo -e "${YELLOW}  Ctrl+C => kapanis + manifest mührü + chown${NC}"
echo -e "${CYAN}======================================================${NC}"

# =============================================================
# 7) BACKGROUND — hedef_teslimi log arşivi (FILTER-DEBUG, U-DONUS-PLAN için)
# =============================================================
docker compose $COMPOSE_FILES logs -f --no-color hedef-teslimi \
    > "$RUN_DIR/system/hedef_teslimi.log" 2>&1 &
HEDEF_LOG_PID=$!
echo -e "${GREEN}[+] hedef_teslimi log arşivi: $RUN_DIR/system/hedef_teslimi.log${NC}"

# =============================================================
# 8) FOREGROUND — talos-controller log akisi (Ctrl+C ile cikis)
# =============================================================
echo -e "${BLUE}[8/8] talos-controller log akisi (Ctrl+C => kapanis)${NC}"
docker compose $COMPOSE_FILES logs -f --no-color talos-controller karar-node engel-node park-durak-node lane-follower yaya-gecidi-node 2>&1 || true

# Background hedef_teslimi log tail'ini kapat
[ -n "${HEDEF_LOG_PID:-}" ] && kill "$HEDEF_LOG_PID" 2>/dev/null

# Trap cleanup yakalar
