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
# TALOS_ARAC=1 → GERÇEK ARAÇ modu: beemobs katmanı otomatik devreye girer ve
# state-bridge (Bee1 Gazebo EMÜLATÖRÜ) BAŞLATILMAZ — gerçek ECU feedback'iyle aynı
# topic'lere binip sahte hız/açı/e-stop üretir (bkz. docker-compose.beemobs.yml başlığı).
if [ "${TALOS_ARAC:-0}" = "1" ]; then
    TALOS_BEEMOBS=1
fi

COMPOSE_FILES="-f docker-compose.yml"
if [ "${TALOS_BEEMOBS:-0}" = "1" ]; then
    COMPOSE_FILES="$COMPOSE_FILES -f docker-compose.beemobs.yml"
    echo -e "${CYAN}[beemobs modu] docker-compose.beemobs.yml katmanı devrede (can-bridge/state-bridge -> beemobs_*).${NC}"
fi
if [ "${TALOS_ARAC:-0}" = "1" ]; then
    echo -e "${RED}[ARAÇ MODU] Gerçek araç: state-bridge (sim emülatörü) BAŞLATILMAYACAK.${NC}"
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

    # Oto-restart gözcüsü (docker restart olaylarını izleyen arka plan döngüsü)
    [ -n "${GOZCU_PID:-}" ]   && kill "$GOZCU_PID"   2>/dev/null

    # Background python köprüleri (geriye dönük uyumluluk için)
    [ -n "${BRIDGE_PID:-}" ]  && kill "$BRIDGE_PID"  2>/dev/null
    [ -n "${STATE_PID:-}"  ]  && kill "$STATE_PID"   2>/dev/null
    [ -n "${VIS_PID:-}"    ]  && kill "$VIS_PID"     2>/dev/null

    # Karar panosu (host GUI, salt-görsel)
    [ -n "${PANEL_PID:-}"  ]  && kill "$PANEL_PID"   2>/dev/null

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

    # Canlı parametre dosyasının koşu-SONU hali (başlangıç kopyasıyla diff'lenir)
    [ -f "$SCRIPT_DIR/config/canli_params.yaml" ] && \
        cp "$SCRIPT_DIR/config/canli_params.yaml" "$RUN_DIR/system/canli_params.bitis.yaml" 2>/dev/null

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
# OTO-RESTART GÖZCÜSÜ (2026-07-15)
# =============================================================
# docker-compose.yml'de her servis `restart: "on-failure:5"` → çöken container'ı
# Docker daemon en fazla 5 kez otomatik yeniden başlatır. Bu döngü RestartCount'ı
# izleyip her restart'ı ve "5 denemede toparlanamadı" (vazgeçti) durumunu
# logs/$RUN_ID/system/watchdog.log'a + terminale UYARI olarak yansıtır (restart'lar
# sessiz kalmasın). Aktüasyon tier'ı (controller/can-bridge/state-bridge) KIRMIZI
# işaretlenir: sim gaz-watchdog'u olmadığından o restart penceresinde araç son
# gazla süzülmüş olabilir → operatör durumu doğrulasın.
# 3 sn aralıkla `docker inspect` polling — çıktı yalnızca DEĞİŞİMDE üretilir (spam yok).
GOZCU_AKTUASYON_RE='talos-controller|talos-can-bridge|talos-state-bridge'
restart_gozcusu() {
    declare -A _rc_seen        # container -> son görülen RestartCount (high-water)
    declare -A _down_uyarildi  # container -> vazgeçme uyarısı bir kez basıldı mı
    local wlog="$RUN_DIR/system/watchdog.log"
    echo "# TALOS oto-restart gözcüsü başladı RUN_ID=$RUN_ID $(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$wlog" 2>/dev/null
    while true; do
        local ids
        ids=$(cd "$SCRIPT_DIR" && docker compose $COMPOSE_FILES ps -aq 2>/dev/null)
        if [ -n "$ids" ]; then
            while read -r cid; do
                [ -n "$cid" ] || continue
                local info name rc status ec ts
                info=$(docker inspect -f '{{.Name}}|{{.RestartCount}}|{{.State.Status}}|{{.State.ExitCode}}' "$cid" 2>/dev/null) || continue
                name=${info%%|*}; name=${name#/}
                rc=$(printf '%s' "$info" | cut -d'|' -f2)
                status=$(printf '%s' "$info" | cut -d'|' -f3)
                ec=$(printf '%s' "$info" | cut -d'|' -f4)
                [ -n "$rc" ] || rc=0
                local prev=${_rc_seen[$name]:-0}
                # Yeni bir otomatik restart olduysa
                if [ "$rc" -gt "$prev" ] 2>/dev/null; then
                    _rc_seen[$name]=$rc
                    ts=$(date -u +%Y-%m-%dT%H:%M:%SZ)
                    if printf '%s' "$name" | grep -qE "$GOZCU_AKTUASYON_RE"; then
                        echo -e "${RED}[GÖZCÜ $ts] ⚠ AKTÜASYON '$name' YENİDEN BAŞLATILDI ($rc/5, exit=$ec) — sim gaz-watchdog'u yok, araç süzülmüş olabilir; DURUMU KONTROL ET.${NC}"
                    else
                        echo -e "${YELLOW}[GÖZCÜ $ts] ↻ '$name' yeniden başlatıldı ($rc/5, exit=$ec)${NC}"
                    fi
                    echo "$ts RESTART $name count=$rc exit=$ec" >> "$wlog" 2>/dev/null
                fi
                # 5 denemede toparlanamadı → vazgeçti (bir kez uyar)
                if [ "$rc" -ge 5 ] 2>/dev/null && { [ "$status" = "exited" ] || [ "$status" = "dead" ]; } && [ -z "${_down_uyarildi[$name]:-}" ]; then
                    _down_uyarildi[$name]=1
                    ts=$(date -u +%Y-%m-%dT%H:%M:%SZ)
                    echo -e "${RED}[GÖZCÜ $ts] ❌ '$name' 5 denemede TOPARLANAMADI — DURDU (exit=$ec). Elle müdahale gerek.${NC}"
                    echo "$ts GAVEUP $name exit=$ec" >> "$wlog" 2>/dev/null
                fi
                # Tekrar çalışır hale gelirse vazgeçme bayrağını temizle
                if [ "$status" = "running" ] && [ -n "${_down_uyarildi[$name]:-}" ]; then
                    unset "_down_uyarildi[$name]"
                fi
            done <<< "$ids"
        fi
        sleep 3
    done
}

# Canlı parametre dosyasının koşu-BAŞI snapshot'ı (provenance — hangi ayarla sürüldü)
[ -f "$SCRIPT_DIR/config/canli_params.yaml" ] && \
    cp "$SCRIPT_DIR/config/canli_params.yaml" "$RUN_DIR/system/canli_params.baslangic.yaml" 2>/dev/null

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
# ARAÇ MODU (TALOS_ARAC=1): servis listesi compose'dan türetilir, state-bridge çıkarılır —
# emülatör gerçek araçta ASLA kalkmasın (elle liste tutulmaz, drift olmaz).
if [ "${TALOS_ARAC:-0}" = "1" ]; then
    ARAC_SERVISLER=$(docker compose $COMPOSE_FILES --profile gui config --services 2>/dev/null | grep -v '^state-bridge$' | tr '\n' ' ')
    if [ -z "$ARAC_SERVISLER" ]; then
        echo -e "${RED}[X] Servis listesi türetilemedi — güvenli tarafta kalınıyor, çıkılıyor.${NC}"
        exit 1
    fi
    docker compose $COMPOSE_FILES --profile gui up -d $ARAC_SERVISLER 2>&1 | tail -20
else
    docker compose $COMPOSE_FILES --profile gui up -d 2>&1 | tail -20
fi

sleep 3
docker compose $COMPOSE_FILES ps

# Oto-restart gözcüsünü başlat (RestartCount izler, çökme/restart uyarısı basar)
restart_gozcusu &
GOZCU_PID=$!
echo -e "${GREEN}[+] Oto-restart gözcüsü aktif (PID=$GOZCU_PID) — restart:on-failure:5, log: $RUN_DIR/system/watchdog.log${NC}"

# =============================================================
# 5b) KARAR PANOSU (host GUI) — /karar + /karar_bt/snapshot canlı izleme
# Salt-görsel matplotlib penceresi (karar_panel.py); hiçbir topic'e YAZMAZ, karar
# davranışını etkilemez. karar-node ayağa kalktıktan (compose up) sonra açılır ki
# ilk tick'te dolu gelsin. GUI olduğu için DISPLAY şart — headless/SSH oturumunda
# (DISPLAY boş) sessizce atlanır, baslat.sh akışı bozulmaz. Container değil host
# python3'ü kullanır (ground_filter/obstacle_detector ile aynı desen).
# Kapatmak: KARAR_PANEL=off ./baslat.sh   ·   Hız: KARAR_PANEL_HZ=5 (bkz. panel)
# =============================================================
KARAR_PANEL="${KARAR_PANEL:-auto}"
if [ "$KARAR_PANEL" != "off" ] && [ -n "${DISPLAY:-}" ] && [ -f "$SCRIPT_DIR/karar/karar_panel.py" ]; then
    MPLBACKEND="${MPLBACKEND:-TkAgg}" nohup python3 "$SCRIPT_DIR/karar/karar_panel.py" \
        > "$RUN_DIR/system/karar_panel.log" 2>&1 &
    PANEL_PID=$!
    echo -e "${GREEN}[+] Karar panosu açıldı (PID=$PANEL_PID) — /karar + snapshot canlı (log: $RUN_DIR/system/karar_panel.log)${NC}"
elif [ "$KARAR_PANEL" = "off" ]; then
    echo -e "${YELLOW}[!] KARAR_PANEL=off — karar panosu atlandı${NC}"
elif [ -z "${DISPLAY:-}" ]; then
    echo -e "${YELLOW}[!] DISPLAY yok — karar panosu atlandı (headless/SSH). Ekran başında çalıştır ya da elle: python3 karar/karar_panel.py${NC}"
else
    echo -e "${YELLOW}[!] karar_panel.py bulunamadı — pano atlandı${NC}"
fi

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
echo -e "${CYAN}  karar-panosu       => /karar + snapshot (salt-görsel GUI)${NC}"
echo -e "${CYAN}------------------------------------------------------${NC}"
echo -e "${YELLOW}  Loglar: $RUN_DIR${NC}"
echo -e "${YELLOW}  CANLI PARAMETRE: config/canli_params.yaml duzenle => RESTART'SIZ uygulanir (~1 sn)${NC}"
echo -e "${YELLOW}                   (istisna: karar-node + RESTART isaretli parametreler)${NC}"
echo -e "${YELLOW}  OTO-RESTART: her servis çökerse Docker 5 kez yeniden dener (on-failure:5)${NC}"
echo -e "${YELLOW}               restart/vazgeçme uyarıları => $RUN_DIR/system/watchdog.log${NC}"
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
