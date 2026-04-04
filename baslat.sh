#!/bin/bash
# ============================================================
#  TALOS Tam Sistem Baslatici
#  Tum Docker'lari + CAN koprusunu + start sinyalini yonetir
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HILMI_TALOS="$SCRIPT_DIR/hilmi-talos"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Renkler
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

TOTAL_STEPS=12

echo -e "${CYAN}======================================================${NC}"
echo -e "${CYAN}  TALOS Otonom Surus - Tam Sistem${NC}"
echo -e "${CYAN}  konum + map-server + hedef_teslimi + engel${NC}"
echo -e "${CYAN}  traffic + lane + karar + control${NC}"
echo -e "${CYAN}======================================================${NC}"

# =============================================================
# TEMIZLIK FONKSIYONU
# =============================================================
cleanup() {
    echo -e "\n${BLUE}[*] Sistem kapatiliyor...${NC}"
    [ -n "$BRIDGE_PID" ]  && kill $BRIDGE_PID 2>/dev/null
    [ -n "$STATE_PID" ]   && kill $STATE_PID 2>/dev/null
    [ -n "$VIS_PID" ]     && kill $VIS_PID 2>/dev/null
    [ -n "$LANE_PID" ]    && kill $LANE_PID 2>/dev/null
    docker stop talos-controller talos-map-server hedef_teslimi konum-server \
                engel-node traffic-node karar-node 2>/dev/null
    docker rm -f talos-controller talos-map-server hedef_teslimi konum-server \
                 engel-node traffic-node karar-node 2>/dev/null
    stty sane 2>/dev/null
    echo -e "${GREEN}[+] Tum konteynerler ve islemler durduruldu${NC}"
}
trap cleanup EXIT

# =============================================================
# 1) ROS ORTAMI
# =============================================================
echo -e "${BLUE}[1/${TOTAL_STEPS}] ROS ortami yukleniyor...${NC}"
if [ -f "$PROJECT_ROOT/devel/setup.bash" ]; then
    source "$PROJECT_ROOT/devel/setup.bash"
    echo -e "${GREEN}[+] ROS ortami yuklendi (devel)${NC}"
else
    source /opt/ros/noetic/setup.bash 2>/dev/null
    echo -e "${YELLOW}[!] devel/setup.bash bulunamadi, sistem ROS kullaniliyor${NC}"
fi

# =============================================================
# 2) VCAN0
# =============================================================
echo -e "${BLUE}[2/${TOTAL_STEPS}] Virtual CAN kontrol ediliyor...${NC}"
if ! ip link show vcan0 > /dev/null 2>&1; then
    echo -e "${YELLOW}[!] vcan0 bulunamadi. Olusturuluyor...${NC}"
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

# =============================================================
# 3) DOCKER IMAGE KONTROL / YUKLEME
# =============================================================
echo -e "${BLUE}[3/${TOTAL_STEPS}] Docker imajlari kontrol ediliyor...${NC}"

load_image() {
    local name=$1 tar=$2
    if ! docker image inspect "$name" > /dev/null 2>&1; then
        if [ -f "$tar" ]; then
            echo -e "${YELLOW}  [*] $name yukleniyor ($tar)...${NC}"
            docker load -i "$tar"
        else
            echo -e "${RED}  [X] $name imaji ve $tar dosyasi bulunamadi!${NC}"
            return 1
        fi
    else
        echo -e "${GREEN}  [+] $name mevcut${NC}"
    fi
}

load_image "konum:latest"                  "$SCRIPT_DIR/konum.tar"
load_image "talos-map-server:latest"       "$SCRIPT_DIR/talos-map-waypoint.tar"
load_image "hedef-yoneticisi:latest"       "$SCRIPT_DIR/hedef_yoneticisi_v1.tar"
load_image "otonom-arac:latest"            "$SCRIPT_DIR/engel_node.tar"
load_image "karar-node:latest"             "$SCRIPT_DIR/karar_node_x86.tar"
load_image "traffic_docker:latest"         "$SCRIPT_DIR/traffic_docker.tar"

if ! docker image inspect talos-control:latest > /dev/null 2>&1; then
    if [ -f "$HILMI_TALOS/talos_control.tar" ]; then
        echo -e "${YELLOW}  [*] talos-control yukleniyor...${NC}"
        docker load -i "$HILMI_TALOS/talos_control.tar"
    else
        echo -e "${YELLOW}  [*] talos-control build ediliyor...${NC}"
        docker build -t talos-control:latest "$HILMI_TALOS"
    fi
else
    echo -e "${GREEN}  [+] talos-control mevcut${NC}"
fi

# =============================================================
# 4) KONUM DOCKER
# =============================================================
echo -e "${BLUE}[4/${TOTAL_STEPS}] Konum servisi baslatiliyor...${NC}"
docker rm -f konum-server 2>/dev/null
docker run -d --rm \
    --name konum-server \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -v "$SCRIPT_DIR/fixes/konum.py:/app/konum.py:ro" \
    konum:latest
sleep 2
if docker ps --filter name=konum-server --format '{{.Names}}' | grep -q konum-server; then
    echo -e "${GREEN}[+] Konum servisi aktif${NC}"
else
    echo -e "${RED}[X] Konum servisi baslatilamadi!${NC}"
    exit 1
fi

# =============================================================
# 5) MAP SERVER DOCKER
# =============================================================
echo -e "${BLUE}[5/${TOTAL_STEPS}] Harita + Waypoint servisi baslatiliyor...${NC}"
docker rm -f talos-map-server 2>/dev/null
docker run -d --rm \
    --name talos-map-server \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -v "$SCRIPT_DIR/fixes/waypoint_pub.py:/app/waypoint_pub.py:ro" \
    talos-map-server:latest
sleep 3
if docker ps --filter name=talos-map-server --format '{{.Names}}' | grep -q talos-map-server; then
    echo -e "${GREEN}[+] Harita + Waypoint servisi aktif${NC}"
else
    echo -e "${RED}[X] Harita servisi baslatilamadi!${NC}"
    exit 1
fi

# =============================================================
# 6) HEDEF TESLIMI DOCKER (matplotlib GUI)
# =============================================================
echo -e "${BLUE}[6/${TOTAL_STEPS}] Hedef teslimi baslatiliyor (matplotlib GUI)...${NC}"
docker rm -f hedef_teslimi 2>/dev/null
xhost +local:docker 2>/dev/null
docker run -d --rm \
    --name hedef_teslimi \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$SCRIPT_DIR/fixes/hedef_yoneticisi.py:/app/hedef_yoneticisi.py:ro" \
    hedef-yoneticisi:latest
sleep 2
if docker ps --filter name=hedef_teslimi --format '{{.Names}}' | grep -q hedef_teslimi; then
    echo -e "${GREEN}[+] Hedef teslimi aktif (matplotlib penceresi acilacak)${NC}"
else
    echo -e "${RED}[X] Hedef teslimi baslatilamadi!${NC}"
    exit 1
fi

# =============================================================
# 7) ENGEL (OBSTACLE) NODE DOCKER (GPU)
# =============================================================
echo -e "${BLUE}[7/${TOTAL_STEPS}] Engel algilama baslatiliyor (GPU)...${NC}"
mkdir -p "$SCRIPT_DIR/logs"
docker rm -f engel-node 2>/dev/null
docker run -d --rm \
    --name engel-node \
    --network host \
    --gpus all \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -v "$SCRIPT_DIR/fixes/engel_node_fixed.py:/app/engel_node_fixed.py:ro" \
    -v "$SCRIPT_DIR/fixes/pointcloud_obstacle_publisher.py:/app/pointcloud_obstacle_publisher.py:ro" \
    otonom-arac:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/cart/center_laser/scan scan:=/converted_scan _min_height:=-1.2 _max_height:=1.0 _range_min:=0.45 _range_max:=100.0 & sleep 2 && stdbuf -oL python3 -u /app/engel_node_fixed.py & stdbuf -oL python3 -u /app/pointcloud_obstacle_publisher.py && wait"
sleep 3
if docker ps --filter name=engel-node --format '{{.Names}}' | grep -q engel-node; then
    docker logs -f engel-node > "$SCRIPT_DIR/logs/engel_node.log" 2>&1 &
    echo -e "${GREEN}[+] Engel algilama aktif (log: logs/engel_node.log)${NC}"
else
    echo -e "${RED}[X] Engel algilama baslatilamadi!${NC}"
    exit 1
fi

# =============================================================
# 8) TRAFFIC DOCKER (GPU)
# =============================================================
echo -e "${BLUE}[8/${TOTAL_STEPS}] Trafik isareti algilama baslatiliyor (GPU)...${NC}"
docker rm -f traffic-node 2>/dev/null
docker run -d --rm \
    --name traffic-node \
    --network host \
    --gpus all \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -v "$SCRIPT_DIR/fixes/yolov8_ros_node_fixed.py:/app/yolov8_ros_node_fixed.py:ro" \
    --entrypoint bash \
    traffic_docker:latest \
    -c "source /opt/ros/noetic/setup.bash && python3 -u /app/yolov8_ros_node_fixed.py"
sleep 3
if docker ps --filter name=traffic-node --format '{{.Names}}' | grep -q traffic-node; then
    docker logs -f traffic-node > "$SCRIPT_DIR/logs/traffic_node.log" 2>&1 &
    echo -e "${GREEN}[+] Trafik isareti algilama aktif (log: logs/traffic_node.log)${NC}"
else
    echo -e "${RED}[X] Trafik isareti algilama baslatilamadi!${NC}"
    exit 1
fi

# =============================================================
# 9) LANE FOLLOWER (Python, yerel)
# =============================================================
echo -e "${BLUE}[9/${TOTAL_STEPS}] Serit takip baslatiliyor (yerel python)...${NC}"
python3 -u "$SCRIPT_DIR/lane/scripts/lane_follow_node.py" > "$SCRIPT_DIR/logs/lane_node.log" 2>&1 &
LANE_PID=$!
sleep 1
if kill -0 $LANE_PID 2>/dev/null; then
    echo -e "${GREEN}[+] Serit takip aktif (PID: $LANE_PID, log: logs/lane_node.log)${NC}"
else
    echo -e "${RED}[X] Serit takip baslatilamadi!${NC}"
    exit 1
fi

# =============================================================
# 10) KARAR (DECISION) NODE DOCKER
# =============================================================
echo -e "${BLUE}[10/${TOTAL_STEPS}] Karar dugumu baslatiliyor...${NC}"
docker rm -f karar-node 2>/dev/null
docker run -d --rm \
    --name karar-node \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -v "$SCRIPT_DIR/fixes/karar.py:/app/karar.py:ro" \
    karar-node:latest
sleep 2
if docker ps --filter name=karar-node --format '{{.Names}}' | grep -q karar-node; then
    docker logs -f karar-node > "$SCRIPT_DIR/logs/karar_node.log" 2>&1 &
    echo -e "${GREEN}[+] Karar dugumu aktif (log: logs/karar_node.log)${NC}"
else
    echo -e "${RED}[X] Karar dugumu baslatilamadi!${NC}"
    exit 1
fi

# =============================================================
# 11) CAN KOPRULERI + GORSELLISTIRICI
# =============================================================
echo -e "${BLUE}[11/${TOTAL_STEPS}] CAN kopruleri baslatiliyor...${NC}"

# CAN -> Gazebo
python3 -u "$HILMI_TALOS/can_to_talos_cart.py" &
BRIDGE_PID=$!
sleep 0.5
if kill -0 $BRIDGE_PID 2>/dev/null; then
    echo -e "${GREEN}  [+] CAN->Gazebo koprusu aktif (PID: $BRIDGE_PID)${NC}"
else
    echo -e "${RED}  [X] CAN->Gazebo koprusu baslatilamadi!${NC}"
    exit 1
fi

# Gazebo -> CAN
mkdir -p "$HILMI_TALOS/logs"
python3 -u "$HILMI_TALOS/talos_state_to_can.py" > "$HILMI_TALOS/logs/state_to_can.log" 2>&1 &
STATE_PID=$!
sleep 0.5
if kill -0 $STATE_PID 2>/dev/null; then
    echo -e "${GREEN}  [+] Gazebo->CAN koprusu aktif (PID: $STATE_PID)${NC}"
else
    echo -e "${RED}  [X] Gazebo->CAN koprusu baslatilamadi!${NC}"
    exit 1
fi

# Gorsellestirici
python3 -u "$HILMI_TALOS/can_visualizer.py" &
VIS_PID=$!
sleep 0.3
echo -e "${GREEN}  [+] CAN gorsellestirici aktif (PID: $VIS_PID)${NC}"

# =============================================================
# 12) TALOS CONTROLLER DOCKER (on plan)
# =============================================================
echo -e "${BLUE}[12/${TOTAL_STEPS}] Talos kontrolcu baslatiliyor...${NC}"
docker rm -f talos-controller 2>/dev/null

echo ""
echo -e "${GREEN}[+] TUM BILESENLER AKTIF${NC}"
echo -e "${CYAN}======================================================${NC}"
echo -e "${CYAN}  konum-server       => /konum (Pose2D)${NC}"
echo -e "${CYAN}  talos-map-server   => /map + /waypoint${NC}"
echo -e "${CYAN}  hedef_teslimi      => /hedef (String: x,y)${NC}"
echo -e "${CYAN}  engel-node         => engel algilama (GPU)${NC}"
echo -e "${CYAN}  traffic-node       => trafik isareti (GPU)${NC}"
echo -e "${CYAN}  lane (python)      => serit takip (yerel)${NC}"
echo -e "${CYAN}  karar-node         => karar dugumu${NC}"
echo -e "${CYAN}  talos-controller   => CAN kontrol${NC}"
echo -e "${CYAN}------------------------------------------------------${NC}"
echo -e "${YELLOW}  Ctrl+C ile tum sistemi kapatabilirsiniz${NC}"
echo -e "${CYAN}======================================================${NC}"
echo ""

docker run --rm \
    --name talos-controller \
    --network host \
    --privileged \
    --cap-add NET_ADMIN \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_IP=127.0.0.1 \
    -v "$HILMI_TALOS/logs:/app/logs" \
    -v "$HILMI_TALOS/control.py:/app/control.py:ro" \
    talos-control:latest \
    control.py "$@"
