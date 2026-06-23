#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROJECT_ROOT="$(cd ../../.. && pwd)"

# Renkler
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}  TALOS Otonom Suruş Sistemi${NC}"
echo -e "${BLUE}  Docker + CAN Koprusu + Gorsellestirici${NC}"
echo -e "${BLUE}======================================================${NC}"

# ROS ortamini yukle
if [ -f "$PROJECT_ROOT/devel/setup.bash" ]; then
    source "$PROJECT_ROOT/devel/setup.bash"
    echo -e "${GREEN}[+] ROS ortami yuklendi${NC}"
else
    source /opt/ros/noetic/setup.bash 2>/dev/null
    echo -e "${YELLOW}[!] devel/setup.bash bulunamadi, sistem ROS kullaniliyor${NC}"
fi

# vcan0 kontrolu
echo -e "${BLUE}[1/5] Virtual CAN kontrol ediliyor...${NC}"
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

# Temizlik
cleanup() {
    echo -e "\n${BLUE}[*] Sistem kapatiliyor...${NC}"
    [ -n "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null
    [ -n "$STATE_PID" ] && kill $STATE_PID 2>/dev/null
    [ -n "$VIS_PID" ] && kill $VIS_PID 2>/dev/null
    docker stop talos-controller 2>/dev/null
    stty sane 2>/dev/null
    echo -e "${GREEN}[+] Temizlik tamamlandi${NC}"
}
trap cleanup EXIT

# CAN->Gazebo koprusu
echo -e "${BLUE}[2/5] CAN->Gazebo koprusu baslatiliyor...${NC}"
python3 -u "$SCRIPT_DIR/can_to_talos_cart.py" &
BRIDGE_PID=$!
sleep 0.5
if kill -0 $BRIDGE_PID 2>/dev/null; then
    echo -e "${GREEN}[+] CAN->Gazebo koprusu aktif (PID: $BRIDGE_PID)${NC}"
else
    echo -e "${RED}[X] CAN->Gazebo koprusu baslatılamadi!${NC}"
    exit 1
fi

# Gazebo->CAN koprusu
echo -e "${BLUE}[3/5] Gazebo->CAN koprusu baslatiliyor...${NC}"
mkdir -p "$SCRIPT_DIR/logs"
python3 -u "$SCRIPT_DIR/talos_state_to_can.py" > "$SCRIPT_DIR/logs/state_to_can.log" 2>&1 &
STATE_PID=$!
sleep 0.5
if kill -0 $STATE_PID 2>/dev/null; then
    echo -e "${GREEN}[+] Gazebo->CAN koprusu aktif (PID: $STATE_PID)${NC}"
else
    echo -e "${RED}[X] Gazebo->CAN koprusu baslatilamadi!${NC}"
    exit 1
fi

# Gorsellestirici
echo -e "${BLUE}[4/5] CAN gorsellestirici baslatiliyor...${NC}"
python3 -u "$SCRIPT_DIR/can_visualizer.py" &
VIS_PID=$!
sleep 0.3
echo -e "${GREEN}[+] Gorsellestirici aktif (PID: $VIS_PID)${NC}"

# Docker kontrolcu
echo -e "${BLUE}[5/5] Docker kontrolcu baslatiliyor...${NC}"

# Onceki container varsa kaldir
docker rm -f talos-controller 2>/dev/null

# Image yoksa yukle veya build et
if ! docker image inspect talos-control:latest > /dev/null 2>&1; then
    if [ -f "$SCRIPT_DIR/talos_control.tar" ]; then
        echo -e "${YELLOW}[*] Docker image tar dosyasindan yukleniyor...${NC}"
        docker load -i "$SCRIPT_DIR/talos_control.tar"
    else
        echo -e "${YELLOW}[*] Docker image bulunamadi, build ediliyor...${NC}"
        docker build -t talos-control:latest "$SCRIPT_DIR"
    fi
fi

echo -e "${GREEN}[+] Tum bilesemler aktif${NC}"
echo -e "${BLUE}======================================================${NC}"
echo -e "${YELLOW}  Ctrl+C ile tum sistemi kapatabilirsiniz${NC}"
echo -e "${BLUE}======================================================${NC}"
echo ""

# Docker container on planda calistir (loglar gorulsun)
docker run --rm \
    --name talos-controller \
    --network host \
    --privileged \
    --cap-add NET_ADMIN \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_IP=127.0.0.1 \
    -v "$SCRIPT_DIR/logs:/app/logs" \
    -v "$SCRIPT_DIR/control.py:/app/control.py:ro" \
    talos-control:latest \
    control.py "$@"
