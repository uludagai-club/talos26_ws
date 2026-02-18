#!/bin/bash

# Bu script'in bulunduğu dizine git
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Proje kök dizini
PROJECT_ROOT="$(cd ../.. && pwd)"

# Renkler
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}  TALOS CAN Waypoint Follower - Otonom Sürüş Sistemi${NC}"
echo -e "${BLUE}======================================================${NC}"
echo -e "${YELLOW}  Maksimum Hız: 5 km/h${NC}"
echo -e "${BLUE}======================================================${NC}"

# 0. ROS Ortamını Yükle
if [ -f "$PROJECT_ROOT/devel/setup.bash" ]; then
    source "$PROJECT_ROOT/devel/setup.bash"
    echo -e "${GREEN}[✓] ROS ortamı yüklendi${NC}"
else
    # Fallback: sistem ROS'u
    source /opt/ros/noetic/setup.bash 2>/dev/null
    echo -e "${YELLOW}[!] devel/setup.bash bulunamadı, sistem ROS kullanılıyor${NC}"
fi

# 1. vcan0 Kontrolü
echo -e "${BLUE}[1/5] Virtual CAN arayüzü kontrol ediliyor...${NC}"
if ! ip link show vcan0 > /dev/null 2>&1; then
    echo -e "${YELLOW}[!] vcan0 bulunamadı. Oluşturuluyor...${NC}"
    if sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0; then
        echo -e "${GREEN}[✓] vcan0 oluşturuldu${NC}"
    else
        echo -e "${RED}[X] vcan0 hatası!${NC}"
        exit 1
    fi
else
    sudo ip link set up vcan0 2>/dev/null
    echo -e "${GREEN}[✓] vcan0 aktif${NC}"
fi

# Temizlik
cleanup() {
    echo -e "\n${BLUE}[*] Sistem kapatılıyor...${NC}"
    [ ! -z "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null
    [ ! -z "$STATE_PID" ] && kill $STATE_PID 2>/dev/null
    [ ! -z "$VIS_PID" ] && kill $VIS_PID 2>/dev/null
    stty sane 2>/dev/null
    echo -e "${GREEN}[✓] Temizlik tamamlandı${NC}"
}
trap cleanup EXIT

# 2. CAN->Gazebo Köprüsü
echo -e "${BLUE}[2/5] CAN->Gazebo köprüsü başlatılıyor...${NC}"
python3 -u "$SCRIPT_DIR/can_to_talos_cart.py" > /dev/null 2>&1 &
BRIDGE_PID=$!
sleep 0.5
echo -e "${GREEN}[✓] CAN->Gazebo köprüsü aktif${NC}"

# 3. Gazebo->CAN Köprüsü
echo -e "${BLUE}[3/5] Gazebo->CAN köprüsü başlatılıyor...${NC}"
python3 -u "$SCRIPT_DIR/talos_state_to_can.py" > /dev/null 2>&1 &
STATE_PID=$!
sleep 0.5
echo -e "${GREEN}[✓] Gazebo->CAN köprüsü aktif${NC}"

# 4. Görselleştirici
echo -e "${BLUE}[4/5] CAN görselleştiricisi başlatılıyor...${NC}"
python3 "$SCRIPT_DIR/can_visualizer.py" > /dev/null 2>&1 &
VIS_PID=$!
echo -e "${GREEN}[✓] Görselleştirici aktif${NC}"

# 5. Waypoint Follower
echo -e "${BLUE}[5/5] Waypoint Follower başlatılıyor...${NC}"
echo ""

if [ -n "$1" ]; then
    echo -e "${GREEN}Kullanıcı waypoint'leri: $1${NC}"
    python3 "$SCRIPT_DIR/can_waypoint_follower.py" "$1"
else
    echo -e "${GREEN}Varsayılan waypoint'ler kullanılıyor${NC}"
    python3 "$SCRIPT_DIR/can_waypoint_follower.py"
fi
