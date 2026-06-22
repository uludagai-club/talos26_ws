#!/bin/bash
# TALOS Otonom Suruş Sistemi - Tek Seferlik Kurulum
set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}  TALOS Otonom Suruş Sistemi - Kurulum${NC}"
echo -e "${BLUE}======================================================${NC}"
echo ""

# 1. Sistem bagimliliklari
echo -e "${BLUE}[1/4] Sistem bagimliliklari kuruluyor...${NC}"
sudo apt-get update -qq
sudo apt-get install -y -qq can-utils python3-pip iproute2 ros-noetic-tf2-ros python3-tk > /dev/null
if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
    pip3 install -r "$SCRIPT_DIR/requirements.txt" 2>/dev/null
else
    pip3 install python-can numpy matplotlib 2>/dev/null
fi
echo -e "${GREEN}[+] Sistem bagimliliklari kuruldu${NC}"

# 2. vcan kernel modulunu kalici yap
echo -e "${BLUE}[2/4] vcan kernel modulu ayarlaniyor...${NC}"
if ! grep -q "^vcan$" /etc/modules 2>/dev/null; then
    echo "vcan" | sudo tee -a /etc/modules > /dev/null
    echo -e "${GREEN}[+] vcan modulu /etc/modules'a eklendi (kalici)${NC}"
else
    echo -e "${GREEN}[+] vcan modulu zaten /etc/modules'da${NC}"
fi

# vcan modulunu simdi yukle
sudo modprobe vcan 2>/dev/null || true
echo -e "${GREEN}[+] vcan kernel modulu aktif${NC}"

# 3. Docker image yukle veya build et
echo -e "${BLUE}[3/4] Docker image hazirlaniyor...${NC}"
if docker image inspect talos-control:latest > /dev/null 2>&1; then
    echo -e "${GREEN}[+] Docker image zaten mevcut${NC}"
elif [ -f "$SCRIPT_DIR/talos_control.tar" ]; then
    echo -e "${YELLOW}[*] Docker image tar dosyasindan yukleniyor...${NC}"
    docker load -i "$SCRIPT_DIR/talos_control.tar"
    echo -e "${GREEN}[+] Docker image yuklendi${NC}"
elif [ -f "$SCRIPT_DIR/Dockerfile" ]; then
    echo -e "${YELLOW}[*] Docker image build ediliyor...${NC}"
    docker build -t talos-control:latest "$SCRIPT_DIR"
    echo -e "${GREEN}[+] Docker image build edildi${NC}"
else
    echo -e "${RED}[X] Docker image bulunamadi!${NC}"
    echo -e "${RED}    talos_control.tar veya Dockerfile olmali${NC}"
    exit 1
fi

# 4. ROS workspace kontrolu
echo -e "${BLUE}[4/4] ROS workspace kontrol ediliyor...${NC}"
if [ -f "$PROJECT_ROOT/devel/setup.bash" ]; then
    echo -e "${GREEN}[+] ROS workspace hazir ($PROJECT_ROOT)${NC}"
else
    echo -e "${YELLOW}[!] ROS workspace build edilmemis!${NC}"
    echo -e "${YELLOW}    Asagidaki komutlari calistirin:${NC}"
    echo -e "${YELLOW}      cd $PROJECT_ROOT${NC}"
    echo -e "${YELLOW}      catkin_make${NC}"
    echo -e "${YELLOW}      source devel/setup.bash${NC}"
fi

echo ""
echo -e "${GREEN}======================================================${NC}"
echo -e "${GREEN}  Kurulum tamamlandi!${NC}"
echo -e "${GREEN}======================================================${NC}"
echo ""
echo -e "Sistemi baslatmak icin:"
echo -e "  ${BLUE}1.${NC} Simulasyonu baslatin:  roslaunch cart_sim cart_sim.launch"
echo -e "  ${BLUE}2.${NC} Kontrolcuyu baslatin: ${YELLOW}./start.sh${NC}"
echo ""
echo -e "${YELLOW}Not: Her terminalde 'source $PROJECT_ROOT/devel/setup.bash' calistirmayi unutmayin${NC}"
