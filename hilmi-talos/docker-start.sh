#!/bin/bash

# Renkler
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}  TALOS Waypoint Follower - Docker${NC}"
echo -e "${BLUE}======================================================${NC}"

# 1. vcan0 oluştur (host'ta)
echo -e "${BLUE}[1/3] Virtual CAN ayarlanıyor...${NC}"
if ! ip link show vcan0 > /dev/null 2>&1; then
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
    echo -e "${GREEN}[✓] vcan0 oluşturuldu${NC}"
else
    sudo ip link set up vcan0 2>/dev/null
    echo -e "${GREEN}[✓] vcan0 aktif${NC}"
fi

# 2. Docker image oluştur
echo -e "${BLUE}[2/3] Docker image oluşturuluyor...${NC}"
docker-compose build

# 3. Çalıştır
echo -e "${BLUE}[3/3] Konteynerler başlatılıyor...${NC}"
docker-compose up

echo -e "${GREEN}[✓] Tamamlandı${NC}"
