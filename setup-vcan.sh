#!/bin/bash
# docker compose up öncesinde bir kez çalıştır
set -e

echo "[*] vcan0 kuruluyor..."
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0
echo "[+] vcan0 hazır"

echo "[*] X11 erişimi veriliyor (GUI container'ları için)..."
xhost +local:docker 2>/dev/null || true
echo "[+] Hazır. Şimdi: docker compose up"
