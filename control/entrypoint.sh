#!/bin/bash
set -e

echo "=============================================="
echo "  TALOS Control System - Starting..."
echo "=============================================="

# vcan0 arayüzünü kontrol et ve oluştur
if ! ip link show vcan0 &>/dev/null; then
    echo "[*] vcan0 bulunamadı, oluşturuluyor..."

    # vcan modülü yüklü mü kontrol et
    if ! lsmod | grep -q vcan; then
        echo "[!] UYARI: vcan kernel modülü yüklü değil!"
        echo "[!] Host makinede şu komutu çalıştırın: sudo modprobe vcan"
        echo "[!] Devam ediliyor..."
    fi

    # vcan0 oluşturmayı dene
    ip link add dev vcan0 type vcan 2>/dev/null || true
    ip link set up vcan0 2>/dev/null || true
fi

# vcan0 durumunu göster
if ip link show vcan0 &>/dev/null; then
    echo "[+] vcan0 hazır"
else
    echo "[!] UYARI: vcan0 oluşturulamadı!"
    echo "[!] Host makinede şu komutları çalıştırın:"
    echo "    sudo modprobe vcan"
    echo "    sudo ip link add dev vcan0 type vcan"
    echo "    sudo ip link set up vcan0"
fi

# ROS ortamını yükle
source /opt/ros/noetic/setup.bash

echo "[*] Python script başlatılıyor..."
exec python3 "$@"
