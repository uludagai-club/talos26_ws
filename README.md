# TALOS CAN Waypoint Control System

Gazebo simülasyonunda TALOS aracını CAN Bus üzerinden kontrol eden otonom waypoint takip sistemi.

## Hızlı Başlangıç (Docker)

```bash
# Projeyi klonla
git clone git@github.com:uludagai-club/talos26_ws.git
cd talos26_ws

# Docker ile çalıştır
./docker-start.sh
```

Bu script otomatik olarak:
- vcan0 sanal CAN arayüzünü oluşturur
- Docker image'ları build eder
- Tüm servisleri başlatır

## Gereksinimler

- Docker & Docker Compose
- ROS Noetic (host'ta roscore çalışmalı)
- TALOS simülasyonu (`~/talos-sim` dizininde)

## Servisler

| Servis | Container | Açıklama |
|--------|-----------|----------|
| `waypoint-follower` | talos-waypoint | PID kontrollü otonom sürüş |
| `can-bridge` | talos-can-bridge | CAN -> Gazebo köprüsü |
| `state-bridge` | talos-state-bridge | Gazebo -> CAN köprüsü |
| `visualizer` | talos-visualizer | Matplotlib gösterge paneli |

## Kullanım

```bash
# Tüm servisleri başlat
./docker-start.sh

# Sadece belirli servisleri başlat
docker compose up waypoint-follower can-bridge

# Logları izle
docker logs -f talos-waypoint

# Servisleri durdur
docker compose down
```

## CAN Mesajları

| ID | Yön | Açıklama |
|----|-----|----------|
| 0x100 | TX | Gaz/Fren/Vites |
| 0x102 | TX | Park freni |
| 0x201 | TX | Direksiyon |
| 0x301 | RX | Araç hızı ve RPM |
| 0x302 | RX | IMU verileri |
| 0x303 | RX | Batarya durumu |
| 0x500 | TX | Sistem komutları (Start) |

## Dosya Yapısı

```
├── Dockerfile              # Docker image tanımı
├── docker-compose.yml      # Servis konfigürasyonu
├── docker-start.sh         # Başlatma scripti
├── requirements.txt        # Python bağımlılıkları
├── control.py   # Otonom waypoint takip
├── can_to_talos_cart.py       # CAN -> Gazebo
├── talos_state_to_can.py      # Gazebo -> CAN
├── can_decoder.py             # CAN mesaj çözücü
└── can_visualizer.py          # GUI gösterge paneli
```

## Parametreler

`control.py` içinde ayarlanabilir:

```python
MAX_SPEED_KMH = 5.0           # Maksimum hız (km/h)
ARRIVAL_THRESHOLD = 1.5       # Waypoint toleransı (m)
MAX_STEER_ANGLE = 30.0        # Direksiyon limiti (derece)
REVERSE_ANGLE_THRESHOLD = 120 # Geri vites açı eşiği
```

## Sorun Giderme

**vcan0 bulunamadı:**
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

**ROS master'a bağlanamıyor:**
```bash
# Host'ta roscore çalıştır
roscore
```

**Görselleştirme açılmıyor:**
```bash
xhost +local:docker
```
