# TALOS CAN Waypoint Control System

Gazebo simülasyonunda TALOS aracını CAN Bus üzerinden kontrol eden otonom waypoint takip sistemi.

## Dosyalar

| Dosya | Açıklama |
|-------|----------|
| `can_waypoint_follower.py` | PID kontrollü otonom waypoint takip sistemi |
| `can_to_talos_cart.py` | CAN -> Gazebo köprüsü |
| `talos_state_to_can.py` | Gazebo -> CAN köprüsü |
| `can_decoder.py` | CAN mesaj çözücü |
| `can_visualizer.py` | Matplotlib gösterge paneli |
| `start_waypoint_demo.sh` | Demo başlatıcı script |

## Kurulum

```bash
# Virtual CAN
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Bağımlılıklar
pip3 install -r requirements.txt
```

## Kullanım

```bash
# Demo başlat
./start_waypoint_demo.sh

# Özel waypoint'ler ile
./start_waypoint_demo.sh "-10,-34 -5,-34 -5,-30"
```

## CAN Mesajları

| ID | Yön | Açıklama |
|----|-----|----------|
| 0x100 | TX | Gaz/Fren/Vites |
| 0x201 | TX | Direksiyon |
| 0x301 | RX | Araç hızı |

## Parametreler

```python
MAX_SPEED_KMH = 5.0       # Maksimum hız
ARRIVAL_THRESHOLD = 1.5   # Waypoint toleransı (m)
MAX_STEER_ANGLE = 30.0    # Direksiyon limiti (derece)
```

## Bağımlılıklar

- ROS Noetic
- Python 3.8+
- python-can, numpy, matplotlib
