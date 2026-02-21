# TALOS Otonom Sürüş Kontrol Sistemi

Gazebo simülasyonunda TALOS aracını CAN Bus üzerinden kontrol eden waypoint takip sistemi.

## Ön Gereksinimler

| Gereksinim | Versiyon |
|------------|----------|
| Ubuntu | 20.04 LTS |
| ROS | Noetic |
| Docker | 20.10+ |
| talos-sim workspace | Build edilmiş (`catkin_make`) |

## Hızlı Kurulum

```bash
# 1. talos-sim workspace'i kur (henüz yapılmadıysa)
cd ~/talos-sim
catkin_make
source devel/setup.bash

# 2. Tek seferlik kurulum
cd scripts/talos26_ws/hilmi-talos
chmod +x setup.sh start.sh
./setup.sh

# 3. Simülasyonu başlat (ayrı terminal)
source ~/talos-sim/devel/setup.bash
roslaunch cart_sim cart_sim.launch

# 4. Kontrol sistemini başlat
cd ~/talos-sim/scripts/talos26_ws/hilmi-talos
./start.sh

# 5. Görevi başlat (ayrı terminal)
cansend vcan0 500#01
```

## Sistem Mimarisi

```
Host Makine                                         Docker Container
=============                                       ================

 Gazebo (cart_sim)                                   control.py
   /base_pose_ground_truth ──┐                         │
   /cart  ◄──────────────────┼──── can_to_talos_cart.py │
                             │         ▲                │
                             │         │ CAN (vcan0)    │
                             │         ▼                ▼
                  talos_state_to_can.py ◄────────► CAN Bus (vcan0)
                             │
                  can_visualizer.py (görsel panel)
```

**Docker** sadece `control.py` çalıştırır (waypoint takip + PID kontrol).
**Host** üzerinde çalışan köprü scriptleri ROS bağımlılığı nedeniyle Docker dışındadır:
- `can_to_talos_cart.py`: CAN komutlarını ROS `/cart` topic'ine çevirir
- `talos_state_to_can.py`: Gazebo araç durumunu CAN mesajlarına çevirir

## Dosya Yapısı

```
hilmi-talos/
├── control.py              # Ana kontrol (Docker içinde) - waypoint takip, PID
├── can_to_talos_cart.py     # CAN -> ROS köprüsü (Host'ta çalışmalı!)
├── talos_state_to_can.py    # ROS -> CAN köprüsü (Host'ta çalışmalı!)
├── can_decoder.py           # CAN mesaj çözümleyici
├── can_visualizer.py        # Gerçek zamanlı CAN görsel paneli
├── entrypoint.sh            # Docker entrypoint scripti
├── Dockerfile               # Docker image tanımı
├── docker-compose.yml       # Docker compose ayarları
├── setup.sh                 # Tek seferlik kurulum scripti
├── start.sh                 # Tüm sistemi başlatan script
└── logs/                    # Çalışma logları
```

## Nasıl Çalışır?

1. `start.sh` sırasıyla tüm bileşenleri başlatır (vcan0, köprüler, görselleştirici, Docker)
2. Container başladığında CAN `0x500` mesajı bekler
3. `cansend vcan0 500#01` ile görev başlatılır
4. Araç waypoint'lere doğru hareket eder
5. Her waypoint'e ulaşıldığında sıradakine geçer
6. Tüm waypoint'ler tamamlandığında durur ve yeni komut bekler

## CAN Protokolü

| ID | Yön | Açıklama | Veri |
|----|-----|----------|------|
| 0x100 | Controller -> Araç | Gaz, Fren, Vites | byte0: gaz, byte1: fren, byte2: vites |
| 0x201 | Controller -> Araç | Direksiyon açısı | int16 (little-endian) |
| 0x301 | Araç -> Controller | Araç hızı | float32 km/h |
| 0x302 | Araç -> Controller | IMU verileri | yaw, pitch, roll |
| 0x400 | Controller -> Araç | Sinyal lambası | byte0: sol/sağ/dörtlü |
| 0x500 | Dış -> Controller | Başlatma komutu | 0x01=başla |

### CAN Mesajlarını İzleme

```bash
# Tüm mesajları izle
candump vcan0

# Belirli ID filtrele
candump vcan0,100:7FF

# Manuel başlatma
cansend vcan0 500#01
```

## Konfigürasyon

### Waypoint'leri Değiştirme

`control.py` dosyasındaki `DEFAULT_WAYPOINTS` listesini düzenleyin:

```python
DEFAULT_WAYPOINTS = [
    {"x": -4.70, "y": -34.31, "type": "normal"},
    {"x": -1.82, "y": -34.31, "type": "obstacle"},  # Engel - şerit değiştirir
    # ...
]
```

### Hız Ayarı

`control.py` içindeki `MAX_SPEED_KMH` değerini değiştirin (varsayılan: 5.0 km/h).

## Sorun Giderme

### Araç hareket etmiyor

1. **Tüm bileşenler çalışıyor mu?**
   ```bash
   pgrep -f rosmaster       || echo "ROS Master YOK"
   pgrep -f gzserver        || echo "Gazebo YOK"
   ip link show vcan0       || echo "vcan0 YOK"
   pgrep -f can_to_talos    || echo "CAN Köprüsü YOK"
   docker ps --filter name=talos-controller --format "{{.Status}}" || echo "Docker YOK"
   ```

2. **Başlatma komutu gönderildi mi?**
   ```bash
   cansend vcan0 500#01
   ```

3. **ROS topic'leri aktif mi?**
   ```bash
   rostopic echo /cart -n1
   rostopic echo /base_pose_ground_truth -n1
   ```

### vcan0 oluşturulamadı

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### Container başlamıyor / sürekli yeniden başlıyor

1. vcan0 kurulumunu kontrol edin
2. ROS master çalışıyor mu: `rostopic list`
3. Logları kontrol edin: `docker logs talos-controller`

### Birden fazla controller çakışması

Joystick veya başka bir node `/cart` topic'ine yazıyorsa:
```bash
rosnode kill /controller /beemobs_sim_bridge
```

### ROS bağlantı sorunu

`ROS_MASTER_URI` ve `ROS_IP` değerlerini kontrol edin:
```bash
echo $ROS_MASTER_URI   # http://localhost:11311 olmalı
echo $ROS_IP            # 127.0.0.1 olmalı
```

## Log Dosyaları

`logs/` klasöründe:
- `control_*.log` - Docker kontrol logları
- `state_to_can.log` - Gazebo->CAN köprüsü logları
- `data_*.csv` - Araç verileri (konum, hız, direksiyon)

```bash
# Docker loglarını canlı izle
docker logs talos-controller -f
```
