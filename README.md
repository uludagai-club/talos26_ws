# TALOS Otonom Sürüş - Tam Sistem

Gazebo simülasyonunda TALOS aracını kontrol eden tam otonom sürüş sistemi.
Tüm bileşenler Docker container olarak çalışır, Python dosyaları bind mount ile
anında güncellenir — kod değişikliği için rebuild gerekmez.

## Bileşenler

| Servis | Image | Açıklama |
|--------|-------|----------|
| `konum-server` | `konum:latest` | Konum/lokalizasyon |
| `talos-map-server` | `talos-map-server:latest` | Harita + waypoint yayıcı |
| `hedef-teslimi` | `hedef-yoneticisi:latest` | Hedef yöneticisi (GUI) |
| `engel-node` | `otonom-arac:latest` | Engel algılama (GPU) |
| `traffic-node` | `traffic_docker:latest` | Trafik işareti algılama (GPU) |
| `lane-follower` | `traffic_docker:latest` | Şerit takip (GPU) |
| `karar-node` | `karar-node:latest` | Karar düğümü |
| `can-bridge` | `talos-control:latest` | CAN → Gazebo köprüsü |
| `state-bridge` | `talos-control:latest` | Gazebo → CAN köprüsü |
| `talos-controller` | `talos-control:latest` | Ana sürüş kontrolcüsü |
| `can-visualizer` | `talos-control:latest` | CAN görselleştirici (opsiyonel) |

## Gereksinimler

- Docker Engine + Docker Compose v2
- NVIDIA Container Toolkit (GPU node'ları için)
- ROS Noetic (host'ta `roscore` çalışmalı)
- `~/talos-sim` workspace build edilmiş olmalı (cart_sim.msg için)

## Kurulum

### 1. Docker image'larını yükle

Image'lar `.tar` dosyaları ile dağıtılır (USB/Google Drive):

```bash
docker load -i konum.tar
docker load -i talos-map-waypoint.tar
docker load -i hedef_yoneticisi_v1.tar
docker load -i engel_node.tar
docker load -i karar_node_x86.tar
docker load -i traffic_docker.tar
```

`talos-control` image'ı yoksa otomatik build edilir (aşağıya bak).

### 2. Repo'yu klonla

```bash
git clone git@github.com:uludagai-club/talos26_ws.git
cd talos26_ws
```

### 3. vcan0 ve X11 hazırla (her oturumda bir kez)

```bash
chmod +x setup-vcan.sh
./setup-vcan.sh
```

### 4. Sistemi başlat

```bash
# Host'ta roscore çalıştır
roscore &

# Tüm servisleri başlat
docker compose up

# CAN görselleştirici ile birlikte
docker compose --profile gui up
```

## Geliştirme Akışı

### Python dosyası değiştirme (rebuild gerekmez)
```bash
# Örn: fixes/hedef_yoneticisi.py düzenle
nano fixes/hedef_yoneticisi.py

# Sadece o servisi yeniden başlat
docker compose restart hedef-teslimi
```

### Düzenlenebilir Python dosyaları ve servisleri
| Dosya | Servis |
|-------|--------|
| `fixes/konum.py` | `konum-server` |
| `fixes/waypoint_pub.py` | `talos-map-server` |
| `fixes/hedef_yoneticisi.py` | `hedef-teslimi` |
| `fixes/engel_node_fixed.py` | `engel-node` |
| `fixes/pointcloud_obstacle_publisher.py` | `engel-node` |
| `fixes/yolov8_ros_node_fixed.py` | `traffic-node` |
| `fixes/karar.py` | `karar-node` |
| `hilmi-talos/control.py` | `talos-controller` |
| `hilmi-talos/can_to_talos_cart.py` | `can-bridge` |
| `hilmi-talos/talos_state_to_can.py` | `state-bridge` |
| `lane/scripts/lane_follow_node.py` | `lane-follower` |

### Dockerfile değişikliği (rebuild gerekir)
```bash
# hilmi-talos/Dockerfile düzenle
nano hilmi-talos/Dockerfile

# Rebuild + yeniden başlat
docker compose build talos-controller
docker compose up talos-controller
```

### Log takibi
```bash
# Tüm loglar
docker compose logs -f

# Belirli servis
docker compose logs -f talos-controller
docker compose logs -f engel-node
```

## Sorun Giderme

**vcan0 bulunamadı:**
```bash
./setup-vcan.sh
```

**ROS master'a bağlanamıyor:**
```bash
roscore
```

**GUI açılmıyor (hedef, lane, visualizer):**
```bash
xhost +local:docker
```

**GPU container başlamıyor:**
```bash
# NVIDIA Container Toolkit kurulu mu?
nvidia-container-cli info
# Kurulum: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
```

**cart_sim.msg bulunamıyor (can-bridge/state-bridge):**
```bash
# ~/talos-sim workspace build edilmeli
cd ~/talos-sim && catkin_make
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
