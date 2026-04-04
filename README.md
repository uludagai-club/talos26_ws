# TALOS Otonom Sürüş - Tam Sistem

Gazebo simülasyonunda TALOS aracını kontrol eden tam otonom sürüş sistemi.
Tüm bileşenler Docker container olarak çalışır, Python dosyaları bind mount ile
anında güncellenir — kod değişikliği için rebuild gerekmez.

---

## İlk Kurulum (Bir Kez)

### 1. SSH anahtarını GitHub'a ekle

```bash
# SSH anahtarı oluştur (zaten varsa atla)
ssh-keygen -t ed25519 -C "email@example.com"

# Anahtarı görüntüle ve kopyala
cat ~/.ssh/id_ed25519.pub
```

Kopyaladığın anahtarı GitHub'a ekle:
**GitHub → Settings → SSH and GPG keys → New SSH key** → yapıştır → Save

Bağlantıyı test et:
```bash
ssh -T git@github.com
# "Hi kullanıcıadı! You've successfully authenticated..." yazmalı
```

### 2. Repo'yu klonla

```bash
cd ~
git clone git@github.com:uludagai-club/talos26_ws.git
```

> `talos26_ws/` klasörü oluşur. `~/talos-sim` ayrı bir repo, karıştırma.

### 3. Docker image'larını yükle

Image'lar `.tar` dosyaları ile dağıtılır (USB/Google Drive). Hepsini aynı klasöre koy, sonra:

```bash
docker load -i konum.tar
docker load -i talos-map-waypoint.tar
docker load -i hedef_yoneticisi_v1.tar
docker load -i engel_node.tar
docker load -i karar_node_x86.tar
docker load -i traffic_docker.tar
```

`talos-control` image'ı yoksa ilk `docker compose up`'ta otomatik build edilir.

### 4. `~/talos-sim` workspace'i build et (cart_sim.msg için)

```bash
cd ~/talos-sim
catkin_make
```

---

## Her Oturumda Sistemi Başlatma

```bash
cd ~/talos26_ws

# 1. vcan0 ve X11 hazırla (her oturumda bir kez)
./setup-vcan.sh

# 2. Host'ta roscore başlat
source ~/talos-sim/devel/setup.bash
roscore &

# 3. Sistemi başlat
docker compose up

# Görselleştirici de istersen
docker compose --profile gui up
```

Durdurmak için `Ctrl+C`.

---

## Kod Güncelleme Akışı

### Başkasının değişikliklerini almak

```bash
cd ~/talos26_ws
git pull
```

Dosyalar bind mount ile çalıştığı için pull sonrası değişen servisi yeniden başlatmak yeter:

```bash
# Sadece o servisi yeniden başlat (tüm sistemi durdurmana gerek yok)
docker compose restart hedef-teslimi
```

### Kendi değişikliğini göndermek

```bash
cd ~/talos26_ws

# Hangi dosyaları değiştirdiğine bak
git status

# Değişiklikleri stage'le
git add fixes/hedef_yoneticisi.py   # hangi dosyaysa

# Commit
git commit -m "fix: kısa açıklama"

# GitHub'a gönder
git push
```

> Birden fazla kişi aynı dosyayı değiştirdiyse `git pull` sırasında conflict çıkabilir.
> O zaman dosyayı aç, `<<<<<<` işaretli kısımları çöz, `git add` + `git commit` yap.

---

## Hangi Dosyayı Değiştirince Ne Olur

| Değişen Dosya | İlgili Servis | Rebuild Gerekir mi? |
|---------------|---------------|---------------------|
| `fixes/hedef_yoneticisi.py` | `hedef-teslimi` | Hayır |
| `fixes/konum.py` | `konum-server` | Hayır |
| `fixes/waypoint_pub.py` | `talos-map-server` | Hayır |
| `fixes/engel_node_fixed.py` | `engel-node` | Hayır |
| `fixes/pointcloud_obstacle_publisher.py` | `engel-node` | Hayır |
| `fixes/yolov8_ros_node_fixed.py` | `traffic-node` | Hayır |
| `fixes/karar.py` | `karar-node` | Hayır |
| `hilmi-talos/control.py` | `talos-controller` | Hayır |
| `hilmi-talos/can_to_talos_cart.py` | `can-bridge` | Hayır |
| `hilmi-talos/talos_state_to_can.py` | `state-bridge` | Hayır |
| `lane/scripts/lane_follow_node.py` | `lane-follower` | Hayır |
| `hilmi-talos/Dockerfile` | `talos-controller` | **Evet** |
| `hilmi-talos/requirements.txt` | `talos-controller` | **Evet** |

Rebuild gereken durum:
```bash
docker compose build talos-controller
docker compose up talos-controller
```

---

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
| `can-visualizer` *(opsiyonel)* | `talos-control:latest` | CAN görselleştirici |

---

## Sorun Giderme

**vcan0 bulunamadı:**
```bash
./setup-vcan.sh
```

**ROS master'a bağlanamıyor:**
```bash
source ~/talos-sim/devel/setup.bash && roscore
```

**GUI açılmıyor (hedef, lane, visualizer):**
```bash
xhost +local:docker
```

**GPU container başlamıyor:**
```bash
# NVIDIA Container Toolkit kurulu mu?
nvidia-container-cli info
```

**`cart_sim` modülü bulunamıyor (can-bridge / state-bridge):**
```bash
cd ~/talos-sim && catkin_make
```

**`git push` reddedildi (başkası push etmiş):**
```bash
git pull --rebase
git push
```

---

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
