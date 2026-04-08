# TALOS Simülasyon - Kurulum ve Sorun Giderme Rehberi

Tarih: 2026-04-08

---

## 1. Dizin Yapısı

```
~/talos-sim/                      # Ana catkin workspace
├── src/cart_sim/                  # Gazebo simülasyonu
│   ├── msg/cart_control.msg      # Araç kontrol mesajı (handbrake dahil)
│   └── plugins/CartPlugin.cc     # Gazebo araç fiziği (handbrake desteği gerekli)
├── devel/                         # catkin_make çıktısı (cart_sim.msg burada)
├── build/
└── scripts/
    └── talos26_ws/                # Docker servisleri (bu repo)
        ├── baslat.sh              # Tek komutla tüm sistemi başlatır
        ├── docker-compose.yml
        ├── setup-vcan.sh
        ├── fixes/                 # Bind mount edilen düzeltilmiş Python dosyaları
        ├── hilmi-talos/           # Kontrol sistemi + Dockerfile
        └── lane/
            ├── scripts/           # lane_follow_node.py
            └── models/            # best.pt (oluşturulmalı, bkz. 3.3)
```

---

## 2. Simülasyon Derleme (catkin_make)

Temiz bir klonlama sonrası **3 düzeltme** gerekir. Hepsi yapıldıktan sonra tek bir `catkin_make` yeterlidir.

### Sorun 1: `CMP0100` Policy Hatası

**Hata:**
```
CMake Error at cart_sim/CMakeLists.txt:4 (cmake_policy):
  Policy "CMP0100" is not known to this version of CMake.
```

**Sebep:** `cmake_policy(SET CMP0100 NEW)` CMake 3.18+ gerektirir, Ubuntu 20.04'te CMake 3.16 var.

**Çözüm:** `~/talos-sim/src/cart_sim/CMakeLists.txt` 4. satır:

```cmake
# ESKİ:
cmake_policy(SET CMP0100 NEW)

# YENİ:
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()
```

### Sorun 2: `'cart_control' object has no attribute 'handbrake'`

**Sebep:** `cart_control.msg` dosyasında `handbrake` alanı tanımlı değil. `can_to_talos_cart.py` ve `talos_state_to_can.py` bu alanı kullanıyor.

**Çözüm:** `~/talos-sim/src/cart_sim/msg/cart_control.msg` dosyasının sonuna ekle:

```
# Range 0 to 1, 1 is handbrake fully engaged
float64 handbrake
```

Tam dosya içeriği:
```
Header header
float64 throttle
float64 brake
float64 steer
uint8 NO_COMMAND=0
uint8 NEUTRAL=1
uint8 FORWARD=2
uint8 REVERSE=3
uint8 shift_gears

# Range 0 to 1, 1 is handbrake fully engaged
float64 handbrake
```

### Sorun 3: Araç Hareket Etmiyor (El Freni Sorunu) - KRİTİK

**Belirti:** Tüm servisler çalışıyor, `/cart` topic'ine mesaj gidiyor, throttle > 0 ama araç kımıldamıyor.

**Sebep:** `CartPlugin.cc` dosyasında iki hata var:
1. `handbrakePercent` başlangıç değeri `1.0` (el freni çekili başlıyor)
2. `/cart` mesajındaki `handbrake` alanı plugin tarafından okunmuyor

**Çözüm:** `~/talos-sim/src/cart_sim/plugins/CartPlugin.cc` dosyasında:

**a)** Satır ~250: Başlangıç değerini değiştir:
```cpp
// ESKİ:
public: double handbrakePercent = 1.0;

// YENİ:
public: double handbrakePercent = 0.0;
```

**b)** `OnCartCommand` fonksiyonunda (satır ~335), throttle okunduktan sonra handbrake okuma ekle:
```cpp
  // Throttle command
  double throttle = ignition::math::clamp(msg->throttle, 0.0, 1.0);
  this->dataPtr->gasPedalPercent = throttle;

  // Handbrake command  ← BU BLOĞU EKLE
  double handbrake = ignition::math::clamp(msg->handbrake, 0.0, 1.0);
  this->dataPtr->handbrakePercent = handbrake;

  switch (msg->shift_gears)
```

### Derleme (3 düzeltmeden sonra)

```bash
cd ~/talos-sim
source /opt/ros/noetic/setup.bash
catkin_make
```

> **ÖNEMLİ:** Plugin değişikliği sonrası Gazebo'nun yeniden başlatılması gerekir (plugin .so dosyası yeniden yüklenmeli).

---

## 3. Docker Kurulumu

### 3.1. Repo Konumu

Repo **mutlaka** `~/talos-sim/scripts/talos26_ws/` altına klonlanmalı:
```bash
mkdir -p ~/talos-sim/scripts
cd ~/talos-sim/scripts
git clone git@github.com:uludagai-club/talos26_ws.git
```

### 3.2. Docker Image'lar

`.tar` dosyaları ile yüklenir:
```bash
docker load -i konum.tar
docker load -i talos-map-waypoint.tar
docker load -i hedef_yoneticisi_v1.tar
docker load -i engel_node.tar
docker load -i karar_node_x86.tar
docker load -i traffic_docker.tar
```

`talos-control:latest` yoksa ilk çalıştırmada otomatik build edilir.

**Image kontrol:**
```bash
docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}"
```

Beklenen image'lar: `konum`, `talos-map-server`, `hedef-yoneticisi`, `otonom-arac`, `traffic_docker`, `karar-node`, `talos-control`

### 3.3. Lane Model Dosyası

Lane follower `lane/models/best.pt` yolunu bekler. Repo'da `best.pt` sadece `lane/` kökünde gelir:
```bash
mkdir -p ~/talos-sim/scripts/talos26_ws/lane/models
cp ~/talos-sim/scripts/talos26_ws/lane/best.pt ~/talos-sim/scripts/talos26_ws/lane/models/best.pt
```

### 3.4. Log Dizini İzin Sorunu

Docker container root olarak log dosyası oluşturur, sonraki çalıştırmalarda host erişim hatası verir:

**Hata:** `Erişim engellendi` (hilmi-talos/logs/)

**Çözüm:**
```bash
sudo chown -R $USER:$USER ~/talos-sim/scripts/talos26_ws/hilmi-talos/logs/
```

---

## 4. Sistemi Başlatma

### Yöntem 1: `baslat.sh` ile (Önerilen)

```bash
# 1. Roscore başlat (ayrı terminal)
source ~/talos-sim/devel/setup.bash
roscore

# 2. Simülasyonu başlat (ayrı terminal)
source ~/talos-sim/devel/setup.bash
roslaunch cart_sim cart_sim.launch

# 3. Tüm sistemi başlat (ayrı terminal)
cd ~/talos-sim/scripts/talos26_ws
bash baslat.sh
```

> `baslat.sh` vcan0 kurulumunu, tüm Docker container'ları ve CAN köprülerini otomatik başlatır.
> Lane follower GPU container olarak çalışır (`traffic_docker:latest`).

### Yöntem 2: Docker Compose ile

```bash
cd ~/talos-sim/scripts/talos26_ws
sudo bash setup-vcan.sh
docker compose up
```

> **Not:** `baslat.sh` daha güvenilirdir çünkü sıralı başlatma ve sağlık kontrolleri yapar.

---

## 5. Sık Karşılaşılan Sorunlar

| Sorun | Çözüm |
|-------|-------|
| `CMP0100` cmake hatası | `if(POLICY CMP0100)` koşuluna al (bkz. Bölüm 2.1) |
| `handbrake` attribute hatası | `cart_control.msg`'ye `float64 handbrake` ekle + `catkin_make` (bkz. Bölüm 2.2) |
| Araç hareket etmiyor | `CartPlugin.cc`'de handbrake düzeltmeleri yap + `catkin_make` + Gazebo yeniden başlat (bkz. Bölüm 2.3) |
| `Erişim engellendi` (logs/) | `sudo chown -R $USER:$USER hilmi-talos/logs/` |
| vcan0 bulunamadı | `sudo bash setup-vcan.sh` |
| ROS master'a bağlanamıyor | Host'ta `roscore` çalışıyor olmalı |
| GUI açılmıyor | `xhost +local:docker` |
| `cart_sim.msg` bulunamıyor | `cd ~/talos-sim && catkin_make` |
| GPU container başlamıyor | `nvidia-container-cli info` ile toolkit kontrol |
| `lane/models/best.pt` yok | `best.pt`'yi `lane/models/` altına kopyala |
| `git push` reddedildi | `git pull --rebase && git push` |
| Launch dosyası bulunamıyor | `demo.launch` değil, `cart_sim.launch` kullanılmalı |

---

## 6. Dosya - Servis Eşleşmesi

| Dosya | Servis | Rebuild? |
|-------|--------|----------|
| `fixes/hedef_yoneticisi.py` | hedef-teslimi | Hayır |
| `fixes/konum.py` | konum-server | Hayır |
| `fixes/waypoint_pub.py` | talos-map-server | Hayır |
| `fixes/engel_node_fixed.py` | engel-node | Hayır |
| `fixes/karar.py` | karar-node | Hayır |
| `fixes/yolov8_ros_node_fixed.py` | traffic-node | Hayır |
| `hilmi-talos/control.py` | talos-controller | Hayır |
| `hilmi-talos/can_to_talos_cart.py` | can-bridge (yerel python) | Hayır |
| `hilmi-talos/talos_state_to_can.py` | state-bridge (yerel python) | Hayır |
| `lane/scripts/lane_follow_node.py` | lane-follower (Docker GPU) | Hayır |
| `hilmi-talos/Dockerfile` | talos-controller | **Evet** |
| `hilmi-talos/requirements.txt` | talos-controller | **Evet** |
| `src/cart_sim/plugins/CartPlugin.cc` | Gazebo plugin | **Evet** (`catkin_make` + Gazebo restart) |
| `src/cart_sim/msg/cart_control.msg` | ROS mesajı | **Evet** (`catkin_make`) |

Rebuild gerektiğinde:
```bash
# Docker servisi için
docker compose build talos-controller
docker compose up talos-controller

# Gazebo plugin veya msg değişikliği için
cd ~/talos-sim && catkin_make
# Sonra Gazebo'yu yeniden başlat
```

Bind mount değişikliğinde sadece restart:
```bash
docker compose restart <servis-adı>
```

---

## 7. Hızlı Komutlar

```bash
# Tüm loglar
docker compose logs -f

# Tek servis logu
docker compose logs -f karar-node

# Tek servisi yeniden başlat
docker compose restart hedef-teslimi

# Sistemi durdur (docker compose ile başlattıysan)
docker compose down

# Çalışan container'lar
docker ps

# ROS topic'leri
rostopic list
rostopic echo /cart
```
