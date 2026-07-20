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
    └── talos26_ws/                # Docker servisleri (bu repo) — iş-alanına göre düzenli
        ├── baslat.sh              # Tek komutla tüm sistemi başlatır
        ├── docker-compose.yml
        ├── Dockerfile.all         # TEK runtime imajı (talos-all:latest)
        ├── setup-vcan.sh
        ├── control/              # Kontrol + CAN köprüsü: control.py, can_*, can_decoder, can_visualizer
        ├── hedef/               # hedef_yoneticisi.py (D* hedef/planlama)
        ├── konum/               # konum.py, konum_yoneticisi.py (lokalizasyon)
        ├── karar/               # Behavior Tree karar düğümü (karar_bt_node.py)
        ├── lidar/               # LİDAR engel birimi: engel_node + pointcloud + talos_obstacle_detector (C++) + ground_filter
        ├── algi/                # KAMERA perception:
        │   ├── serit/           #   lane_follow + models/best.pt
        │   ├── levha/           #   yolov8_ros (trafik levha/ışık)
        │   ├── yaya_gecidi/     #   yaya_gecidi_node + best.pt
        │   └── park_durak/      #   park/durak alan algılama
        ├── maps/                # my_map.* + waypoint_pub.py (map-server)
        ├── missions/            # geojson + graf dosyaları
        └── talos_common/        # paylaşılan loglama kütüphanesi
```

> **Not:** `fixes/` ve `hilmi-talos/` kaldırıldı (2026-06-22); node'lar iş-alanı klasörlerine,
> `hilmi-talos/` → `control/`, `algi/engel/` → `lidar/` taşındı. **13 servisin hepsi** tek
> `talos-all:latest` imajını kullanır (ayrım `command`'la). map-server da Kerem'in eski prebuilt
> imajından talos-all'a taşındı (2026-06-22) — artık harici imaj/`.tar` gerekmez.

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

### Sorun 4: LİDAR engel paketi (talos_obstacle_detector) catkin tarafından görülmüyor

C++ engel dedektörü `scripts/talos26_ws/lidar/talos_obstacle_detector` altında ama catkin onu
`~/talos-sim/src` altında görmeli. Fresh clone'da bu symlink yoktur → `catkin_make` paketi atlar,
`baslat.sh` engel algılamayı sessizce devre dışı bırakır (legacy /engel* fallback). Düzelt:

```bash
# Symlink (bir kez) — talos_obstacle_detector'ı catkin'e tanıt
ln -sfn ~/talos-sim/scripts/talos26_ws/lidar/talos_obstacle_detector ~/talos-sim/src/talos_obstacle_detector
# C++ bağımlılığı (jsk_recognition_msgs)
sudo apt install -y ros-noetic-jsk-recognition-msgs
```

### Derleme (4 düzeltmeden sonra)

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

### 3.2. Docker Image (TEK imaj)

`.tar` dağıtımı kaldırıldı. **Tüm servisler tek `talos-all:latest` imajını kullanır**;
ilk `./baslat.sh` çalıştığında `Dockerfile.all`'dan otomatik build edilir (repo kendi
kendine yeter, harici `.tar` gerekmez). İstersen elle:
```bash
docker build -t talos-all:latest -f Dockerfile.all .
```

**Image kontrol:**
```bash
docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}"
```

Beklenen **tek** image: `talos-all:latest` (13 servisin hepsi; map-server dahil — 2026-06-22'de
talos-all'a taşındı: `ros-noetic-map-server` + `maps/my_map.*` + `maps/final_graph.yaml`). Harici
`.tar` GEREKMEZ. Kod compose'da bind-mount'lu olduğundan imaj yalnızca ROS/pip çalışma-zamanı
bağımlılıklarını taşır; kod değişince rebuild gerekmez.

### 3.3. Lane (Şerit) Model Dosyası

Lane follower `algi/serit/models/best.pt` yolunu bekler (compose bunu mount eder).
Model repo ile gelir; eksikse `algi/serit/best.pt`'den kopyala:
```bash
mkdir -p ~/talos-sim/scripts/talos26_ws/algi/serit/models
cp ~/talos-sim/scripts/talos26_ws/algi/serit/best.pt ~/talos-sim/scripts/talos26_ws/algi/serit/models/best.pt
```

### 3.4. Log Dizini İzin Sorunu

Docker container root olarak log dosyası oluşturur, sonraki çalıştırmalarda host erişim hatası verir:

**Hata:** `Erişim engellendi` (control/logs/ veya logs/)

**Çözüm:**
```bash
sudo chown -R $USER:$USER ~/talos-sim/scripts/talos26_ws/control/logs/ ~/talos-sim/scripts/talos26_ws/logs/
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
> GPU servisleri (engel-node, traffic-node) tek `talos-all:latest` imajını kullanır.

### Yöntem 2: Docker Compose ile

```bash
cd ~/talos-sim/scripts/talos26_ws
sudo bash setup-vcan.sh
docker compose up
```

> **Not:** `baslat.sh` daha güvenilirdir çünkü sıralı başlatma ve sağlık kontrolleri yapar.

### Canlı parametre değiştirme (restart'sız)

Sistem çalışırken bir servisin parametresini değiştirmek için container'ı
yeniden başlatmana gerek yok:

```bash
# 1. Dosyayı aç, ilgili servis bölümünde satırın yorumunu kaldır, değeri yaz:
nano ~/talos-sim/scripts/talos26_ws/config/canli_params.yaml
#    ör.  control:
#           MAX_SPEED_KMH: 3.0

# 2. Kaydet — ~1 sn içinde uygulanır. Doğrulamak için servis loguna bak:
docker logs -f talos-controller | grep canli_params
#    [canli_params][control] MAX_SPEED_KMH: 5.0 → 3.0
```

Satırı tekrar yoruma alınca kod içindeki varsayılana döner. Varsayılanlar her
servisin dosyasının en üstündeki "AYARLANABİLİR PARAMETRELER" bloğundadır.
İstisnalar: `karar-node` (params.yaml + restart) ve YAML'da `(RESTART)`
işaretli parametreler. Detay: `README.md → Canlı Parametreler`.

---

## 5. Sık Karşılaşılan Sorunlar

| Sorun | Çözüm |
|-------|-------|
| `CMP0100` cmake hatası | `if(POLICY CMP0100)` koşuluna al (bkz. Bölüm 2.1) |
| `handbrake` attribute hatası | `cart_control.msg`'ye `float64 handbrake` ekle + `catkin_make` (bkz. Bölüm 2.2) |
| Araç hareket etmiyor | `CartPlugin.cc`'de handbrake düzeltmeleri yap + `catkin_make` + Gazebo yeniden başlat (bkz. Bölüm 2.3) |
| `Erişim engellendi` (logs/) | `sudo chown -R $USER:$USER control/logs/ logs/` |
| vcan0 bulunamadı | `sudo bash setup-vcan.sh` |
| ROS master'a bağlanamıyor | Host'ta `roscore` çalışıyor olmalı |
| GUI açılmıyor | `xhost +local:docker` |
| `cart_sim.msg` bulunamıyor | `cd ~/talos-sim && catkin_make` |
| GPU container başlamıyor | `nvidia-container-cli info` ile toolkit kontrol |
| `algi/serit/models/best.pt` yok | `best.pt`'yi `algi/serit/models/` altına kopyala |
| `git push` reddedildi | `git pull --rebase && git push` |
| Launch dosyası bulunamıyor | `demo.launch` değil, `cart_sim.launch` kullanılmalı |

---

## 6. Dosya - Servis Eşleşmesi

| Dosya | Servis | Rebuild? |
|-------|--------|----------|
| `hedef/hedef_yoneticisi.py` | hedef-teslimi | Hayır |
| `konum/konum.py` | konum-server | Hayır |
| `maps/waypoint_pub.py` | talos-map-server | Hayır |
| `lidar/engel_node_fixed.py` | engel-node | Hayır |
| `karar/` (BT karar düğümü) | karar-node | Hayır |
| `algi/levha/yolov8_ros_node_fixed.py` | traffic-node | Hayır |
| `algi/serit/lane_follow_node_fixed.py` | lane-follower | Hayır |
| `algi/yaya_gecidi/yaya_gecidi_node.py` | yaya-gecidi-node | Hayır |
| `algi/park_durak/park_durak_node.py` | park-durak-node | Hayır |
| `control/control.py` | talos-controller | Hayır |
| `control/can_to_talos_cart.py` | can-bridge | Hayır |
| `control/talos_state_to_can.py` | state-bridge | Hayır |
| `Dockerfile.all` | **tüm servisler** | **Evet** (`docker build -t talos-all:latest -f Dockerfile.all .`) |
| `src/cart_sim/plugins/CartPlugin.cc` | Gazebo plugin | **Evet** (`catkin_make` + Gazebo restart) |
| `src/cart_sim/msg/cart_control.msg` | ROS mesajı | **Evet** (`catkin_make`) |

Rebuild gerektiğinde:
```bash
# Tek imaj (yalnız Dockerfile.all değişince — yeni pip/apt bağımlılığı):
docker build -t talos-all:latest -f Dockerfile.all .
docker compose down && ./baslat.sh

# Gazebo plugin veya msg değişikliği için:
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
