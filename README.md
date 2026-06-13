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

### 2. `~/talos-sim` simülasyon workspace'ini kur (ZORUNLU ÖN ADIM)

Bu stack, TALOS Gazebo simülasyonunu süren ayrı bir ROS workspace'ine (`~/talos-sim`)
bağımlıdır. `cart_sim.msg` (özellikle `Decision` ve `cart_control` mesajları) buradan
gelir; container'lar `~/talos-sim/devel`'i bind-mount eder. Bu workspace `~/talos-sim`
yolunda kurulu ve **derlenmiş** olmadan stack çalışmaz.

```bash
# TALOS simülasyon workspace'i ~/talos-sim altında olmalı (takım ana deposundan edinilir).
# Yerleştirdikten sonra mesajları derle:
cd ~/talos-sim
catkin_make
source devel/setup.bash
```

> `~/talos-sim/devel/lib/python3/dist-packages/cart_sim/msg/_Decision.py` oluştuysa
> mesajlar derlenmiş demektir. Bu dosya yoksa `karar-node`/`engel-node` `/karar_decision`
> topic'ini yayınlamaz (sadece `/karar` String).

### 3. Bu repo'yu klonla

```bash
cd ~
git clone git@github.com:uludagai-club/talos26_ws.git
```

> `~/talos26_ws/` klasörü oluşur. `~/talos-sim` ile karıştırma; container mount'ları
> `~/talos-sim/devel`'i mutlak yolla bulur, bu repo nereye klonlanırsa klonlansın çalışır.

### 4. Docker image (TEK image)

Image'ı elle yüklemen **gerekmez** — `.tar` dağıtımı kaldırıldı. **Tüm servisler tek bir
`talos-all:latest` image'ını kullanır** ve ilk `./baslat.sh` çalıştığında `Dockerfile.all`'dan
otomatik **build** edilir (repo kendi kendine yeter):

`talos-all:latest` (`Dockerfile.all`) — `konum`, `talos-map-server`, `hedef-teslimi`,
`engel-node`, `karar-node`, `traffic-node`, `safe-zone-detector`, `can-bridge`,
`state-bridge`, `talos-controller`, `can-visualizer` — **11 servisin hepsi**. Eski 6
prebuilt imajın (`konum`, `talos-map-server`, `hedef-yoneticisi`, `otonom-arac`,
`karar-node`, `traffic_docker`) ve `talos-control:latest`'in yerini alır. Tüm Python kodu
bind-mount edildiğinden imaj sadece ROS/pip çalışma-zamanı bağımlılıklarını taşır; kod
değişince rebuild gerekmez, `docker compose restart <servis>` yeter.

İstersen baştan elle build edebilirsin:

```bash
docker build -t talos-all:latest -f Dockerfile.all .
```

> Not: `talos-all` `ultralytics` (torch + opencv) içerdiğinden ilk build birkaç GB indirir.
> GPU servisleri (`engel-node`, `traffic-node`, `safe-zone-detector`) için NVIDIA Container
> Toolkit kurulu olmalı.

---

## Her Oturumda Sistemi Başlatma

```bash
cd ~/talos26_ws

# Tek komut: vcan0 + X11 + roscore + eksik image build + tüm servisler.
./baslat.sh
```

`baslat.sh` kanonik giriş noktasıdır — `setup-vcan.sh`, host `roscore`, image build,
`docker compose --profile gui up -d` ve log streaming'i kendisi yapar; `Ctrl+C` ile
her şeyi temizleyerek kapatır. `docker compose up` zincirini elle kurmana gerek yok.

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

### Node ekleme / arkadaş node'unu güncelleme (TEK İMAJ KURALI)

Bu repo **tek `talos-all:latest` imajı** kullanır. Bir node'u güncellerken/eklerken
**ayrı `Dockerfile` veya yeni image OLUŞTURMA** — aksi halde repo tekrar çok-imaja döner.
Kurala göre:

1. **Sadece Python kodu değiştiyse** (mantık/parametre/fix): hiçbir şey yapma.
   Kod bind-mount'lu → `git pull` + `docker compose restart <servis>`. Rebuild yok.

2. **Node'a YENİ bir bağımlılık (pip/apt) gerekiyorsa**: paketi `Dockerfile.all`'a ekle,
   sonra `talos-all`'ı yeniden build et:
   ```bash
   docker build -t talos-all:latest -f Dockerfile.all .
   docker compose down && ./baslat.sh
   ```
   (Örnek: `karar_bt` `py_trees` gerektirdi → `Dockerfile.all`'a `py_trees==2.2.3` eklendi,
   ayrı `karar-bt` imajı kullanılmadı.)

3. **Yeni servis ekliyorsan** `docker-compose.yml`'a şu kalıpla ekle — `image: talos-all:latest`
   + `entrypoint: bash` + `command: -c "source /opt/ros/noetic/setup.bash && python3 -u /app/<node>.py"`,
   ve node'unu `volumes` ile `/app` altına bind-mount et.

> **Arkadaşın node'unu kendi `Dockerfile`'ı ile push ettiyse** (örn. `karar_bt/Dockerfile`):
> merge sırasında o servisin `build:`/ayrı image'ını kaldır, `image: talos-all:latest` yap;
> bağımlılıklarını `Dockerfile.all`'a taşı. Kod ve `command`'ı aynen koru.

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
| `levha_tespiti/yolov8_ros/scripts/best.pt` | `traffic-node` | Hayır (bind-mount, image içindeki modeli ezer) |
| `karar_bt/` (BT karar düğümü) | `karar-node` | Hayır |
| `hilmi-talos/control.py` | `talos-controller` | Hayır |
| `hilmi-talos/can_to_talos_cart.py` | `can-bridge` | Hayır |
| `hilmi-talos/talos_state_to_can.py` | `state-bridge` | Hayır |
| `prototip-1/safe_zone_detector.py` | `safe-zone-detector` | Hayır |
| `Dockerfile.all` | **tüm 11 servis** | **Evet** (`docker build -t talos-all:latest -f Dockerfile.all .`) |

Rebuild yalnızca `Dockerfile.all` değişince gerekir:
```bash
docker build -t talos-all:latest -f Dockerfile.all .
docker compose down && ./baslat.sh
```

---

## Bileşenler

Tüm servisler tek `talos-all:latest` image'ını kullanır; ayrım `command` ile yapılır.

| Servis | Açıklama |
|--------|----------|
| `konum-server` | Konum/lokalizasyon |
| `talos-map-server` | Harita + waypoint yayıcı (`/waypoint`) |
| `hedef-teslimi` | Hedef yöneticisi (GUI) |
| `engel-node` | Engel algılama + `pointcloud_to_laserscan` (GPU) |
| `traffic-node` | Trafik işareti algılama (GPU) |
| `safe-zone-detector` | Güvenli alan tespiti (GPU, prototip) |
| `karar-node` | Karar düğümü |
| `can-bridge` | CAN → Gazebo köprüsü |
| `state-bridge` | Gazebo → CAN köprüsü |
| `talos-controller` | Ana sürüş kontrolcüsü |
| `can-visualizer` *(opsiyonel, `--profile gui`)* | CAN görselleştirici |

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
