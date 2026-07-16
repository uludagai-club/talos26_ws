# YZT | TALOS Otonom Araç Takımı

Bursa Uludağ Üniversitesi Yapay Zeka Topluluğu çatısı altında faaliyet gösteren **YZT | TALOS Otonom Araç Takımı** olarak, Robotaksi-Binek Otonom Araç Yarışması — Hazır Araç Kategorisi için hazırladığımız repoyu paylaşmaktan büyük gurur duyuyoruz.

2020 yılından 2025 yılına kadar her yıl kesintisiz olarak Teknofest Robotaksi Binek Otonom Araç Yarışması Finalisti olma başarısını göstermiş köklü bir ekip olarak, edindiğimiz tüm tecrübe ve mühendislik becerilerini bu yılki simülasyon mimarimize yansıttık. Geliştirdiğimiz sistemde makine öğrenmesi algoritmalarını; ZED 2 Kamera, LiDAR, GPS ve IMU gibi zengin sensör füzyonları ile besleyerek uluslararası standartlarda güvenli sürüş çözümleri üretiyoruz.

## 🎯 Misyonumuz ve Vizyonumuz

Toplumun, devletin ve sanayinin yapay zeka alanındaki sorunlarına çözüm ortağı olmak amacıyla, yerli ve milli teknoloji hamlesine katkıda bulunmak en büyük motivasyonumuz. Simülasyon aşamasında başarıyla tamamladığımız bu görevleri, gerçek piste taşımak için sabırsızlanıyoruz!

## 🔗 YZT-TALOS Takımını Sosyal Medyada Takip Edin

- **Instagram:** [@talos.team](https://instagram.com/talos.team) / [@uludagaiclub](https://instagram.com/uludagaiclub)
- **LinkedIn:** [linkedin.com/company/talosteam](https://www.linkedin.com/company/talosteam/)
- **Web Sitemiz:** [yapayzekatoplulugu.uludag.edu.tr](https://yapayzekatoplulugu.uludag.edu.tr)

---

# TALOS Otonom Sürüş - Tam Sistem

Gazebo simülasyonunda TALOS aracını kontrol eden tam otonom sürüş sistemi.
Tüm bileşenler Docker container olarak çalışır, Python dosyaları bind mount ile
anında güncellenir — kod değişikliği için rebuild gerekmez.

---

## İlk Kurulum (Bir Kez)

### 0. Sistem paketleri

```bash
sudo apt update && sudo apt install -y can-utils
```

> `can-utils` (`cansend` / `candump`) olmadan `baslat.sh` ham CAN yedeğini alamaz
> (`[!] candump yok` uyarısı) ve GUI'siz başlatma (`cansend`) mümkün olmaz.

GPU servisleri (`engel-node`, `traffic-node`, `lane-follower`, `yaya-gecidi-node`) için
**NVIDIA Container Toolkit** gerekir. Kurulu değilse `docker compose up` şu hatayı verir
ve bu dört servis hiç başlamaz:

```
Error response from daemon: could not select device driver "nvidia" with capabilities: [[gpu]]
```

Kontrol: `nvidia-container-cli info`. NVIDIA GPU'n yoksa bu dört servis çalışmaz; araç
waypoint takibiyle yine sürer ama engel algılama, şerit takibi ve levha algısı olmaz.

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

> **Bu adım atlanırsa araç HİÇ yürümez.** `control/can_to_talos_cart.py` `cart_sim.msg`'i
> korumasız import eder → `can-bridge` container'ı ImportError ile anında ölür. `can-bridge`
> ölünce controller'ın CAN frame'leri Gazebo'ya hiç ulaşmaz: her şey yeşil görünür, buton
> çalışır, log hedef basar, ama araç yerinde durur (`Hız: 0.0`, mesafe sabit).
> `state-bridge` ve `karar-node` aynı import'u `try/except` ile sardığı için ölmez, sadece
> uyarı verir — bu yüzden arıza yalnız `can-bridge`'in eksikliğinden anlaşılır.

Derlemenin başarılı olduğunu doğrula:

```bash
ls ~/talos-sim/devel/lib/python3/dist-packages/cart_sim/msg/_Decision.py
```

> Bu dosya yoksa mesajlar derlenmemiş demektir (`/karar_decision` da yayınlanmaz,
> sadece `/karar` String gelir).

Simülasyonun tek başına açıldığını burada doğrula (stack'e geçmeden önce):

```bash
source ~/talos-sim/devel/setup.bash
roslaunch cart_sim cart_sim.launch
```

Gazebo penceresi açılıp araç pistte görünmeli. Görünmüyorsa Python node'ları `+x`
iznini kaybetmiş olabilir: `cd ~/talos-sim/src/cart_sim/scripts && chmod +x *.py`
(aynısı `src/cart_sim/nodes/` için).

### 3. Bu repo'yu klonla (`~/talos-sim/scripts/` altına — ZORUNLU)

```bash
mkdir -p ~/talos-sim/scripts
cd ~/talos-sim/scripts
git clone git@github.com:uludagai-club/talos26_ws.git
```

> **ÖNEMLİ:** Repo MUTLAKA `~/talos-sim/scripts/talos26_ws/` altında olmalı. `baslat.sh`,
> `~/talos-sim`'i (dolayısıyla `~/talos-sim/devel`'i) `SCRIPT_DIR/../..` ile bulur — repo
> başka yere (örn. `~/talos26_ws`) klonlanırsa `devel`/`cart_sim.msg` bulunamaz, köprü
> container'ları (`can-bridge`, `state-bridge`, `karar-node`) ImportError ile çöker.

### 4. Docker image (TEK image)

**13 servisin hepsi** tek `talos-all:latest` image'ını kullanır ve ilk `./baslat.sh`
çalıştığında `Dockerfile.all`'dan otomatik **build** edilir (repo kendi kendine yeter).
map-server da (eskiden Kerem'in ayrı prebuilt imajı) 2026-06-22'de talos-all'a taşındı —
artık harici imaj/`.tar` gerekmez; `/map` + güncel `/waypoint` (644-node graf) talos-all'dan gelir.

`talos-all:latest` (`Dockerfile.all`) — `konum`, `talos-map-server`, `hedef-teslimi`,
`engel-node`, `traffic-node`, `lane-follower`, `yaya-gecidi-node`, `park-durak-node`,
`karar-node`, `can-bridge`, `state-bridge`, `talos-controller`, `can-visualizer` — **13 servisin hepsi**. Eski 6
prebuilt imajın (`konum`, `talos-map-server`, `hedef-yoneticisi`, `otonom-arac`,
`karar-node`, `traffic_docker`) ve `talos-control:latest`'in yerini alır. Tüm Python kodu
bind-mount edildiğinden imaj sadece ROS/pip çalışma-zamanı bağımlılıklarını taşır; kod
değişince rebuild gerekmez, `docker compose restart <servis>` yeter.

İstersen baştan elle build edebilirsin:

```bash
docker build -t talos-all:latest -f Dockerfile.all .
```

> Not: `talos-all` `ultralytics` (torch + opencv) içerdiğinden ilk build birkaç GB indirir.
> GPU servisleri (`engel-node`, `traffic-node`, `lane-follower`, `yaya-gecidi-node`) için NVIDIA Container
> Toolkit kurulu olmalı.

---

## Her Oturumda Sistemi Başlatma (3 ADIM — üçü de zorunlu)

İki nokta yeni gelenleri sürekli yanıltıyor, en baştan bilinsin:

1. **`baslat.sh` Gazebo'yu BAŞLATMAZ.** Simülasyonu ayrı bir terminalde sen açarsın;
   `baslat.sh` yalnız Docker stack'ini (otonom sürüş yazılımını) ayağa kaldırır.
2. **Son adımdaki butona basmadan araç YÜRÜMEZ.** Her şey yeşil görünse, 13 servis
   de aksa bile araç `0x500` başlatma frame'i gelene kadar yerinde bekler.

### Adım 1 — Gazebo simülasyonu (Terminal 1)

```bash
source ~/talos-sim/devel/setup.bash
roslaunch cart_sim cart_sim.launch
```

Gazebo penceresi açılmalı ve araç pistte görünmeli. (`roscore`'u `roslaunch` kendisi
başlatır; zaten çalışıyorsa ona bağlanır.)

### Adım 2 — Docker stack (Terminal 2)

```bash
cd ~/talos-sim/scripts/talos26_ws

# Tek komut: vcan0 + X11 + roscore (yoksa) + eksik image build + 13 servis + log akışı.
./baslat.sh
```

`baslat.sh` kanonik giriş noktasıdır — `setup-vcan.sh`, host `roscore`, image build,
`docker compose --profile gui up -d` ve log streaming'i kendisi yapar; `Ctrl+C` ile
her şeyi temizleyerek kapatır. `docker compose up` zincirini elle kurmana gerek yok.

`TUM BILESENLER AKTIF` bannerından sonra log şu satırda **durur ve bekler**.
Bu bir hata DEĞİLDİR — Adım 3'ü bekliyor:

```
talos-controller  |   [DURUM] Başlatma komutu bekleniyor (CAN ID 0x500)...
```

### Adım 3 — `ROTA BAŞLAT` butonuna bas (araç bu olmadan YÜRÜMEZ)

`baslat.sh` ile birlikte **can-visualizer** penceresi açılır. Penceredeki üstte duran
yeşil **`ROTA BAŞLAT`** butonuna bas. Buton CAN'e `0x500` byte0=1 gönderir; `control.py`
görevi ancak bu frame'i alınca başlatır (o ana kadar `mission_started=False` ile
bloklayıcı bir bekleme döngüsünde durur, araca tek bir gaz frame'i bile gitmez).

Bastıktan sonra controller logunda görmen gerekenler:

```
talos-controller  | >>> CAN Başlatma komutu alındı (0x500) <<<
talos-controller  | GÖREV BAŞLATILIYOR! /hedef bekleniyor...
```

Ardından `/hedef` gelince araç hareket eder.

GUI açılmadıysa (X11 yok, uzak makine, `xhost` verilmemiş) aynı frame'i elle gönderebilirsin
(`can-utils` gerekir — Adım 0):

```bash
cansend vcan0 500#0100000000000000
```

---

## Kod Güncelleme Akışı

### Başkasının değişikliklerini almak

```bash
cd ~/talos-sim/scripts/talos26_ws
git pull
```

Dosyalar bind mount ile çalıştığı için pull sonrası değişen servisi yeniden başlatmak yeter:

```bash
# Sadece o servisi yeniden başlat (tüm sistemi durdurmana gerek yok)
docker compose restart hedef-teslimi
```

### Kendi değişikliğini göndermek

```bash
cd ~/talos-sim/scripts/talos26_ws

# Hangi dosyaları değiştirdiğine bak
git status

# Değişiklikleri stage'le
git add hedef/hedef_yoneticisi.py   # hangi dosyaysa

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
| `hedef/hedef_yoneticisi.py` | `hedef-teslimi` | Hayır |
| `konum/konum.py` | `konum-server` | Hayır |
| `maps/waypoint_pub.py` | `talos-map-server` | Hayır |
| `lidar/engel_node_fixed.py` | `engel-node` | Hayır |
| `lidar/pointcloud_obstacle_publisher.py` | `engel-node` | Hayır |
| `algi/levha/yolov8_ros_node_fixed.py` | `traffic-node` | Hayır |
| `algi/levha/yolov8_ros/scripts/best.pt` | `traffic-node` | Hayır (bind-mount, image içindeki modeli ezer) |
| `algi/serit/lane_follow_node_fixed.py` | `lane-follower` | Hayır |
| `algi/yaya_gecidi/yaya_gecidi_node.py` | `yaya-gecidi-node` | Hayır |
| `algi/park_durak/park_durak_node.py` | `park-durak-node` | Hayır |
| `karar/` (BT karar düğümü) | `karar-node` | Hayır |
| `control/control.py` | `talos-controller` | Hayır |
| `control/can_to_talos_cart.py` | `can-bridge` | Hayır |
| `control/talos_state_to_can.py` | `state-bridge` | Hayır |
| `Dockerfile.all` | **tüm 13 servis** | **Evet** (`docker build -t talos-all:latest -f Dockerfile.all .`) |

Rebuild yalnızca `Dockerfile.all` değişince gerekir:
```bash
docker build -t talos-all:latest -f Dockerfile.all .
docker compose down && ./baslat.sh
```

---

## Bileşenler

Tüm 13 servis tek `talos-all:latest` image'ını kullanır; ayrım `command` ile yapılır.

| Servis | Açıklama |
|--------|----------|
| `konum-server` | Konum/lokalizasyon (`konum/`) |
| `talos-map-server` | Harita + waypoint yayıcı `/waypoint` (`maps/`) |
| `hedef-teslimi` | Hedef yöneticisi / D* planlama, GUI (`hedef/`) |
| `engel-node` | Engel algılama + `pointcloud_to_laserscan` (GPU, `lidar/`) |
| `traffic-node` | Trafik levha/ışık algılama (GPU, `algi/levha/`) |
| `lane-follower` | Şerit takip (GPU, `algi/serit/`) |
| `yaya-gecidi-node` | Yaya geçidi algılama (GPU, `algi/yaya_gecidi/`) |
| `park-durak-node` | Park/durak alanı algılama (`algi/park_durak/`) |
| `karar-node` | Behavior Tree karar düğümü (`karar/`) |
| `can-bridge` | CAN → Gazebo köprüsü (`control/`) |
| `state-bridge` | Gazebo → CAN köprüsü (`control/`) |
| `talos-controller` | Ana sürüş kontrolcüsü (`control/`) |
| `can-visualizer` *(opsiyonel, `--profile gui`)* | CAN görselleştirici (`control/`) |

---

## Sorun Giderme

**Araç hiç yürümüyor — log `[DURUM] Başlatma komutu bekleniyor (CAN ID 0x500)...`da duruyor:**

En sık yaşanan durum ve **bir hata değil**: Adım 3 atlanmış. can-visualizer penceresindeki
yeşil `ROTA BAŞLAT` butonuna bas. Pencere yoksa:
```bash
cansend vcan0 500#0100000000000000
```
Doğrulama: controller logunda `>>> CAN Başlatma komutu alındı (0x500) <<<` görünmeli.

**`0x500` alındı, hedef de geliyor, ama araç kımıldamıyor (`Hız: 0.0`, mesafe sabit):**

```
talos-controller | >>> CAN Başlatma komutu alındı (0x500) <<<
talos-controller | Hedef (-11.8,-34.3) | Mesafe: 3.2m | Hız: 0.0/5.0 km/h    ← mesafe hiç düşmüyor
```

Neredeyse her zaman **`can-bridge` çökmüştür** → controller'ın CAN frame'leri Gazebo'ya
ulaşmıyor. Önce ayakta mı bak (`docker compose ps` yalnız ÇALIŞANLARI listeler; listede
`talos-can-bridge` yoksa çökmüş demektir):

```bash
docker compose ps -a | grep can-bridge
docker compose logs can-bridge | tail -30
```

Log'da `ModuleNotFoundError: No module named 'cart_sim'` görüyorsan sebep derlenmemiş
`devel`'dir (Kurulum Adım 2) — bu, `karar-node`'un `cart_sim.msg.Decision import edilemedi`
uyarısıyla **aynı kök nedendir**:

```bash
cd ~/talos-sim && catkin_make
cd ~/talos-sim/scripts/talos26_ws && docker compose restart can-bridge
```

**`0x500` alındı ama log `GÖREV BAŞLATILIYOR! /hedef bekleniyor...`da duruyor:**

Bu sefer gerçekten `/hedef` gelmiyor demektir — `hedef-teslimi` ayakta mı bak:
```bash
rostopic echo -n1 /hedef      # "x,y" gelmeli
docker compose logs hedef-teslimi | tail -30
```

**`./setup-vcan.sh: Permission denied` / `sudo: setup-vcan.sh: command not found`:**

`baslat.sh` vcan0'ı zaten kendisi kurar — `[+] vcan0 olusturuldu` gördüysen bu scripti
elle çalıştırmana gerek YOK. Yine de gerekirse `sudo` PATH'te `.` aramaz, `./` şart:
```bash
chmod +x setup-vcan.sh      # "-x" DEĞİL, "+x"
sudo ./setup-vcan.sh        # "sudo setup-vcan.sh" değil
# ya da izinle uğraşmadan:
sudo bash setup-vcan.sh
```

**Gazebo açık değil / araç sahnede yok:** `baslat.sh` simülasyonu başlatmaz, Adım 1'i
ayrı terminalde sen çalıştırmalısın:
```bash
source ~/talos-sim/devel/setup.bash && roslaunch cart_sim cart_sim.launch
```

**`karar-node` logunda `cart_sim.msg.Decision import edilemedi` uyarısı:**

`~/talos-sim/devel` derlenmemiş ya da bayat:
```bash
cd ~/talos-sim && catkin_make
```
Sistem bu uyarıyla da sürer (`/karar` String olarak yayınlanır, control onu dinler) ama
`/karar_decision` (yapılandırılmış `Decision`) yayınlanmaz. Kontrol: `~/talos-sim/devel/lib/python3/dist-packages/cart_sim/msg/_Decision.py` var mı?

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
