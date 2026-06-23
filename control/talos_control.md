# talos_control - Mini Otonom Arac Kontrol Sistemi

## .tar Dosyasi Nasil Olusturuldu?

```bash
cd ~/talos-sim/scripts/talos26_ws
docker build -t talos-control:latest -f Dockerfile .
docker save -o hilmi-talos/talos_control.tar talos-control:latest
```

**Base image:** `ros:noetic-ros-base`
**Icerik:** `control.py` (PID waypoint takip), gerekli Python bagimliliklari ve CAN araclari.

## Sistem Nasil Calistirilir?

### Otomatik (Onerilen)

```bash
cd ~/talos-sim/scripts/talos26_ws
./baslat.sh
```

`baslat.sh` tum sistemi (konum, harita, hedef, engel, trafik, serit, karar ve kontrol) sirasiyla baslatir.

### Manuel

```bash
# 1. Docker imajini yukle (ilk seferde)
docker load -i hilmi-talos/talos_control.tar

# 2. Container'i calistir
docker run --rm \
    --name talos-controller \
    --network host \
    --privileged \
    --cap-add NET_ADMIN \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_IP=127.0.0.1 \
    -v "$(pwd)/hilmi-talos/logs:/app/logs" \
    -v "$(pwd)/hilmi-talos/control.py:/app/control.py:ro" \
    talos-control:latest \
    control.py
```

### Gorevi Baslatma

Container basladiktan sonra CAN uzerinden baslat komutu gonderilir:

```bash
cansend vcan0 500#01
```

## Publish Edilen ROS Topic'leri

| Topic | Mesaj Tipi | Aciklama |
|-------|-----------|----------|
| `/gorev_durumu` | `std_msgs/String` | Waypoint'e varis bildirimi (hedef_yoneticisi'ne) |
| `/hedef_marker` | `visualization_msgs/Marker` | Aktif hedefin RViz gorsellestirmesi |

## Publish Edilen CAN Mesajlari

| CAN ID | Yon | Aciklama | Veri Formati |
|--------|-----|----------|-------------|
| `0x100` | Controller -> Arac | Gaz, Fren, Vites | byte0: gaz (0-100), byte1: fren (0-100), byte2: vites (1=bos, 2=ileri) |
| `0x201` | Controller -> Arac | Direksiyon acisi | int16 little-endian (derece * 10) |
| `0x301` | Controller -> Visualizer | Arac hizi | float32 (km/h) |
| `0x400` | Controller -> Arac | Sinyal lambasi | byte0: sol/sag/dortlu |

## Subscribe Olunan ROS Topic'leri

| Topic | Mesaj Tipi | Kaynak Node | Aciklama |
|-------|-----------|-------------|----------|
| `/base_pose_ground_truth` | `nav_msgs/Odometry` | Gazebo (konum-server) | Arac konumu ve yaw bilgisi |
| `/hedef` | `std_msgs/String` | hedef_teslimi | Dinamik hedef koordinatlari (`x,y` formati) |
| `/karar` | `std_msgs/String` | karar-node | Surucu karari (normal, slow, dur, acildurus, sag, sol) |
| `/line` | `std_msgs/Float32` | lane-node | Serit takip acisi (derece) |

## Subscribe Olunan CAN Mesajlari

| CAN ID | Yon | Aciklama |
|--------|-----|----------|
| `0x500` | Dis -> Controller | Gorev baslat komutu (0x01 = basla) |
| `0x302` | Arac -> Controller | IMU verileri (yaw, pitch, roll) |

## Sistem Ne Zaman Calismaz?

### Zorunlu Bagimliliklar

- **ROS Master** (`roscore`) calismiyor olmali. `rostopic list` ile kontrol edilebilir.
- **vcan0** arayuzu aktif olmali. `ip link show vcan0` ile kontrol edilebilir.
- **Docker Engine** kurulu ve calisir durumda olmali.
- **`--network host`** ve **`--privileged`** flag'leri olmadan container CAN erisimi saglanamaz.

### Eksik Topic Durumlari

| Eksik Topic | Sonuc |
|-------------|-------|
| `/base_pose_ground_truth` | Arac konumu alinamaz, hicbir hareket olmaz |
| `/hedef` | Dinamik hedef alinmaz, varsayilan waypoint'ler kullanilir |
| `/karar` | Karar bilgisi gelmez, varsayilan `normal` mod devam eder |
| `/line` | Serit duzeltmesi uygulanmaz, sadece waypoint PID ile gider |
| CAN `0x500` | Gorev baslamaz, container beklemede kalir |

### Diger Hatalar

- **konum-server** container'i calismiyor ise `/base_pose_ground_truth` yayinlanmaz.
- **hedef_teslimi** container'i calismiyor ise hedef gelmez.
- **karar-node** container'i calismiyor ise karar bilgisi gelmez.
- **CAN koprusu** (`can_to_talos_cart.py`, `talos_state_to_can.py`) calismiyor ise CAN komutlari Gazebo'ya ulasmaz.

## Gerekli Ortam ve Bagimliliklar

### Sistem

| Gereksinim | Versiyon |
|------------|----------|
| Ubuntu | 20.04 LTS |
| ROS | Noetic |
| Docker | 20.10+ |
| CAN araclari | `can-utils`, `iproute2` |
| GPU | Gerekli degil (control node icin) |

### Python (Container icinde kurulu)

| Paket | Versiyon |
|-------|----------|
| `python-can` | >= 4.0.0 |
| `numpy` | >= 1.20.0 |
| `matplotlib` | >= 3.4.0 |

### ROS Paketleri (Container icinde kurulu)

- `ros-noetic-tf`
- `ros-noetic-tf2-ros`

### Host Tarafinda Gerekenler

- `vcan0` virtual CAN arayuzu (kernel modulu: `vcan`)
- ROS workspace build edilmis (`catkin_make`)
- CAN kopru scriptleri: `can_to_talos_cart.py`, `talos_state_to_can.py`
