# TALOS Waypoint Editor

Gazebo simulasyonundaki pist haritasi uzerinde interaktif waypoint olusturma araci. Haritaya tiklayarak waypoint ekleyip, surekleyip, silip, kaydedin. Araci surup waypoint cikarmaya gerek kalmaz.

![Kalibrasyon Dogrulama](docs/calibration_check.png)

## Projeyi Klonlama

```bash
# Repoyu klonla
git clone git@github.com:uludagai-club/talos26_ws.git

# Klasore gir
cd talos26_ws
```

> **Not:** SSH key'in GitHub hesabina ekli olmasi gerekir. HTTPS ile klonlamak istersen:
> ```bash
> git clone https://github.com/uludagai-club/talos26_ws.git
> ```

Sonraki guncellemeleri almak icin:

```bash
git pull origin main
```

## Kurulum

```bash
cd waypoint-editor
pip install -r requirements.txt
```

> **Not:** `tkinter` sisteminizde yoksa: `sudo apt install python3-tk`

## Kullanim

```bash
# Temel kullanim (varsayilan pist goruntusu ile)
python3 waypoint_editor.py

# Mevcut waypoint dosyasini yukleyerek
python3 waypoint_editor.py --load output/waypoints.json

# Farkli pist goruntusu ile
python3 waypoint_editor.py --track data/track_layout.jpg

# Lidar haritasini da ekleyerek
python3 waypoint_editor.py --lidar-yaml /path/to/my_map.yaml
```

## Kontroller

### Fare

| Kontrol | Islem |
|---------|-------|
| Sol Tik | Waypoint ekle (Ekle modunda) |
| Sag Tik | En yakin waypoint'i sil |
| Orta Tik + Surukle | Waypoint'i tasi |
| Scroll | Zoom in/out |

### Klavye

| Tus | Islem |
|-----|-------|
| `1` | Ekleme modu |
| `2` | Secme/Tasima modu |
| `3` | Silme modu |
| `s` | Secili noktayi durak yap/kaldir |
| `Delete` / `Backspace` | Secili waypoint'i sil |
| `Ctrl+Z` | Geri al |
| `Ctrl+Y` | Ileri al |
| `Ctrl+S` | Kaydet |

### Gorunum

| Tus | Islem |
|-----|-------|
| `C` | Kalibrasyon modunu ac/kapat |
| `R` | Referans noktalarini goster/gizle |
| `T` | Pist goruntusunu ac/kapat |
| `L` | Lidar haritasini ac/kapat |
| `G` | Grid cizgilerini ac/kapat |

### Kalibrasyon Modu (`C` ile aktiflestir)

Pist goruntusunu Gazebo koordinatlarina hizalamak icin kullanilir.

| Tus | Islem |
|-----|-------|
| Ok tuslari | Goruntuyu kaydir |
| `+` / `-` | Goruntuyu buyut/kucult |
| `Shift+Ok` | Tek eksende ince ayar |
| `Ctrl+S` | Kalibrasyonu kaydet |

## Cikti Formatlari

Kaydet butonuna basinca (`Ctrl+S`) 3 formatta dosya uretir:

### 1. JSON (`output/waypoints.json`)
```json
{
  "metadata": {
    "created": "20260402_120000",
    "count": 8,
    "total_distance_m": 95.3,
    "coordinate_system": "gazebo_xy_meters"
  },
  "waypoints": [
    {"x": -4.7047, "y": -34.308881},
    {"x": 8.8342, "y": -34.313881, "stop": true}
  ]
}
```

### 2. Python (`output/waypoints_export.py`)
```python
DEFAULT_WAYPOINTS = [
    (-4.704700, -34.308881),
    (8.834200, -34.313881),  # DURAK
]
```
> Dogrudan `can_waypoint_follower.py` icine kopyalanabilir.

### 3. CSV (`output/waypoints.csv`)
```
index,x,y,name,stop,speed
1,-4.704700,-34.308881,,False,
2,8.834200,-34.313881,,True,
```

Ayrica terminale `--waypoints` komut satiri formatini yazdirir:
```
--waypoints "-4.7047,-34.3089 8.8342,-34.3139"
```

## Dosya Yapisi

```
waypoint-editor/
├── waypoint_editor.py      # Ana uygulama
├── requirements.txt        # Python bagimliliklari
├── README.md
├── data/
│   └── track_layout.jpg    # Pist layout goruntusu
├── output/                 # Cikti dosyalari (otomatik olusur)
│   ├── waypoints.json
│   ├── waypoints_export.py
│   ├── waypoints.csv
│   └── backups/            # Otomatik yedekler
└── track_calibration.json  # Goruntu hizalama verisi
```

## Kalibrasyon

Pist goruntusu, Gazebo simülasyon koordinatlarina (XY metre) hizalanmis olarak gelir. Kalibrasyon degerleri `track_calibration.json` dosyasinda saklanir.

Varsayilan kalibrasyon, senaryo2 modelinin DAE mesh dosyasindaki 85+ bariyer pozisyonundan otomatik hesaplanmistir.

Eger goruntu hizalamasi bozuksa:
1. `C` tusuyla kalibrasyon moduna girin
2. Referans noktalarini (`R`) acin - mavi kare aracin baslangic konumu
3. Ok tuslari ve `+/-` ile goruntuyu hizalayin
4. `Ctrl+S` ile kalibrasyonu kaydedin

## Entegrasyon

### can_waypoint_follower.py ile kullanim

```bash
# 1. Editorden waypoint'leri cikart ve kaydet
python3 waypoint_editor.py

# 2a. Komut satirindan kullan
python3 can_waypoint_follower.py --waypoints "-4.7047,-34.3089 8.8342,-34.3139 ..."

# 2b. Veya waypoints_export.py dosyasindaki DEFAULT_WAYPOINTS listesini
#     can_waypoint_follower.py'ye kopyalayin
```

### hedef_yoneticisi.py ile kullanim

`output/waypoints.json` dosyasi dogrudan hedef yoneticisi tarafindan okunabilir.
