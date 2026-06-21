# talos_obstacle_detector

3B LiDAR nokta bulutundan **DBSCAN kümeleme + PCA tabanlı OBB (Oriented Bounding
Box)** çıkarımı ve **zamansal takip** ile engel tespiti yapan ROS Noetic (catkin)
paketi.

Girdi olarak zemin ayıklanmış nokta bulutunu (`/cart/points_noground`,
Patchwork++ çıktısı) alır; her engel için konum/boyut/yönelim içeren yönlü kutu
üretir ve `jsk_recognition_msgs/BoundingBoxArray` olarak yayınlar.

## Boru hattı

```
/cart/points_noground               (Patchwork++ zemin sonrası = engel adayları)
        │
        ▼  DBSCAN (eps, MinPts)      gürültü eleme + komşuluk tabanlı kümeleme
        │
        ▼  PCA OBB                   her küme için centroid + kovaryans +
        │                            özdeğer/özvektör → yönlü kutu (konum/boyut/yaw)
        │
        ▼  Tracker                   kareler arası eşleme + EMA yumuşatma +
        │                            yön-sıçraması (±90°) düzeltme + histerezis
        ▼
/obstacles            jsk_recognition_msgs/BoundingBoxArray  (label = stabil iz id)
/obstacles/poses      geometry_msgs/PoseArray
/obstacles/markers    visualization_msgs/MarkerArray (RViz id etiketleri)
/obstacles/clusters   sensor_msgs/PointCloud2 (renkli kümeler, debug)
```

Tüm hesap girdi bulutunun frame'inde (`velodyne`) yapılır — TF interpolasyon hatası
olmadan en yüksek geometrik doğruluk.

## Algoritma

**DBSCAN** (`Density-Based Spatial Clustering`): her nokta etrafında `eps` yarıçaplı
arama yapılır; komşu sayısı `min_pts`'i aşan noktalar *çekirdek* sayılır ve
kümeler çekirdeklerden genişletilir. Çekirdek olamayan, hiçbir kümeye komşu
olmayan noktalar **gürültü** olarak elenir.

**PCA OBB**: her küme için ağırlık merkezi `p̄` ve kovaryans matrisi `C`
hesaplanır; `C`'nin özvektörleri kutu eksenlerini, noktaların bu eksenlere
izdüşümünün min/max'ı kutu merkezi ile boyutlarını verir. `vertical_box=true`
iken Z dik tutulur ve yalnız baskın yatay yön (yaw) PCA'dan alınır (zemindeki
engeller için en kararlı sonuç).

**Tracker**: kare kare tespit doğal olarak "git gel" yapar (seyrek LiDAR'da küme
yanıp söner; PCA baskın ekseni ~90° atlar). Tracker olçümleri mevcut izlere
merkez mesafesine göre eşler, pozisyon/boyut/yaw'ı EMA ile yumuşatır, yaw'ı
önceki kareye en yakın 90°'lik eşitine çekerek eksen sıçramasını söndürür ve
histerezis ile kısa kopmaları köprüler. Bu karede tespit edilen her engel daima
gösterilir (hiçbir gerçek engel gizlenmez).

## Bağımlılıklar

`roscpp`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `pcl_ros`,
`pcl_conversions`, `jsk_recognition_msgs` (+ PCL, Eigen).

```bash
sudo apt install ros-noetic-jsk-recognition-msgs ros-noetic-jsk-rviz-plugins
```

## Derleme

Paketi catkin workspace'inizin `src/` dizinine koyun:

```bash
cd ~/catkin_ws && catkin_make -j6
source devel/setup.bash
```

## Çalıştırma

Sim ve zemin ayıklama (Patchwork++) ayaktayken:

```bash
roslaunch talos_obstacle_detector obstacle_detector.launch
```

RViz'de kutuları görmek için bir **BoundingBoxArray** display ekleyip topic'i
`/obstacles` yapın (jsk_rviz_plugins gerekir), frame `velodyne`.

## Parametreler (`config/params.yaml`)

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `input_topic` | `/cart/points_noground` | Zemin ayıklanmış girdi bulutu |
| `eps` | `0.5` | DBSCAN arama yarıçapı (m) |
| `min_pts` | `5` | DBSCAN çekirdek nokta eşiği (MinPts) |
| `min_cluster_size` | `10` | Bu sayıdan az noktalı kümeler atılır |
| `max_cluster_size` | `25000` | Üst küme boyutu |
| `max_extent_xy` | `15.0` | XY'de bu boyutu aşan kutular elenir (duvar/bariyer) |
| `max_height` | `6.0` | Z'de bu yüksekliği aşan kutular elenir |
| `vertical_box` | `true` | `true`: Z dik + yaw PCA'dan; `false`: tam 3B PCA |
| `track_enable` | `true` | `false`: ham kare-kare tespit (yumuşatma yok) |
| `assoc_dist` | `1.5` | Ardışık karelerde aynı engel için maks merkez mesafesi (m) |
| `pos_alpha` | `0.5` | Pozisyon EMA katsayısı (0=durağan, 1=tepkili) |
| `dim_alpha` | `0.3` | Boyut EMA katsayısı |
| `yaw_alpha` | `0.3` | Yönelim EMA katsayısı (eksen sıçraması sönümleme) |
| `min_hits` | `2` | Bir iz "onaylı" sayılana kadar gereken toplam kare |
| `max_misses` | `5` | Kaybolan onaylı iz kaç kare daha canlı tutulur |

## Çıkış mesajı

`/obstacles` → `jsk_recognition_msgs/BoundingBoxArray`. Her kutuda:
`pose` (konum + yönelim quaternion), `dimensions` (x/y/z boyut), `label` (stabil iz
id), `value` (kümedeki nokta sayısı). Konum/boyut/yönelim verileri ROS tabanlı
otonom sürüş karar mekanizmalarına aktarılabilir.
