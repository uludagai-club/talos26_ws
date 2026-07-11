# talos_obstacle_detector

Canlı 3B LiDAR taramasını, önceden çıkarılmış **statik PCD haritasıyla
karşılaştırarak** engel tespiti yapan ROS Noetic (catkin) paketi. Haritada
bulunmayan (yeni/dinamik) noktalar **Öklid kümeleme** ile gruplanır ve her küme
için **PCA tabanlı yönlü kutu (OBB)** üretilerek `jsk_recognition_msgs/BoundingBoxArray`
olarak yayınlanır.

Bu branch (`kerem-obstacle-gebzesel`), zemin-ayıklama (Patchwork++) gerektirmeyen,
**harita-farkı (map-diff)** tabanlı bir yaklaşım kullanır. DBSCAN + zamansal takip
tabanlı önceki sürüm `kerem-obstacle-dbscan-obb` branch'inde durur.

## İki düğüm

| Düğüm | Görev |
|---|---|
| `map_publisher_node` | `~/talos_maps/clean.pcd` haritasını okur, `/map_cloud` konusuna **latched** olarak bir kez yayınlar |
| `obstacle_detector_node` | Canlı taramayı haritayla karşılaştırıp engelleri tespit eder ve kutular |

## Boru hattı

```
~/talos_maps/clean.pcd                          /cart/center_laser/scan
        │ map_publisher_node                            │ (canlı LiDAR, PointCloud2)
        ▼ (latched, bir kez)                            │
   /map_cloud ──────────► voxel(0.10) ► KdTreeFLANN     │
                                        (statik dünya)  │
                                              │         ▼  yarıçap filtresi (r < min_range at)
                                              │         ▼  TF: sensör frame → map
                                              │         ▼  voxel(0.15) seyrekleştir
                                              │         │
                                              └──► KD-tree fark: her canlı noktanın
                                                   haritaya en yakın komşusu
                                                   > novel_threshold ise "yeni" say
                                                          │
                                                          ▼  Öklid kümeleme
                                                          │  (cluster_tolerance, min/max size)
                                                          ▼  her küme → PCA OBB (yaw-only, Z dünya-hizalı)
                                                          ▼
/obstacles            jsk_recognition_msgs/BoundingBoxArray  (yönlü kutular)
/obstacles/poses      geometry_msgs/PoseArray                (kutu merkezleri)
/obstacles/cloud      sensor_msgs/PointCloud2                (fark bulutu = yeni noktalar)
/obstacles/x_extremes geometry_msgs/PoseArray                (her kutunun uzun-eksen uç noktaları)
/obstacles/markers    visualization_msgs/MarkerArray         (RViz id etiketleri + uç çizgileri)
```

## Algoritma

**Harita-farkı (novelty)**: Statik harita bulutu bir `KdTreeFLANN`'a konur. Her
canlı nokta için haritadaki en yakın komşu bulunur; mesafe `novel_threshold`'u
**aşıyorsa** o nokta haritada yoktur → *yeni/engel* noktası sayılır. Böylece
duvar/zemin/statik yapı elenir, yalnız sonradan gelen nesneler kalır. Bu, ayrı bir
zemin-ayıklama adımına gerek bırakmaz.

**Öklid kümeleme**: Yeni noktalara `pcl::EuclideanClusterExtraction` uygulanır;
`cluster_tolerance` içinde kalan komşu noktalar aynı nesne sayılır. `min/max_cluster_size`
dışındaki kümeler atılır.

**Hibrit PCA OBB**: Her küme için XY düzleminde kovaryans → özvektörlerden baskın
yön (yaw) bulunur; Z dünyaya dik tutulur (yalnız dönme). Noktalar bu eksene
döndürülüp yerel min/max ile kutu merkezi ve boyutları hesaplanır. `max_extent_xy`'yi
aşan kümeler (duvar/bariyer) elenir. Ayrıca kutunun uzun ekseni boyunca iki uç nokta
(`/obstacles/x_extremes`) çıkarılır — kaçınma/planlama için nesne sınır noktaları.

## Bağımlılıklar

`roscpp`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `pcl_ros`,
`pcl_conversions`, `tf2_ros`, `tf2_sensor_msgs`, `jsk_recognition_msgs` (+ PCL, Eigen).

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

Statik haritanız `~/talos_maps/clean.pcd` konumundayken (veya `pcd_path` argümanıyla):

```bash
roslaunch talos_obstacle_detector obstacle_detector.launch
# özel harita yolu:
roslaunch talos_obstacle_detector obstacle_detector.launch pcd_path:=/yol/harita.pcd
```

RViz'de kutuları görmek için **BoundingBoxArray** display'i ekleyip topic'i
`/obstacles` yapın (jsk_rviz_plugins gerekir), fixed frame `map`.

> Not: `map` çerçevesi ile sensör çerçevesi arasındaki TF ağacının canlı olması
> gerekir (aksi halde düğüm `TF lookup failed` uyarısı verir ve kare atlar).

## Parametreler (`config/params.yaml`)

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `map_topic` | `/map_cloud` | Statik harita bulutu konusu |
| `input_topic` | `/cart/center_laser/scan` | Canlı LiDAR girdi konusu |
| `output_frame` | `map` | Tespitlerin yayınlandığı çerçeve |
| `novel_threshold` | `0.30` | Haritaya bu mesafeden (m) uzak noktalar "yeni" sayılır |
| `cluster_tolerance` | `0.40` | Öklid kümeleme komşuluk mesafesi (m) |
| `min_cluster_size` | `8` | Bu sayıdan az noktalı kümeler atılır |
| `max_cluster_size` | `25000` | Üst küme boyutu |
| `max_extent_xy` | `5.0` | XY'de bu boyutu (m) aşan kutular elenir (duvar/bariyer) |
| `map_voxel_leaf` | `0.10` | Harita bulutu voxel yaprak boyutu (m) |
| `input_voxel_leaf` | `0.15` | Canlı bulut voxel yaprak boyutu (m) |
| `tf_timeout` | `0.10` | TF lookup zaman aşımı (s) |
| `min_range` | `2.5` | Sensöre bu mesafeden (m) yakın noktalar atılır (araç gövdesi) |

## Çıkış mesajı

`/obstacles` → `jsk_recognition_msgs/BoundingBoxArray`. Her kutuda: `pose`
(konum + yaw quaternion), `dimensions` (x/y/z), `label` (kare-içi engel id),
`value` (kümedeki nokta sayısı). Konum/boyut/yönelim verileri karar (`karar/`) ve
kontrol (`control/`) katmanlarına aktarılabilir.
