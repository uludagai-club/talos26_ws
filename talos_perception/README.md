# talos_perception

Talos otonom aracı için **3B LiDAR engel-tespiti zinciri**. Ham LiDAR taramasından
yönlü engel kutularına (OBB) kadar tüm boru hattını oluşturan üç catkin paketi:

```
/cart/center_laser/scan   (LiDAR ham tarama — simülasyon/araç tarafından yayınlanır)
        │
        ▼  talos_lidar_filter
        │     pass_filter   → /cart/points_filtered   (ROI / mesafe kırpma)
        │     voxel_filter  → /cart/points_voxel      (downsample)
        │
        ▼  talos_ground_removal  (Patchwork++)
        │     → /cart/points_noground   (engel adayları)
        │     → /cart/points_ground     (zemin, görselleştirme)
        │
        ▼  talos_obstacle_detector
        │     DBSCAN kümeleme + PCA OBB + zamansal takip
        │     → /obstacles          (jsk_recognition_msgs/BoundingBoxArray)
        │     → /obstacles/poses, /obstacles/markers, /obstacles/clusters
```

## Paketler

| Paket | Görev |
|---|---|
| `talos_lidar_filter` | Ön-işleme: pass-through (ROI) + voxel downsample (+ ops. outlier) |
| `talos_ground_removal` | Patchwork++ ile zemin ayıklama (header-only, dış bağımlılık yok) |
| `talos_obstacle_detector` | DBSCAN + PCA OBB + takip ile engel tespiti (detay: alt README) |

## Bağımlılıklar

```bash
sudo apt install ros-noetic-jsk-recognition-msgs ros-noetic-jsk-rviz-plugins \
                 ros-noetic-pcl-ros
```
(+ ROS Noetic, PCL, Eigen — standart)

## Derleme

Üç paketi de catkin workspace'inizin `src/` dizinine kopyalayın:

```bash
cp -r talos_perception/* ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make -j6
source devel/setup.bash
```

## Çalıştırma

Simülasyon/araç ayaktayken (`/cart/center_laser/scan` yayınlanıyorken) **tüm zincir
tek komutla**:

```bash
roslaunch talos_obstacle_detector perception_all.launch
```

Ya da aşamaları ayrı ayrı:

```bash
roslaunch talos_lidar_filter lidar_filter.launch
roslaunch talos_ground_removal ground_removal.launch
roslaunch talos_obstacle_detector obstacle_detector.launch
```

RViz'de kutuları görmek için **BoundingBoxArray** display ekleyip topic'i
`/obstacles`, frame'i `velodyne` yapın.

## Notlar

- Tüm hesap `velodyne` (sensör) frame'inde yapılır — TF interpolasyon hatası yok.
- Girdi topic adları (`/cart/...`) her paketin `config/*.yaml` dosyasından
  değiştirilebilir; farklı bir LiDAR topic'i için yalnız `talos_lidar_filter`
  girişini güncellemek yeterli.
- Engel tespiti parametreleri (DBSCAN eps/MinPts, OBB, takip) için bkz.
  `talos_obstacle_detector/README.md`.
