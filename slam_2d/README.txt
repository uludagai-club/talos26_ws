SLAM 2D Project Files
---------------------
Bu klasör, 2D SLAM ve Simülasyon projesi için gerekli kaynak kodlarını içerir.

İçerik:
1. slam_2d/     : SLAM, haritalama ve waypoint araçlarını içeren ana paket.

Sistem Gereksinimleri:
- Ubuntu 20.04
- ROS Noetic

Gerekli ROS Paketleri (Dependencies):
Projenin çalışması için aşağıdaki paketlerin yüklü olması gerekir:
- gmapping (Haritalama için)
- pointcloud_to_laserscan (3D Lidar verisini 2D'ye çevirmek için)
- gazebo_ros_pkgs (Simülasyon ortamı için)
- rviz (Görselleştirme için)
- python3-rospkg

Bağımlılıkları Yükleme Komutu:
Terminalde şu komutu çalıştırarak gerekli tüm paketleri yükleyebilirsiniz:

    sudo apt update
    sudo apt install ros-noetic-gmapping ros-noetic-pointcloud-to-laserscan ros-noetic-gazebo-ros-pkgs ros-noetic-rviz python3-rospkg

Kurulum (Karşı Bilgisayarda):
1. Bu klasörleri bir catkin workspace'inin 'src' klasörüne kopyalayın (örn: ~/catkin_ws/src).
2. Workspace ana dizininde 'catkin_make' çalıştırın.
3. 'source devel/setup.bash' komutunu girin.

Çalıştırma:
roslaunch slam_2d start_slam.launch

