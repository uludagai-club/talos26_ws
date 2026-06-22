ROS Noetic Tabanlı Trafik Levhası Tespiti 

Bu proje, ROS Noetic kullanılarak geliştirilen ve kamera görüntüsü üzerinden trafik levhası tespiti yapan bir görüntü işleme paketidir.
Sistem WSL (Windows Subsystem for Linux) üzerinde çalışmaktadır.

Kullanılan araçlar:

-Ubuntu 20.04 (WSL 2)

-ROS Noetic

-Python 3.8.10

-OpenCV

-YOLOV8

-gazebo11

KURULUM

1. Workspace içine ekle

cd ~/catkin_ws/src
git clone https://github.com/uludagai-club/talos26_ws/yolov8_ros.git
cd ..
catkin_make
source devel/setup.bash


2. Gerekli Python kütüphaneleri

pip install ultralytics opencv-python


3.Çalıştırma

rosrun yolov8_ros yolov8_ros_node.py

Gazebo Kamera Testi:
python3 yolo_gazebo_check.py
