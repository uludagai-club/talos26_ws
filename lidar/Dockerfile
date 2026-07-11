# =============================================================================
#  talos_obstacle_detector — map-diff tabanli engel tespiti (C++/PCL) icin imaj
# -----------------------------------------------------------------------------
#  talos-all:latest runtime-only imaji derleyici + PCL + jsk_recognition_msgs
#  TASIMADIGI icin bu C++ node kendi imajinda derlenir. Bu, control/ ve karar/
#  ile ayni "per-modul Dockerfile" desenidir. Diger (Python) node'lar gibi
#  kod bind-mount edilmez — C++ derleme gerektirdiginden paket imaja kopyalanip
#  catkin ile build edilir. config/ ve launch/ compose'da uzerine mount'lanarak
#  parametreler rebuild'siz degistirilebilir.
#
#  Runtime data bagimliligi: statik PCD haritasi /maps/clean.pcd konumuna
#  compose volume ile mount edilir (bkz. docker-compose.yml obstacle-detector).
# =============================================================================
FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libpcl-dev \
    libeigen3-dev \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-jsk-recognition-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# Catkin workspace + paketi kopyala ve derle
WORKDIR /ws
COPY lidar/talos_obstacle_detector /ws/src/talos_obstacle_detector

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd /ws/src && catkin_init_workspace \
    && cd /ws && catkin_make -j2 -DCMAKE_BUILD_TYPE=Release"

# Harita compose volume ile buraya mount edilir (repoya 37 MB PCD commit'lenmez)
ENV TALOS_MAP_PCD=/maps/clean.pcd

CMD ["bash"]
