FROM ros:noetic-ros-base

# Çalışma dizini
WORKDIR /app

# Sistem bağımlılıkları
RUN apt-get update && apt-get install -y \
    python3-pip \
    can-utils \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Python bağımlılıkları
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Uygulama dosyalarını kopyala
COPY *.py ./
COPY *.sh ./
RUN chmod +x *.sh *.py

# ROS ortamını yükle
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Varsayılan komut
CMD ["python3", "can_waypoint_follower.py"]
