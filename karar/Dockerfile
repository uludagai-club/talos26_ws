FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-rospy \
        ros-noetic-std-msgs \
        ros-noetic-sensor-msgs \
        ros-noetic-geometry-msgs \
        ros-noetic-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
        "py_trees==2.2.3" \
        "PyYAML>=5.4"

WORKDIR /app

CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && python3 -u /app/karar_bt_node.py"]
