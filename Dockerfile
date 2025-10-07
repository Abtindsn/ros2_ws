# syntax=docker/dockerfile:1.4

FROM docker.io/library/ros:humble-ros-base

# Prevent interactive tzdata setup
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        ros-humble-can-msgs \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rosbag2-storage-default-plugins \
        \
        wget \
        curl && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
        "numpy<2" \
        opencv-python-headless==4.8.1.78 \
        python-can \
        pyserial \
        pynmea2 \
        fastapi \
        "uvicorn[standard]"

WORKDIR /ws

COPY . /ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    (rosdep init || true) && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

CMD ["/bin/bash", "-lc", ". install/setup.bash && ros2 launch vehicle_telemetry telemetry_stack.launch.py"]
