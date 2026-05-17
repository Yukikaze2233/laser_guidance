FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ros-jazzy-ament-cmake \
    ros-jazzy-rclcpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-std-srvs \
    python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/laser_guidance-ws30

CMD ["bash"]
