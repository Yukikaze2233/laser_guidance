FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i "s@http://archive.ubuntu.com@http://mirrors.aliyun.com@g" /etc/apt/sources.list.d/ubuntu.sources \
 && sed -i "s@http://security.ubuntu.com@http://mirrors.aliyun.com@g" /etc/apt/sources.list.d/ubuntu.sources \
 && apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gcc-14 \
    g++-14 \
    libopencv-dev \
    libyaml-cpp-dev \
    zsh \
    ros-jazzy-ament-cmake \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rclcpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-std-srvs \
    python3-colcon-common-extensions \
  && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 100 \
  && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 100 \
  && update-alternatives --install /usr/bin/cc  cc  /usr/bin/gcc-14 100 \
  && update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++-14 100 \
  && rm -rf /var/lib/apt/lists/*

COPY docker/ros2-jazzy-bridge-entrypoint.sh /usr/local/bin/ws30-bridge-entrypoint
RUN chmod +x /usr/local/bin/ws30-bridge-entrypoint

WORKDIR /workspace/laser_guidance-ws30

ENTRYPOINT ["/usr/local/bin/ws30-bridge-entrypoint"]
CMD []
