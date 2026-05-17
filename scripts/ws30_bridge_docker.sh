#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-laser-guidance-ros2-jazzy-bridge}"

docker build \
  -t "$IMAGE_NAME" \
  -f "$REPO_ROOT/docker/ros2-jazzy-bridge.Dockerfile" \
  "$REPO_ROOT"

docker run --rm -it \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v "$REPO_ROOT:/workspace/laser_guidance-ws30" \
  -w /workspace/laser_guidance-ws30 \
  "$IMAGE_NAME" \
  bash -lc '
    set -euo pipefail
    cmake -S external/ws30_lidar_core -B external/ws30_lidar_core/build -DCMAKE_BUILD_TYPE=Release
    cmake --build external/ws30_lidar_core/build --parallel
    source /opt/ros/jazzy/setup.bash
    cd ros2/ws30_lidar_bridge
    rm -rf build install log
    colcon build --packages-select ws30_lidar_bridge
    source install/setup.bash
    exec ros2 launch ws30_lidar_bridge ws30_lidar.launch.py "$@"
  ' bridge "$@"
