#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/workspace/laser_guidance-ws30}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
BUILD_PARALLELISM="${BUILD_PARALLELISM:-$(nproc)}"
FOXGLOVE_ENABLED="${FOXGLOVE_ENABLED:-true}"
FOXGLOVE_WS_PORT="${FOXGLOVE_WS_PORT:-8765}"

if [[ ! -d "$REPO_ROOT/external/ws30_lidar_core" ]]; then
  echo "ws30-bridge-entrypoint: repo mount missing at $REPO_ROOT" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$REPO_ROOT"

mode="run"
if [[ $# -gt 0 ]]; then
  mode="$1"
  shift
fi

build_ws30_bridge() {
  cmake -S external/ws30_lidar_core -B external/ws30_lidar_core/build -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  cmake --build external/ws30_lidar_core/build --target ws30_lidar_core --parallel "$BUILD_PARALLELISM"

  cd ros2/ws30_lidar_bridge
  rm -rf build install log
  colcon build \
    --packages-select ws30_lidar_bridge \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  set +u
  source install/setup.bash
  set -u
}

start_foxglove_bridge() {
  if [[ "$FOXGLOVE_ENABLED" != "true" ]]; then
    return
  fi

  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="$FOXGLOVE_WS_PORT" &
  FOXGLOVE_PID=$!
}

cleanup() {
  if [[ -n "${FOXGLOVE_PID:-}" ]]; then
    kill "$FOXGLOVE_PID" 2>/dev/null || true
    wait "$FOXGLOVE_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

case "$mode" in
  build)
    build_ws30_bridge
    ;;
  shell)
    build_ws30_bridge
    exec bash
    ;;
  run)
    build_ws30_bridge
    start_foxglove_bridge
    exec ros2 launch ws30_lidar_bridge ws30_lidar.launch.py "$@"
    ;;
  *)
    exec "$mode" "$@"
    ;;
esac
