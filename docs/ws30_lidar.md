# WS30 LiDAR

## Repository

WS30 核心库已抽出为独立子模块：
- `external/ws30_lidar_core` — `github.com/Yukikaze2233/ws30_lidar_core`
- 命名空间 `ws30_lidar`，零 ROS/OpenCV 依赖

## Current Status

当前仓库已经落地的 WS30 能力：

- `Ws30UdpSocket` / `Ws30PacketParser` / `Ws30FrameAssembler` / `Ws30Client` ✅
- `tool_lidar_dump`（CLI 调试入口）✅
- `ws30_lidar_node`（ROS2 bridge，已验证可发布 PointCloud2/Imu/DiagnosticArray）✅
- 完整 Docker 容器工作流（Ubuntu Noble + ROS2 Jazzy，含 Foxglove WebSocket bridge）✅
- Foxglove WebSocket 可视化已验证 ✅

已验证项：

- UDP 收包稳定 ✅
- points / imu / status 解析正确 ✅
- 点云帧能稳定组装 ✅
- Foxglove 3D 面板可实时看点云 ✅
- 容器内 `foxglove_bridge` WebSocket `ws://localhost:8765` 可用 ✅

当前**尚未实现**：

- 与 `GuidancePipeline` 的深度融合

## Current Debug Entry

```bash
./build/tool_lidar_dump --help
./build/tool_lidar_dump --device-ip 192.168.137.200 --iterations 10
./build/tool_lidar_dump --device-ip 192.168.137.200 --iterations 100 --record-raw /tmp/ws30.rawlog
./build/tool_lidar_dump --replay /tmp/ws30.rawlog --write-pcd /tmp/ws30_pcd
```

当前 CLI 会：

- 请求 points / imu / serial number
- 打印点云完整帧摘要
- 打印 imu 摘要
- 打印 status / serial number 摘要
- 在无设备时清晰打印 timeout
- 录制 WS30 原始 UDP payload (`--record-raw`)
- 从 raw log 回放 (`--replay`)
- 将完整点云帧导出成 ASCII PCD (`--write-pcd`)

## ROS2 Bridge

启动 bridge 后在 Foxglove 或 RViz2 中实时看点云：

```bash
cmake -S external/ws30_lidar_core -B external/ws30_lidar_core/build -DCMAKE_BUILD_TYPE=Release
cmake --build external/ws30_lidar_core/build --parallel

colcon build --packages-select ws30_lidar_bridge
source install/setup.bash
ros2 launch ws30_lidar_bridge ws30_lidar.launch.py

ros2 run rviz2 rviz2 -d rviz/ws30_lidar.rviz
```

在 Arch 等非 Ubuntu Noble 宿主机上，推荐直接使用完整容器工作流：

```bash
docker compose -f docker-compose.ws30-bridge.yml up --build
```

例如指定设备 IP：

```bash
WS30_DEVICE_IP=192.168.137.200 docker compose -f docker-compose.ws30-bridge.yml up --build
```

Foxglove WebSocket 默认地址：

```text
ws://localhost:8765
```

只编译 / 编译后进入 shell：

```bash
docker compose -f docker-compose.ws30-bridge.yml run --rm ws30-bridge build
docker compose -f docker-compose.ws30-bridge.yml run --rm ws30-bridge shell
```

## Why Standalone First

当前优先级是确认 WS30 原始数据正确，而不是先接 ROS2 UI。

所以调试顺序应固定为：

1. `tool_lidar_dump`
2. raw replay / PCD export
3. ROS2 bridge + RViz2 / Foxglove 可视化
4. guidance 深度接入

## Current Raw Log Format

raw log 当前是自定义二进制格式：

- 文件头：`WS30LOG1`
- entry header：`stream_kind + capture_unix_ns + payload_size`
- payload：WS30 原始 UDP datagram

这样可以把设备问题、parser 问题、bridge 问题分开排查。

## Current Next Step

现在最合理的下一步是：

1. 把 WS30 深度接入 `GuidancePipeline`
2. 相机-雷达外参标定
3. 视觉-雷达深度融合
