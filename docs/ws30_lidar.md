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
- `LidarDepthEstimator`（点云靶子/机身分离 + 靶子簇测距）✅
- `GuidancePipeline` 接入点云深度（`depth_source: lidar_target_cluster`）✅
- 分离调试 overlay（ROI 点数、聚类数、目标分数、深度、3D 尺寸）✅

已验证项：

- UDP 收包稳定 ✅
- points / imu / status 解析正确 ✅
- 点云帧能稳定组装 ✅
- Foxglove 3D 面板可实时看点云 ✅
- 容器内 `foxglove_bridge` WebSocket `ws://localhost:8765` 可用 ✅
- 靶子/机身分离硬筛选 + 多指标打分 ✅
- 靶子簇深度输出（优先 median depth）✅
- 分离结果可在 `tool_guidance` / `tool_competition` overlay 直接查看 ✅

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

## 靶子/机身点云分离

点云分离逻辑位于 `src/guidance/lidar_depth_estimator.cpp`，流程为：

1. **ROI 裁剪** — 用视觉检测框 `bbox` 裁出目标附近点云
2. **3D 聚类** — 对 ROI 内点做欧式聚类
3. **硬筛选** — 过滤中心偏差过大、厚度过大、尺寸过大、深度波动过大的簇
4. **多指标打分** — 对通过筛选的簇按居中/薄/尺寸匹配/深度稳定/密度/强度综合评分
5. **置信判决** — 第一名分数 < 0.60 或与第二名差距 < 0.10 则拒绝
6. **深度输出** — 优先中位深度

运行时 overlay 显示（仅当 `depth_source: lidar_target_cluster`）：

```text
LIDAR ROI=... clusters=... TARGET/NONE
depth=... score=... center=...px std=...
size=(x,y,z)mm
```

配置参数（`guidance` 段）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `depth_source` | `monocular_bbox` | `lidar_target_cluster` 启用点云分离测距 |
| `lidar_bbox_margin_px` | 24 | bbox ROI 扩展像素 |
| `lidar_cluster_tolerance_mm` | 120 | 聚类邻域容差 |
| `lidar_min_cluster_points` | 8 | 最小簇点数 |
| `lidar_max_depth_mm` | 40000 | 最大有效深度 |

靶板物理尺寸已统一为 `72.5 × 50.0 mm`，用于单目测距和簇尺寸评分。

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

1. 多帧稳定锁定同一目标簇，减少靶子/机身来回跳
2. 相机-雷达外参标定
3. 视觉-雷达深度融合
