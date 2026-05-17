# ROS2 Interface

## Design Rule

ROS2 只做 bridge，不进入 core。

bridge 的职责固定为：

- 参数读取
- 输入采集
- 结果发布

不要在 bridge 中重新实现 WS30 协议解析。

## Package Shape

当前 bridge package 位置：

- `ros2/ws30_lidar_bridge/`

它只依赖 standalone `ws30_lidar_core`（通过路径查找已编译出的 `libws30_lidar_core.a`），但不反向污染 core。

先编译 standalone `ws30_lidar_core`，再 colcon build bridge：

```bash
cmake -S external/ws30_lidar_core -B external/ws30_lidar_core/build -DCMAKE_BUILD_TYPE=Release
cmake --build external/ws30_lidar_core/build --parallel
cd ros2/ws30_lidar_bridge
colcon build --packages-select ws30_lidar_bridge
```

如果宿主机不是 Ubuntu Noble，推荐直接使用完整 Docker 工作流：

```bash
docker compose -f docker-compose.ws30-bridge.yml up --build
```

常用参数通过环境变量传入：

```bash
WS30_DEVICE_IP=192.168.137.200 docker compose -f docker-compose.ws30-bridge.yml up --build
ROS_DOMAIN_ID=7 docker compose -f docker-compose.ws30-bridge.yml up --build
FOXGLOVE_WS_PORT=8765 docker compose -f docker-compose.ws30-bridge.yml up --build
```

容器模式：

```bash
docker compose -f docker-compose.ws30-bridge.yml up --build
docker compose -f docker-compose.ws30-bridge.yml run --rm ws30-bridge build
docker compose -f docker-compose.ws30-bridge.yml run --rm ws30-bridge shell
```

Foxglove 如果使用 WebSocket 连接，默认地址为：

```text
ws://localhost:8765
```

## Topics

当前实际 topic（namespace 下）：

- `~/points`
  - `sensor_msgs/msg/PointCloud2`
- `~/imu`
  - `sensor_msgs/msg/Imu`
- `~/status`
  - `diagnostic_msgs/msg/DiagnosticArray`

节点 namespace 为 `/gimbal/laser_guidance/ws30`，完整路径：

- `/gimbal/laser_guidance/ws30/points`
- `/gimbal/laser_guidance/ws30/imu`
- `/gimbal/laser_guidance/ws30/status`

## Parameters

- `device_ip`
- `points_port`
- `imu_port`
- `status_port`
- `frame_id`，默认 `ws30_lidar`
- `packet_timeout_ms`
- `publish_imu`
- `use_sensor_timestamp`

## Services

- `~/reopen`
  - `std_srvs/srv/Trigger`

## Running

```bash
source install/setup.bash
ros2 launch ws30_lidar_bridge ws30_lidar.launch.py
```

自定义参数：

```bash
ros2 launch ws30_lidar_bridge ws30_lidar.launch.py device_ip:=192.168.1.100 publish_imu:=false
```

## Visualization

点云查看推荐使用 Foxglove WebSocket 连接（容器内已内置 `foxglove_bridge`）：

1. 打开 Foxglove → `Open connection` → 选 **Foxglove WebSocket**
2. 地址填 `ws://localhost:8765`（远程则换宿主机 IP）
3. 推荐面板组合：
   - `3D` → 订阅 `/gimbal/laser_guidance/ws30/points`，Frame: `ws30_lidar`，Color: `Intensity`
   - `Raw Messages` → 订阅 `/gimbal/laser_guidance/ws30/status`
   - `Plot` → 订阅 `/gimbal/laser_guidance/ws30/imu`

也支持 RViz2：

```bash
ros2 run rviz2 rviz2 -d rviz/ws30_lidar.rviz
```

两者都消费标准 `PointCloud2`。

## Deferred Items

当前先不做：

- `/tf`
- 自定义复杂消息
- 把 Foxglove / RViz2 UI 逻辑并入 core
