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

如果宿主机不是 Ubuntu Noble，推荐直接使用 Docker：

```bash
bash scripts/ws30_bridge_docker.sh
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

点云查看支持两条路径：

1. Foxglove — 直接连接 ROS2，订阅 `PointCloud2`
2. RViz2 — 使用预置配置：
   ```bash
   ros2 run rviz2 rviz2 -d rviz/ws30_lidar.rviz
   ```

两者都消费标准 `PointCloud2`。

## Deferred Items

当前先不做：

- `/tf`
- 自定义复杂消息
- 把 Foxglove / RViz2 UI 逻辑并入 core
