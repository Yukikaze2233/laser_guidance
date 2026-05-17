# laser_guidance

`laser_guidance` 是一个激光视觉引导系统，当前阶段覆盖：

- `V4L2/UVC` 采集卡取图
- 原始视频会话录制与可选离线抽帧导出
- YAML 配置加载
- `Pipeline` 统一视觉入口与可切换后端骨架
- 调试 overlay 与回放样本
- 自动测试与工具入口
- ONNX / TensorRT GPU 推理，双后端运行时热切换
- FT4222H USB-to-SPI 振镜控制
- Direct voltage 视觉→电压多项式映射（Poly3，支持增量标定训练）
- 比赛模式统一守护进程 (`tool_competition`)
- 空中机器人被瞄准进度计算 (`HitProgress`，RoboMaster 2026 §5.6.3)
- EKF 跟踪（可运行时关闭，切换原始检测直驱）
- 敌方颜色过滤（red/blue/auto）
- WS30 激光雷达 standalone scaffold（UDP 收包、协议解析、点云组帧、CLI 调试入口）

当前阶段**不包含**：

- 控制链路闭环（比赛模式已闭环）
- `/gimbal/*` 接口
- `tracker` / `solver` / `planner`
- `fire_control`

## Build

```bash
# 基础构建
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

启用可选特性：

```bash
# ONNX Runtime + TensorRT + FT4222 (推荐比赛配置)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DWITH_ONNXRUNTIME=ON \
  -DWITH_TENSORRT=ON \
  -DWITH_FT4222=ON

# 仅 ONNX Runtime
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DWITH_ONNXRUNTIME=ON

# 禁用 FT4222H
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DWITH_FT4222=OFF
```

## Tests

```bash
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Tools

构建后在 `build/` 下直接运行：

```bash
./build/tool_competition             # 比赛模式（守护进程，TensorRT+EKF+振镜+推流+录制）
./build/tool_preview                 # 本地预览（RTP推流+SHM+FIFO）
./build/tool_guidance                # 激光引导（模型+EKF+振镜，支持标定模式）
./build/tool_capture                 # 录帧 (PNG + manifest)
./build/tool_record                  # 原始视频会话录制
./build/tool_replay                  # 回放预览
./build/tool_model_infer             # 模型推理
./build/tool_detector_benchmark      # 检测 benchmark
./build/tool_calibrate               # 相机标定
./build/tool_dac8568_smoke           # FT4222 -> DAC8568 硬件冒烟
./build/tool_galvo_smoke             # DAC8568 -> 振镜信号冒烟
./build/tool_calib_solve             # 外参旋转求解
./build/tool_export                  # 离线抽帧导出
./build/tool_transcode               # 已录视频转码
./build/tool_smoke                   # 离线冒烟
./build/tool_lidar_dump              # WS30 雷达原始收包/解析调试
```

大部分工具接受可选的配置文件路径：

```bash
./build/tool_preview config/capture_red_20m.yaml
```

录制工具支持显式参数覆盖：

```bash
./build/tool_record \
  config/capture_red_20m.yaml \
  ./videos/my_session \
  30 \
  indoor_lab plain_wall 20m red
```

不显式传输出目录时，默认保存到 `videos/`。

模型推理覆盖模型路径：

```bash
./build/tool_model_infer \
  config/default.yaml \
  test_data/sample_images \
  models/my_model.onnx
```

离线抽帧导出：

```bash
./build/tool_export /path/to/session_dir /path/to/dataset_root train 200
```

转码已录 session：

```bash
./build/tool_transcode /path/to/videos/<session_id>
```

WS30 雷达调试：

```bash
# 查看 CLI 参数
./build/tool_lidar_dump --help

# 联机请求点云/IMU/SN（无设备时可用于验证超时路径）
./build/tool_lidar_dump --device-ip 192.168.137.200 --iterations 10

# 录制 WS30 原始包
./build/tool_lidar_dump --device-ip 192.168.137.200 --iterations 100 --record-raw /tmp/ws30.rawlog

# 回放原始包并导出 PCD
./build/tool_lidar_dump --replay /tmp/ws30.rawlog --write-pcd /tmp/ws30_pcd
```

## Quick Scripts

```bash
make set-config      # 选择配置文件（交互式，选一次全局生效）
make scan-camera     # 扫描可用采集卡
make competition     # 比赛模式（守护进程 + RTP推流 + ffplay）
make stream          # RTP 推流（自动打开 ffplay 窗口）
make preview         # 本地预览（cv::imshow 窗口，不推流）
make stop            # 停止后台守护进程
make record          # 视频录制
```

守护进程运行时，通过 `/tmp/laser_cmd` FIFO 控制：

```bash
echo "stream on"       > /tmp/laser_cmd   # 推流开关
echo "record on"       > /tmp/laser_cmd   # 录制开关
echo "enemy red"       > /tmp/laser_cmd   # 敌方颜色过滤
echo "backend tensorrt" > /tmp/laser_cmd  # 推理后端切换
echo "ekf off"         > /tmp/laser_cmd   # EKF 直驱模式
echo "quit"            > /tmp/laser_cmd   # 优雅退出
```

## 项目结构

```
laser_guidance/
├── include/               # 公开 API
│   ├── config.hpp         # YAML 配置加载
│   ├── types.hpp          # Frame, TargetObservation
│   ├── pipeline.hpp       # 统一视觉入口
│   └── laser_guidance.hpp
├── src/
│   ├── core/              # 核心模块
│   ├── vision/            # 检测/推理模块
│   ├── capture/           # 视频采集 / WS30 雷达协议与组帧
│   ├── streaming/         # 网络推流 (RTP/UDP/SHM)
│   ├── tracking/          # EKF 跟踪
│   └── io/                # 硬件 I/O (FT4222H 等)
├── tools/                 # 可执行工具
├── tests/                 # 自动化测试
│   ├── core/
│   ├── vision/
│   └── tracking/
├── config/                # YAML 配置文件
├── models/                # .onnx / .engine 模型文件
└── test_data/             # 样本回放资源
```

## C++ API 示例

```cpp
#include "config.hpp"
#include "pipeline.hpp"
#include "types.hpp"

#include "core/frame_format.hpp"
#include "capture/v4l2_capture.hpp"

using namespace rmcs_laser_guidance;

// 加载配置
auto config = load_config("config/default.yaml");

// 打开相机
V4l2Capture capture(config.v4l2);
auto negotiated = capture.open();
if (!negotiated) { /* 处理错误 */ }

// 创建 Pipeline
Pipeline pipeline(config);

// 主循环
while (true) {
    auto frame = capture.read_frame();
    if (!frame) continue;

    auto obs = pipeline.process(*frame);
    if (obs && obs->detected) {
        // 检测到目标
        printf("target at (%.1f, %.1f)\n", obs->center.x, obs->center.y);
    }
}
```

FT4222H 振镜控制：

```cpp
#include "io/ft4222_spi.hpp"

auto spi = Ft4222Spi::open(Ft4222Config{
    .sys_clock = Ft4222SysClock::k60MHz,
    .clock_div = Ft4222SpiDiv::kDiv64,   // SCK ≈ 937 kHz
    .cs_channel = 0                       // SS0O
});

if (spi) {
    uint8_t dac_cmd[2] = {0x30, 0xFF};   // 12-bit DAC value
    spi->write(dac_cmd, 2);
}
```

## Notes

- 默认配置 `config/default.yaml`，采集配置推荐 `config/capture_red_20m.yaml`
- 采集卡设备路径通过 `v4l2.device_path` 配置；`make scan-camera` 可扫描可用设备
- 默认采集模式 `1920x1080 @ 60 FPS`，优先 `mjpeg` 编码
- 默认回放样本位于 `test_data/sample_images`
- 原始视频会话目录默认为 `videos/`
- `models/` 放置 `.onnx`、`.engine`、电压映射模型文件（`vision_voltage_poly_v*.yaml`、`vision_voltage_lut*.yaml`）
- `test_data/calib/` 放置标定 CSV（voltage_records, geometry_calib_records 等）
- `Pipeline` 通过 `inference.backend` 在 `bright_spot` / `model`(ONNX) / `tensorrt`(GPU) 间切换
- TensorRT 需 `-DRMCS_LASER_GUIDANCE_WITH_TENSORRT=ON` 且预先生成 `.engine` 文件
- 模型后端支持 YOLO26 端到端推理，输出 `[1,300,6]`（3 class：purple/red/blue）
- 本仓库不负责本地训练；训练应在外部平台完成，仓库负责数据集生成和模型接入
- EKF 跟踪器 (`EkfTracker`) 为 standalone 模块，常加速度 6 维状态，支持 lookahead 超前预测
- Direct voltage 视觉→电压映射：`config/direct_voltage_run.yaml`（比赛配置）、`config/direct_voltage_calib.yaml`（标定采集）。训练脚本 `scripts/train_voltage_poly.py`，模型版本 `models/vision_voltage_poly_v*.yaml`
- 比赛模式 (`tool_competition`) 融合预览+引导+录制，守护进程启动，支持 RTP 推流（可配码率 `streaming.bitrate`）、视频内录（`record.output_root`）、FIFO 运行时控制
- 双推理后端：ONNX + TensorRT，启动时同时加载，运行时通过 FIFO 或配置 `inference.backend` 切换
- 敌方颜色过滤：`inference.enemy_color` (red/blue/auto)，仅处理敌方颜色+紫色候选框
- EKF 可运行时关闭 (`ekf.enabled: false` 或 `ekf off`)，切换原始检测直驱振镜
- 被瞄准进度 (`HitProgress`)：规则手册 §5.6.3，3 阶段 P0 阈值，45s 锁定，overlay 进度条
- 训练数据链路推荐「先录原始视频会话，再直接上传外部平台」；离线抽帧为备用链路
- 录制输出 `raw.mp4 + session.yaml + notes.txt`，默认 H.264/avc1 编码
- RTP 推流：`make stream` 后台 daemon + ffplay 窗口，关闭即停。`streaming` 配置段控制，默认端口 5004
- `.script/` 提供便捷脚本：`set-config`、`scan-camera`、`preview`、`stream`、`stop`
- WS30 当前只实现 standalone core：`Ws30UdpSocket` / `Ws30PacketParser` / `Ws30FrameAssembler` / `Ws30Client`
- WS30 当前调试路径优先使用 `tool_lidar_dump`；ROS2 bridge、RViz/Foxglove 可视化和 PCD 导出是下一步

## Docs

- `plan.md`
- `docs/architecture.md`
- `docs/hardware_ft4222_dac8568.md`
- `docs/dataset_collection.md`
- `docs/development.md`
- `docs/future_rmcs_integration.md`
- `docs/ws30_lidar.md`
- `docs/ros2_interface.md`

## ROS2 Bridge via Docker ✅

已在 Arch + Docker + Foxglove 环境验证通过。

如果宿主机不是 Ubuntu Noble，推荐直接使用完整容器工作流构建并运行 WS30 ROS2 bridge：

```bash
docker compose -f docker-compose.ws30-bridge.yml up --build
```

也可以用仓库脚本：

```bash
bash scripts/ws30_bridge_docker.sh
```

容器工作流会：

1. 基于 `ros:jazzy-ros-base-noble` 构建镜像
2. 在容器内编译 `external/ws30_lidar_core`
3. 在容器内 `colcon build` `ros2/ws30_lidar_bridge`
4. 通过 `--network host` 直接启动 `ros2 launch ws30_lidar_bridge ws30_lidar.launch.py`
5. 同时启动 `foxglove_bridge`，提供 Foxglove WebSocket 连接（默认 `ws://localhost:8765`）

常用环境变量：

```bash
WS30_DEVICE_IP=192.168.137.200 docker compose -f docker-compose.ws30-bridge.yml up --build
ROS_DOMAIN_ID=7 docker compose -f docker-compose.ws30-bridge.yml up --build
FOXGLOVE_WS_PORT=8765 docker compose -f docker-compose.ws30-bridge.yml up --build
```

也支持三种容器模式：

```bash
# 默认：编译后直接启动 bridge
docker compose -f docker-compose.ws30-bridge.yml up --build

# 只编译，不启动
docker compose -f docker-compose.ws30-bridge.yml run --rm ws30-bridge build

# 编译后进入容器 shell
docker compose -f docker-compose.ws30-bridge.yml run --rm ws30-bridge shell
```

Foxglove WebSocket 连接地址：

```text
ws://localhost:8765
```

如果 Foxglove 在别的机器上运行，把 `localhost` 换成 Docker 宿主机 IP。
