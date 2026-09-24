# LeRobot ROS 2 回放操作指南

本文档说明如何构建工作区、回放 LeRobot v3.0 数据、运行 OpenPI 推理客户端，以及在其他终端查看 ROS 2 话题。所有命令均在仓库根目录执行：

```bash
cd /home/zme/vla-2026/VLA-Review
```

## 脚本说明

| 脚本 | 用途 |
| --- | --- |
| `scripts/build_lerobot_ros2_replay.sh` | 构建两个接口包和回放节点 |
| `scripts/setup_inference_env.sh` | 创建 `.venv/inference` 并安装推理客户端依赖 |
| `scripts/setup_ros2_replay_env.sh` | 为当前终端加载 ROS 2、工作区和 Domain 100 |
| `scripts/run_replay.sh` | 读取数据集并发布状态、动作真值和三路相机 |
| `scripts/run_inference_replay.sh` | 接收回放观测、连接策略服务并发布推理动作 |

## 首次初始化

构建 ROS 2 工作区：

```bash
scripts/build_lerobot_ros2_replay.sh
```

创建推理环境。该环境位于 `.venv/inference`，不会修改系统 Python：

```bash
scripts/setup_inference_env.sh
```

## 运行流程

终端 1：启动数据集回放。

```bash
scripts/run_replay.sh
```

默认配置为 `src/lerobot_ros2_replay/config/replay.yaml`。也可指定其他配置：

```bash
scripts/run_replay.sh /absolute/path/to/replay.yaml
```

终端 2：在策略服务已监听 `127.0.0.1:7979` 后启动推理。

```bash
scripts/run_inference_replay.sh \
  --host 127.0.0.1 \
  --port 7979 \
  --task "stir-fry chili with meat."
```

终端 3：查看话题。环境脚本必须使用 `source`，否则环境变量不能保留在当前终端。

```bash
source scripts/setup_ros2_replay_env.sh
ros2 daemon stop
ros2 topic list -t
ros2 topic echo /execute_arm_cmd_dummy
```

## 话题与安全隔离

所有脚本固定使用 `ROS_DOMAIN_ID=100`，避免连接真实机器人所在的 Domain 38。

| 话题 | 类型 | 来源 |
| --- | --- | --- |
| `/arm_status` | `arm_interfaces/msg/ArmStatus` | 数据集状态 |
| `/execute_arm_cmd_dummy` | `arm_interfaces/msg/AutonomyArmCommand` | 数据集动作真值 |
| `/execute_arm_cmd_inference_dummy` | `arm_interfaces/msg/AutonomyArmCommand` | 模型预测动作 |
| `/camera_dcw2/custom_cam_color_1M` | `custom_image_msg/msg/Image1m` | 头部相机 |
| `/camera_dcl_left/custom_cam_color_1M` | `custom_image_msg/msg/Image1m` | 左手相机 |
| `/camera_dcl_right/custom_cam_color_1M` | `custom_image_msg/msg/Image1m` | 右手相机 |

Replay 模式拒绝向真实 `/execute_arm_cmd` 发布。各话题频率在 `replay.yaml` 中独立设置；`hz: 0` 可关闭对应话题。

## 常见问题

出现 `topic does not appear to be published` 时，先确认回放进程仍在运行，然后在查询终端重新加载环境：

```bash
source scripts/setup_ros2_replay_env.sh
ros2 daemon stop
ros2 topic info -v /execute_arm_cmd_dummy
```

出现 `Could not determine the type` 时，通常是当前终端没有加载 `install/setup.bash`；上述环境脚本会完成加载。出现 `No module named msgpack` 或 `websockets` 时，重新运行 `scripts/setup_inference_env.sh`。

可通过以下环境变量覆盖默认路径：`PYTHON_BIN`、`INFERENCE_VENV`、`OPENPI_CLIENT_SRC`、`UV_BIN`。`openpi_client` 默认读取 `inference/pi05_inference/openpi_client`。
