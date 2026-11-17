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
| `scripts/setup_visualization_env.sh` | 创建 `.venv/visualization` 并安装 Rerun 0.26.2 |
| `scripts/setup_ros2_replay_env.sh` | 为当前终端加载 ROS 2、工作区和 Domain 100 |
| `scripts/run_replay.sh` | 读取数据集并发布状态、动作真值和三路相机 |
| `scripts/run_inference_replay.sh` | 接收回放观测、连接策略服务并发布推理动作 |
| `scripts/run_visualization.sh` | 只读订阅回放/推理话题并启动 Rerun Viewer |
| `scripts/run_model_eval.sh` | 比较数据集与模型动作并输出 JSON/CSV 评估报告 |

## 首次初始化

构建 ROS 2 工作区：

```bash
scripts/build_lerobot_ros2_replay.sh
```

创建推理环境。该环境位于 `.venv/inference`，不会修改系统 Python：

```bash
scripts/setup_inference_env.sh
```

创建独立的可视化环境。该环境位于 `.venv/visualization`，使用 Python 3.10 和
`rerun-sdk==0.26.2`：

```bash
scripts/setup_visualization_env.sh
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

终端 3：启动 Rerun 实时可视化。推理未启动时，三路相机、state 和数据集 action
仍会正常显示：

```bash
scripts/run_visualization.sh
```

默认配置位于 `frontend/config/visualizer.yaml`。相机、关节和曲线来源可在配置中
限制；运行后可通过 Rerun 的相机/关节标签页及 Blueprint 眼睛按钮即时切换。
可视化默认同时启用动作评估，在 `Evaluation` 标签页显示实时归一化误差，并在
`outputs/eval_model/` 写入 `summary.json` 和 `aligned_samples.csv`。模型标识可通过
`MODEL_EVAL_RUN_LABEL` 设置：

```bash
MODEL_EVAL_RUN_LABEL=pi05-checkpoint-10000 scripts/run_visualization.sh
```

仅查看画面、不生成评估报告时使用 `scripts/run_visualization.sh --no-evaluation`。
无界面记录可用于测试或离线查看：

```bash
scripts/run_visualization.sh --save /tmp/lerobot-replay.rrd
```

终端 4（可选）：不启动 Rerun，独立生成可批处理的评估报告。不要与已启用评估的
可视化进程同时用于同一个 run label，以免产生两份报告。

```bash
MODEL_EVAL_RUN_LABEL=pi05-checkpoint-10000 scripts/run_model_eval.sh
```

默认评估配置为 `src/eval_model/config/eval_model.yaml`。当前配置使用
`diagnostic` 模式；训练 episode 的结果只用于链路排障，正式比较模型时必须换用
未参与训练的多个 episode，并将 `report.evaluation_mode` 设置为 `held_out`。

另一个终端查看话题。环境脚本必须使用 `source`，否则环境变量不能保留在当前终端。

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

出现 `No module named rerun` 或 Rerun 版本不正确时，重新运行
`scripts/setup_visualization_env.sh`。三路相机默认在可视化端降采样到 10 Hz 并以
JPEG quality 85 保存，ROS 2 原话题和推理频率不会被修改。

可通过以下环境变量覆盖默认路径：`PYTHON_BIN`、`INFERENCE_VENV`、
`OPENPI_CLIENT_SRC`、`VISUALIZATION_VENV`、`VISUALIZATION_PYTHON_BIN`、
`VISUALIZATION_CONFIG`、`UV_BIN`。`openpi_client` 默认读取
`inference/pi05_inference/openpi_client`。

评估相关路径还可通过 `MODEL_EVAL_CONFIG`、`MODEL_EVAL_RUN_LABEL` 和
`REPLAY_CONFIG` 覆盖。
