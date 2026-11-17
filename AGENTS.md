# Repository Guidelines

## 沟通与工作方式

- 始终使用中文回复用户，命令、路径、接口名和代码标识符保持原样。
- 修改前先阅读相关源码、`scripts/README.md` 和配置；不要覆盖无关的用户改动。
- 实现改动后执行与风险相匹配的语法检查、单元测试或 ROS 2 冒烟测试，并如实说明未验证项。

## 项目目标与结构

本项目将 LeRobot v3.0 episode 回放为 Lite 机器人兼容的 ROS 2 话题，并允许 OpenPI 推理客户端消费这些观测。主要目录：

- `src/interface/arm_interfaces/`：机械臂状态和动作消息。
- `src/interface/custom_image_msg/`：固定容量相机消息。
- `src/lerobot_ros2_replay/`：配置解析、数据读取、消息构造、回放节点和测试。
- `src/eval_model/`：动作时间对齐、误差指标、ROS 2 评估节点和 JSON/CSV 报告。
- `inference/pi05_inference/`：时序平滑推理客户端及本地 `openpi_client`。
- `frontend/`：ROS 2 只读订阅、Rerun Blueprint、可视化配置和测试。
- `scripts/`：唯一推荐的构建、环境初始化和运行入口；使用方法见 `scripts/README.md`。

## 环境与常用命令

目标环境为 Ubuntu、ROS 2 Humble 和 Python 3.10。LeRobot 数据解码使用 `pyav`。从仓库根目录运行：

```bash
scripts/build_lerobot_ros2_replay.sh
scripts/setup_inference_env.sh
scripts/setup_visualization_env.sh
scripts/run_replay.sh
scripts/run_visualization.sh
scripts/run_model_eval.sh
scripts/run_inference_replay.sh --host 127.0.0.1 --port 7979 --task "<task>"
```

其他终端查询话题前必须执行：

```bash
source scripts/setup_ros2_replay_env.sh
```

该命令加载 ROS 2、`install/setup.bash` 并设置 `ROS_DOMAIN_ID=100`。

## 数据与接口约定

- 默认数据集和话题频率在 `src/lerobot_ros2_replay/config/replay.yaml` 中配置。
- 内部 16 维状态/动作为 `[left_joint0..6, left_gripper, right_joint0..6, right_gripper]`；数据集元数据为右臂优先时必须显式重排。
- 相机发布 `rgb8`、`640x480` 的 `custom_image_msg/msg/Image1m`，有效负载后补零到固定容量。
- 状态 QoS 为 Reliable/深度 10，动作 Reliable/深度 1，相机 Best Effort/深度 2。
- 可视化固定使用 `rerun-sdk==0.26.2`；配置位于 `frontend/config/visualizer.yaml`。
- 评估使用数据集 action 的 `q01/q99` 归一化关节误差；夹爪统一到 `[0,100]` 后单独评估状态和切换事件。

## 安全边界

- 离线回放固定使用 `ROS_DOMAIN_ID=100`；Domain 38 保留给真实机器人，不得用于回放测试。
- 数据集动作只能发布到 `/execute_arm_cmd_dummy`。
- 推理动作只能发布到 `/execute_arm_cmd_inference_dummy`。
- Replay 模式不得发布到真实 `/execute_arm_cmd`，不得启用 HIL 模式切换或真实机器人控制服务。
- 可视化节点只能订阅话题，不得创建控制 publisher、service 或 action client。
- 评估节点同样只能订阅安全话题；训练 split 或 `diagnostic` 结果不得作为正式模型排名依据。

## 编码与验证

- Python 使用 4 空格缩进、类型标注和清晰的 `snake_case`；配置校验应尽早失败并给出可操作错误。
- Shell 脚本使用 Bash、`set -eo pipefail`，需要保留环境变量的脚本必须提示用户通过 `source` 调用。
- 修改接口后重新运行构建脚本；修改 Python 后至少运行对应 pytest；修改 Shell 后运行 `bash -n scripts/*.sh`。
- 修改可视化后使用 `.venv/visualization/bin/python -m pytest -q frontend/tests`，并至少完成一次 Domain 100 回放冒烟测试。
- 修改评估代码后运行 `src/eval_model/test`，并用精确动作和已知延迟动作验证报告结果。
- 样例数据相关测试依赖 `/home/zme/datasets/lerobot_test/episode_2026-09-11_10-46-33`。
