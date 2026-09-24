---
name: operate-lerobot-ros2-replay
description: 构建、配置、运行、可视化、验证和排查本仓库的 LeRobot v3.0 到 ROS 2 回放及 OpenPI 推理链路。用户要求启动回放、推理或 Rerun 可视化，修改话题或频率、检查自定义接口、查看 ROS 2 话题、处理 Domain/依赖/消息发现问题，或验证离线回放安全性时使用。
---

# 操作 LeRobot ROS 2 回放

从仓库根目录工作。先阅读根目录 `AGENTS.md`；需要精确路径、话题、数据布局、QoS 或测试命令时，再读取 [references/project-map.md](references/project-map.md)。

## 选择工作流

1. 首次配置或构建：运行 `scripts/build_lerobot_ros2_replay.sh`；需要推理时再运行 `scripts/setup_inference_env.sh`。
2. 修改回放参数：编辑 `src/lerobot_ros2_replay/config/replay.yaml`，保持 `ros_domain_id: 100`，校验数据集路径、帧范围和各话题 Hz。
3. 启动回放：运行 `scripts/run_replay.sh [config]`。
4. 启动推理：确认策略服务地址后运行 `scripts/run_inference_replay.sh --host ... --port ... --task ...`。
5. 启动可视化：首次运行 `scripts/setup_visualization_env.sh`，然后运行 `scripts/run_visualization.sh`；界面配置位于 `frontend/config/visualizer.yaml`。
6. 查询或排障：在每个新终端先执行 `source scripts/setup_ros2_replay_env.sh`，再使用 `ros2 topic list -t`、`ros2 topic info -v <topic>` 或 `ros2 topic echo <topic>`。

## 执行约束

- 始终保持 `ROS_DOMAIN_ID=100`；拒绝将离线回放切换到真实机器人 Domain 38。
- 保持真值动作与推理动作分离，分别使用 `/execute_arm_cmd_dummy` 和 `/execute_arm_cmd_inference_dummy`。
- 不得让 Replay 模式发布 `/execute_arm_cmd`，不得调用真实模式切换或控制服务。
- Rerun 可视化节点必须保持只读，固定使用 Domain 100 和 `rerun-sdk==0.26.2`。
- 优先调用根目录 `scripts/` 中的入口，不复制它们的环境逻辑到临时命令。
- 改动接口定义时，同时检查消息构造代码、推理订阅端和接口展示结果。
- 除非用户明确要求并授权，不通过 SSH 操作真实机器人。

## 验证结果

- Shell 修改：运行 `bash -n scripts/*.sh src/lerobot_ros2_replay/scripts/*.sh`。
- Python、配置或消息映射修改：运行 reference 中的 pytest 命令。
- 可视化修改：运行 `.venv/visualization/bin/python -m pytest -q frontend/tests`，并检查 Blueprint、图像限频和 `.rrd` 冒烟输出。
- 接口修改：运行构建脚本，并用 `ros2 interface show` 检查字段。
- 运行链路修改：短暂启动回放，确认五个数据集话题的类型；推理服务可用时再确认预测动作话题。
- 报告实际通过的检查；策略服务或真实机器人未参与时明确说明未做端到端验证。
