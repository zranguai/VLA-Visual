# LeRobot ROS 2 Replay

该包把 LeRobot v3.0 episode 回放为 Lite 机器人使用的 ROS 2 话题。默认配置只在
`ROS_DOMAIN_ID=100` 中运行，不会与真实机器人使用的 Domain 38 通信。统一的构建、
环境设置和运行入口请参阅仓库根目录的 `scripts/README.md`。

## 构建

```bash
cd /home/zme/vla-2026/VLA-Review
scripts/build_lerobot_ros2_replay.sh
```

显式指定系统 Python 3.10 可避免 CMake 误用 Conda Python 3.12，造成 ROS 消息扩展 ABI 不兼容。

回放数据依赖 Python 3.10 的 LeRobot 环境。本机已验证环境：
`/home/zme/anaconda/envs/lerobot/bin/python`（LeRobot 0.4.4、PyAV 15.1）。

## 回放

先按需编辑 `config/replay.yaml`，再运行：

```bash
scripts/run_replay.sh
```

`hz: 0` 会关闭对应话题；低于数据集 FPS 时跳帧，高于 FPS 时重复最近的数据帧。
`end_frame` 为不包含上界的 episode 内帧号。默认播放一次后退出；设置
`playback.loop: true` 可循环播放。

## 推理联调

首次使用时，创建基于系统 Python 3.10、可访问 ROS 2 包的轻量环境：

```bash
scripts/setup_inference_env.sh
```

该脚本仅安装 `msgpack`、`typing_extensions` 和 `websockets` 到项目的
`.venv/inference`，不会修改系统 Python。然后另开终端运行：

```bash
scripts/run_inference_replay.sh \
  --host 127.0.0.1 \
  --port 7979 \
  --task "stir-fry chili with meat."
```

脚本固定启用 `--replay-mode`，预测动作发布到
`/execute_arm_cmd_inference_dummy`。数据集真值动作位于
`/execute_arm_cmd_dummy`。Replay 模式会拒绝真实 `/execute_arm_cmd`。

默认使用 `inference/pi05_inference/openpi_client`；如客户端不在默认位置，通过
`OPENPI_CLIENT_SRC` 指定包含 `openpi_client` 目录的父目录；
通过 `PYTHON_BIN` 指定包含 `websockets`、`msgpack`、NumPy 和 OpenCV 的 Python 3.10。
