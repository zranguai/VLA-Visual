# 项目参考

## 路径

- 回放包：`src/lerobot_ros2_replay/`
- 评估包：`src/eval_model/`，默认配置为 `src/eval_model/config/eval_model.yaml`
- 接口包：`src/interface/arm_interfaces/`、`src/interface/custom_image_msg/`
- 推理客户端：`inference/pi05_inference/lite_inference_openpi_temporal_smoothing_hil.py`
- OpenPI 客户端：`inference/pi05_inference/openpi_client/`
- Rerun 可视化：`frontend/`，默认配置为 `frontend/config/visualizer.yaml`
- 默认配置：`src/lerobot_ros2_replay/config/replay.yaml`
- 操作文档：`scripts/README.md`
- 默认样例：`/home/zme/datasets/lerobot_test/episode_2026-09-11_10-46-33`
- 推理虚拟环境：`.venv/inference/`，Python 3.10，继承系统 ROS 2 包。
- 可视化虚拟环境：`.venv/visualization/`，Python 3.10，固定 Rerun 0.26.2。

## 数据布局

LeRobot episode 必须是 v3.0，视频后端使用 `pyav`。ROS/推理使用的 16 维布局为：

```text
[left_joint0..6, left_gripper, right_joint0..6, right_gripper]
```

样例元数据为右臂优先；`dataset_reader.py` 负责按字段名重排。夹爪状态在数据集中为 0–100，推理观测归一化为 0–1。

## 话题和 QoS

| 话题 | 类型 | QoS |
| --- | --- | --- |
| `/arm_status` | `arm_interfaces/msg/ArmStatus` | Reliable，深度 10 |
| `/execute_arm_cmd_dummy` | `arm_interfaces/msg/AutonomyArmCommand` | Reliable，深度 1 |
| `/execute_arm_cmd_inference_dummy` | `arm_interfaces/msg/AutonomyArmCommand` | 推理安全输出 |
| `/camera_dcw2/custom_cam_color_1M` | `custom_image_msg/msg/Image1m` | Best Effort，深度 2 |
| `/camera_dcl_left/custom_cam_color_1M` | `custom_image_msg/msg/Image1m` | Best Effort，深度 2 |
| `/camera_dcl_right/custom_cam_color_1M` | `custom_image_msg/msg/Image1m` | Best Effort，深度 2 |

相机消息为 `rgb8`、640×480、`step=1920`。`Image1m.data` 固定为 1,000,000 字节，有效 RGB 数据之后补零。

## 常用命令

```bash
scripts/build_lerobot_ros2_replay.sh
scripts/setup_inference_env.sh
scripts/setup_visualization_env.sh
scripts/run_replay.sh
scripts/run_visualization.sh
scripts/run_model_eval.sh
scripts/run_inference_replay.sh --host 127.0.0.1 --port 7979 --task "<task>"
```

新终端查询 ROS 2：

```bash
source scripts/setup_ros2_replay_env.sh
ros2 daemon stop
ros2 topic list -t
ros2 topic info -v /execute_arm_cmd_dummy
```

运行测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PYTHONPATH:/home/zme/anaconda/envs/lerobot/lib/python3.10/site-packages"
export LD_LIBRARY_PATH="/home/zme/anaconda/envs/lerobot/lib:$LD_LIBRARY_PATH"
export HF_HOME=/tmp/vla-review-hf-home
export HF_DATASETS_CACHE=/tmp/vla-review-hf-datasets
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 -m pytest -q -p no:cacheprovider src/lerobot_ros2_replay/test
```

## 排障顺序

1. `topic does not appear to be published`：确认回放进程存活，并在查询终端 source 环境脚本。
2. `Could not determine the type`：确认 `install/setup.bash` 已加载且接口包构建成功。
3. `No module named msgpack/websockets`：运行 `scripts/setup_inference_env.sh`。
4. 推理动作话题不存在：确认策略服务已返回结果；数据集真值话题与预测话题不是同一话题。
5. 图像偶发丢帧：相机使用 Best Effort 且单帧接近 1 MB，命令行打印过慢时丢帧可能是预期行为。
6. Rerun 无法导入或版本不匹配：重新运行 `scripts/setup_visualization_env.sh`。
7. 模型曲线为空：确认推理脚本已经向 `/execute_arm_cmd_inference_dummy` 发布；其他可视化不依赖该话题。
8. 评估报告为 `insufficient_data`：确认模型话题存在、时间戳与回放时钟一致，并达到配置的最小匹配样本数。
9. 报告提示不可用于排名：当前为 `diagnostic` 或训练 split；改用 held-out episode 后再做模型结论。
