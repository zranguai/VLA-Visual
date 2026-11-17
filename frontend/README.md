# LeRobot ROS 2 Rerun Visualizer

该项目从 ROS 2 Domain 100 只读订阅 LeRobot 回放和 OpenPI 推理话题，并在
Rerun Viewer 中同步展示三路相机、机械臂状态、数据集动作和模型动作。

## 界面

- 相机区域提供 `All cameras`、`Head`、`Left wrist`、`Right wrist` 标签页。
- 曲线区域为左右各 7 个关节及两个夹爪提供独立标签页。
- `State`、`Dataset action`、`Model action` 使用固定的青蓝、琥珀和洋红色。
- 在左侧 Blueprint 面板中点击眼睛按钮，可随时隐藏或显示任意曲线来源。
- 底部时间轴支持暂停、回放、缩放和历史定位。
- `Evaluation` 标签页显示关节归一化误差、1 秒滚动均值、夹爪状态匹配和逐信号误差。

默认配置位于 `config/visualizer.yaml`。`signals.sources` 控制订阅的数据来源，
`signals.default_visible_sources` 控制启动时可见的曲线；后者必须是前者的子集。
相机默认以 10 Hz、JPEG quality 85 写入 Rerun，避免三路原始 30 Hz 图像导致
过快的内存增长。

## 运行

所有命令从仓库根目录执行：

```bash
scripts/setup_visualization_env.sh
scripts/run_replay.sh
scripts/run_visualization.sh
```

模型动作对比需要同时运行推理脚本。无模型动作发布时，其他画面和曲线仍正常更新。
评估结果默认写入 `outputs/eval_model/<run-id>/`；训练数据上的默认结果标记为
diagnostic，不作为泛化能力结论。使用 `--no-evaluation` 可完全关闭评估。

可指定其他配置，或无界面记录 `.rrd`：

```bash
scripts/run_visualization.sh --config /absolute/path/to/visualizer.yaml
scripts/run_visualization.sh --save /tmp/lerobot-replay.rrd
```

可视化节点不会发布 ROS 2 话题、调用服务或连接真实机器人控制 Domain。
