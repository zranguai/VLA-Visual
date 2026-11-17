# Model Action Evaluation

该 ROS 2 包在 Domain 100 中只读订阅数据集动作和模型动作，对两路异步动作进行
时间对齐，并生成可复查的端到端评估报告。

## 运行

从仓库根目录构建并启动：

```bash
scripts/build_lerobot_ros2_replay.sh
scripts/run_model_eval.sh --run-label pi05-checkpoint-name
```

默认配置位于 `config/eval_model.yaml`，默认读取
`src/lerobot_ros2_replay/config/replay.yaml` 中的数据集路径。回放停止并空闲 5 秒后，
评估节点自动写入：

```text
outputs/eval_model/<timestamp>-<run-label>/summary.json
outputs/eval_model/<timestamp>-<run-label>/aligned_samples.csv
```

也可通过环境变量 `MODEL_EVAL_CONFIG`、`REPLAY_CONFIG` 和
`MODEL_EVAL_RUN_LABEL` 覆盖默认值，或通过 `--output-dir` 指定本次运行的精确目录。

## 指标语义

- 关节误差按数据集 action 的 `q99-q01` 归一化。
- 夹爪先裁剪到 `[0,100]`，连续误差按 100 归一化，并单独计算开合状态及事件指标。
- 正的 `estimated_delay_ms` 表示模型动作落后于数据集动作。
- `diagnostic` 模式和训练 split 上的结果不会标记为可用于模型排名。
- 报告不生成单一总分；应联合查看 NMAE、P95、延迟、夹爪事件和运动区间指标。

该节点不会创建 ROS 2 publisher、service 或 action client，也不会连接真实机器人
Domain 38。
