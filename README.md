# VLA-Review

将 LeRobot v3.0 数据回放为兼容 Lite 机器人的 ROS 2 话题，支持 OpenPI 推理、Rerun 可视化和动作误差评估，并输出 JSON/CSV 报告。

离线回放固定使用 `ROS_DOMAIN_ID=100`，动作仅发布至 dummy 话题。

环境配置、构建与运行见 [操作指南](scripts/README.md)。
