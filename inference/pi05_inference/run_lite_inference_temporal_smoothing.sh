#!/usr/bin/env bash
# lite_inference_openpi_temporal_smoothing.py 运行脚本
#
# 下面包含四条命令（每次只保留一条生效）：
#   1) 普通 dry-run 测试命令
#   2) 普通正式命令
#   3) HIL dry-run 测试命令（lite_inference_openpi_temporal_smoothing_hil.py）
#   4) HIL 正式命令
#
# 运行前请确认：roscore/ROS2 节点、相机、机械臂驱动均已启动，
# 并已 source 好 ROS2 + 本项目所需的 Python 环境。
set -euo pipefail

# ROS2 环境（38号机 domain=38）。使用默认值，仍允许调用方通过环境变量覆盖。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-38}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/zme/robot_config/system_config/zme_robot/cyclonedds_config/cyclonedds_out.xml}"

# setup.bash 内部可能引用未定义变量，source 时临时关闭 nounset。
set +u
source /home/zme/ros2_humble_release/install/setup.bash
source /home/zme/robot/system/zme_robot/install/setup.bash
source /home/zme/lite_arm_manager/install/setup.bash
set -u


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLIENT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export PYTHONDONTWRITEBYTECODE="${PYTHONDONTWRITEBYTECODE:-1}"
export PYTHONPATH="${CLIENT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"


<<eof
# ── 1) 普通 dry-run 测试命令 ──────────────────────────────────────────────
python3 "$SCRIPT_DIR/lite_inference_openpi_temporal_smoothing.py" \
  --host 127.0.0.1 \
  --port 8008 \
  --task "make pasta." \
  --hz 30 \
  --inference-rate 3.0 \
  --latency-k 8 \
  --min-smooth-steps 8 \
  --dry-run
eof


# ── 2) 普通正式命令（确认安全距离/急停已就位后，取消注释再运行）────────────
<<eof
python3 "$SCRIPT_DIR/lite_inference_openpi_temporal_smoothing.py" \
  --host 127.0.0.1 \
  --port 8000 \
  --task "stir-fry chili with meat." \
  --hz 30 \
  --inference-rate 3.0 \
  --latency-k 8 \
  --min-smooth-steps 8
eof


# ── 3) HIL dry-run 测试命令 ───────────────────────────────────────────────
# HIL 运行期只通过 /tele_vr_cmd 的 left_button[15] 上升沿切换控制权。
# X/A/Y 等任务可中断模型；任务结束后须再次按左摇杆，模型不会自动恢复。
# --start-armed 保留一次启动自动授权；外部 grant 服务不再授予控制权。
# dry-run 只推理，不下发动作或请求底层模式切换。
<<eof
python3 "$SCRIPT_DIR/lite_inference_openpi_temporal_smoothing_hil.py" \
  --host 127.0.0.1 \
  --port 8008 \
  --task "make pasta." \
  --hz 30 \
  --inference-rate 3.0 \
  --latency-k 8 \
  --min-smooth-steps 8 \
  --tele-vr-cmd-topic /tele_vr_cmd \
  --vla-grant-service /hil/request_vla_autonomy_grant \
  --dry-run
eof
# ── 4) HIL 正式命令（确认安全距离/急停/左摇杆已就位后运行）────────────────

python3 "$SCRIPT_DIR/lite_inference_openpi_temporal_smoothing_hil.py" \
  --host 127.0.0.1 \
  --port 7979 \
  --task "stir-fry chili with meat." \
  --hz 30 \
  --inference-rate 3.0 \
  --latency-k 8 \
  --min-smooth-steps 8 \
  --tele-vr-cmd-topic /tele_vr_cmd \
  --vla-grant-service /hil/request_vla_autonomy_grant \
  --start-armed
