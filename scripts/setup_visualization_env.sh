#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${VISUALIZATION_VENV:-${REPO_ROOT}/.venv/visualization}"
UV_BIN="${UV_BIN:-$(command -v uv || true)}"

if [[ -z "${UV_BIN}" || ! -x "${UV_BIN}" ]]; then
  echo "错误：未找到 uv；可通过 UV_BIN 指定 uv 可执行文件。" >&2
  exit 1
fi
if [[ ! -r "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "错误：工作区尚未构建，请先运行 scripts/build_lerobot_ros2_replay.sh。" >&2
  exit 1
fi

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  if [[ -e "${VENV_DIR}" ]]; then
    echo "错误：${VENV_DIR} 已存在，但不是可用的 Python 虚拟环境。" >&2
    exit 1
  fi
  "${UV_BIN}" venv \
    --python /usr/bin/python3 \
    --system-site-packages \
    "${VENV_DIR}"
fi

"${UV_BIN}" pip install \
  --python "${VENV_DIR}/bin/python" \
  --editable "${REPO_ROOT}/frontend" \
  "pytest>=8,<9"

source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
"${VENV_DIR}/bin/python" -c \
  'import rclpy, rerun; from arm_interfaces.msg import ArmStatus; from custom_image_msg.msg import Image1m; assert rerun.__version__ == "0.26.2"; print("Rerun 可视化环境验证通过")'

echo "环境位置：${VENV_DIR}"
