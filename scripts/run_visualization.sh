#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${VISUALIZATION_VENV:-${REPO_ROOT}/.venv/visualization}"
PYTHON_BIN="${VISUALIZATION_PYTHON_BIN:-${VENV_DIR}/bin/python}"
CONFIG_PATH="${VISUALIZATION_CONFIG:-${REPO_ROOT}/frontend/config/visualizer.yaml}"

source "${SCRIPT_DIR}/setup_ros2_replay_env.sh"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误：可视化 Python 环境不存在，请先运行 scripts/setup_visualization_env.sh。" >&2
  exit 1
fi

exec "${PYTHON_BIN}" -m lerobot_ros2_visualizer.node \
  --config "${CONFIG_PATH}" \
  "$@"
