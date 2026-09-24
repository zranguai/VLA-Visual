#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${1:-${REPO_ROOT}/src/lerobot_ros2_replay/config/replay.yaml}"
PYTHON_BIN="${PYTHON_BIN:-/home/zme/anaconda/envs/lerobot/bin/python}"

source "${SCRIPT_DIR}/setup_ros2_replay_env.sh"
set -u

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误：LeRobot Python 不可执行：${PYTHON_BIN}" >&2
  exit 1
fi
if [[ ! -r "${CONFIG_PATH}" ]]; then
  echo "错误：配置文件不可读：${CONFIG_PATH}" >&2
  exit 1
fi

exec "${PYTHON_BIN}" -m lerobot_ros2_replay.replay_node --config "${CONFIG_PATH}"
