#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LOCAL_PYTHON="${INFERENCE_VENV:-${REPO_ROOT}/.venv/inference}/bin/python"
PYTHON_BIN="${PYTHON_BIN:-${LOCAL_PYTHON}}"
OPENPI_CLIENT_SRC="${OPENPI_CLIENT_SRC:-${REPO_ROOT}/inference/pi05_inference_mem}"
INFERENCE_SCRIPT="${REPO_ROOT}/inference/pi05_inference_mem/lite_inference_openpi_temporal_smoothing_hil.py"

source "${SCRIPT_DIR}/setup_ros2_replay_env.sh"
export PYTHONPATH="${OPENPI_CLIENT_SRC}:${REPO_ROOT}/inference${PYTHONPATH:+:${PYTHONPATH}}"
set -u

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误：推理环境不存在：${PYTHON_BIN}" >&2
  echo "请先运行 ${SCRIPT_DIR}/setup_inference_env.sh。" >&2
  exit 1
fi
if [[ ! -r "${OPENPI_CLIENT_SRC}/openpi_client/websocket_client_policy.py" ]]; then
  echo "错误：OPENPI_CLIENT_SRC 无效：${OPENPI_CLIENT_SRC}" >&2
  exit 1
fi
if [[ ! -r "${INFERENCE_SCRIPT}" ]]; then
  echo "错误：MEM 推理脚本不存在：${INFERENCE_SCRIPT}" >&2
  exit 1
fi
if ! DEPENDENCY_ERROR="$("${PYTHON_BIN}" -c \
  'import cv2, msgpack, numpy, rclpy, websockets; from openpi_client import websocket_client_policy' \
  2>&1)"; then
  echo "错误：推理 Python 依赖不完整。" >&2
  echo "详情：${DEPENDENCY_ERROR}" >&2
  echo "请运行 ${SCRIPT_DIR}/setup_inference_env.sh 修复环境。" >&2
  exit 1
fi

exec "${PYTHON_BIN}" "${INFERENCE_SCRIPT}" \
  --replay-mode \
  --cmd-topic /execute_arm_cmd_inference_dummy \
  "$@"
