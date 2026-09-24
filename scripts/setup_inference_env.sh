#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${INFERENCE_VENV:-${REPO_ROOT}/.venv/inference}"
OPENPI_CLIENT_SRC="${OPENPI_CLIENT_SRC:-${REPO_ROOT}/inference/pi05_inference}"
UV_BIN="${UV_BIN:-$(command -v uv || true)}"

if [[ -z "${UV_BIN}" || ! -x "${UV_BIN}" ]]; then
  echo "错误：未找到 uv；可通过 UV_BIN 指定 uv 可执行文件。" >&2
  exit 1
fi
if [[ ! -r "${OPENPI_CLIENT_SRC}/openpi_client/websocket_client_policy.py" ]]; then
  echo "错误：OPENPI_CLIENT_SRC 无效：${OPENPI_CLIENT_SRC}" >&2
  exit 1
fi

"${UV_BIN}" venv \
  --python /usr/bin/python3 \
  --system-site-packages \
  "${VENV_DIR}"

"${UV_BIN}" pip install \
  --python "${VENV_DIR}/bin/python" \
  "msgpack>=1.0.5" \
  "typing_extensions>=4.5" \
  "websockets>=11.0"

source /opt/ros/humble/setup.bash
export PYTHONPATH="${OPENPI_CLIENT_SRC}${PYTHONPATH:+:${PYTHONPATH}}"
"${VENV_DIR}/bin/python" -c \
  'import cv2, msgpack, numpy, rclpy, websockets; from openpi_client import websocket_client_policy; print("推理 Python 环境验证通过")'

echo "环境位置：${VENV_DIR}"
