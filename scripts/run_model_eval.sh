#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${MODEL_EVAL_CONFIG:-${REPO_ROOT}/src/eval_model/config/eval_model.yaml}"
REPLAY_CONFIG_PATH="${REPLAY_CONFIG:-${REPO_ROOT}/src/lerobot_ros2_replay/config/replay.yaml}"
RUN_LABEL="${MODEL_EVAL_RUN_LABEL:-model}"

source "${SCRIPT_DIR}/setup_ros2_replay_env.sh"

exec /usr/bin/python3 -m eval_model.node \
  --config "${CONFIG_PATH}" \
  --replay-config "${REPLAY_CONFIG_PATH}" \
  --run-label "${RUN_LABEL}" \
  "$@"
