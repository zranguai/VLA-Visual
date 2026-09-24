#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "此脚本需要被 source，不能直接执行：" >&2
  echo "  source scripts/setup_ros2_replay_env.sh" >&2
  exit 2
fi

_replay_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export VLA_REVIEW_ROOT="$(cd -- "${_replay_script_dir}/.." && pwd)"
export ROS_DOMAIN_ID=100
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/vla_review_ros_log}"
export HF_HOME="${HF_HOME:-/tmp/vla_review_hf_home}"
export HF_DATASETS_CACHE="${HF_DATASETS_CACHE:-/tmp/vla_review_hf_cache}"

_replay_restore_nounset=false
if [[ $- == *u* ]]; then
  _replay_restore_nounset=true
  set +u
fi

source /opt/ros/humble/setup.bash
if [[ ! -r "${VLA_REVIEW_ROOT}/install/setup.bash" ]]; then
  echo "错误：工作区尚未构建，请先运行 scripts/build_lerobot_ros2_replay.sh。" >&2
  if [[ "${_replay_restore_nounset}" == true ]]; then set -u; fi
  unset _replay_script_dir _replay_restore_nounset
  return 1
fi
source "${VLA_REVIEW_ROOT}/install/setup.bash"

if [[ "${_replay_restore_nounset}" == true ]]; then set -u; fi
unset _replay_script_dir _replay_restore_nounset

echo "ROS 2 回放环境已加载：VLA_REVIEW_ROOT=${VLA_REVIEW_ROOT}，ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
