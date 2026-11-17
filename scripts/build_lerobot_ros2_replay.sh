#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
cd "${REPO_ROOT}"
CMAKE_PREFIX_PATH=/opt/ros/humble PATH=/usr/bin:/bin \
  colcon build --symlink-install --cmake-clean-cache \
  --packages-select arm_interfaces custom_image_msg lerobot_ros2_replay eval_model \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so

echo "构建完成。新终端请运行：source scripts/setup_ros2_replay_env.sh"
