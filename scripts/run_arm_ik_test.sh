#!/usr/bin/env bash
set -euo pipefail

readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly GALAXEA_SETUP="/home/r1lite/galaxea/install/setup.bash"
readonly DUOJIN_SETUP="/home/r1lite/duojin_ws/install/setup.bash"

for required_file in "${ROS_SETUP}" "${GALAXEA_SETUP}" "${DUOJIN_SETUP}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required environment file not found: ${required_file}" >&2
    exit 1
  fi
done

source "${ROS_SETUP}"
source "${GALAXEA_SETUP}"
source "${DUOJIN_SETUP}"

exec ros2 run duojin_arm_test move_arm_to_pose "$@"
