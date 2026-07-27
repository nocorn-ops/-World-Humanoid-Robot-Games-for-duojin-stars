#!/usr/bin/env bash
set -euo pipefail

readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly GALAXEA_SETUP="/home/r1lite/galaxea/install/setup.bash"
readonly DUOJIN_WORKSPACE="/home/r1lite/duojin_ws"

for required_file in "${ROS_SETUP}" "${GALAXEA_SETUP}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required environment file not found: ${required_file}" >&2
    exit 1
  fi
done

if [[ ! -d "${DUOJIN_WORKSPACE}/src" ]]; then
  echo "Workspace source directory not found: ${DUOJIN_WORKSPACE}/src" >&2
  exit 1
fi

# ROS 2 Humble setup files may inspect variables that are intentionally unset.
# Temporarily disable nounset while loading underlays, then restore strict mode.
set +u
source "${ROS_SETUP}"
source "${GALAXEA_SETUP}"
set -u

cd "${DUOJIN_WORKSPACE}"
colcon build --symlink-install

echo "Build complete. Load the overlay with:"
echo "source ${DUOJIN_WORKSPACE}/install/setup.bash"
