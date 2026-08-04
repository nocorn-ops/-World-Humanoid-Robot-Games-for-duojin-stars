#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_WORKSPACE="$(cd "${DUOJIN_SCRIPT_DIR}/.." && pwd)"
readonly GALAXEA_SETUP="${HOME}/galaxea/install_430/setup.bash"
readonly DUOJIN_SETUP="${DUOJIN_WORKSPACE}/install/setup.bash"

for required_file in "${GALAXEA_SETUP}" "${DUOJIN_SETUP}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required environment file not found: ${required_file}" >&2
    exit 1
  fi
done

# ROS 2/ament setup files are not nounset-safe on every Humble installation.
set +u
source "${GALAXEA_SETUP}"
source "${DUOJIN_SETUP}"
set -u

execute_requested=false
for argument in "$@"; do
  if [[ "${argument}" == "execute:=true" ]]; then
    execute_requested=true
    break
  fi
done

if [[ "${execute_requested}" == true ]]; then
  echo "Physical Cartesian diagnostic execution is disabled." >&2
  echo "Vendor Relaxed IK publishes joints before project-side validation; use the " \
    "unified API preview and wait for the documented IK isolation work." >&2
  exit 2
fi

exec ros2 run duojin_arm_test move_arm_to_pose "$@"
