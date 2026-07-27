#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
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

if [[ "${execute_requested}" == true ]] && command -v tmux >/dev/null 2>&1 \
  && tmux has-session -t r1lite_teleop 2>/dev/null; then
  echo "Refusing autonomous arm motion while the r1lite_teleop session is running." >&2
  echo "Stop that SDK teleoperation session, then run this command again." >&2
  exit 1
fi

exec ros2 run duojin_arm_test move_arm_to_pose "$@"
