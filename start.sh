#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_SDK_START="${DUOJIN_WORKSPACE}/scripts/start_robot_sdk.sh"
readonly DUOJIN_ARM_CHECK="${DUOJIN_WORKSPACE}/scripts/start_arm_environment.sh"
readonly DUOJIN_SDK_SETUP="${HOME}/galaxea/install_430/setup.bash"
readonly DUOJIN_OVERLAY="${DUOJIN_WORKSPACE}/install/setup.bash"

for required_file in \
  "${DUOJIN_SDK_START}" \
  "${DUOJIN_ARM_CHECK}" \
  "${DUOJIN_SDK_SETUP}" \
  "${DUOJIN_OVERLAY}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required project file not found: ${required_file}" >&2
    if [[ "${required_file}" == "${DUOJIN_OVERLAY}" ]]; then
      echo "Build the project first with: ${DUOJIN_WORKSPACE}/scripts/build_robot.sh" >&2
    fi
    exit 1
  fi
done

echo "=== Duojin one-command startup ==="
echo "Keep the emergency stop ready and clear the robot workspace."
echo

"${DUOJIN_SDK_START}"

echo
echo "Disabling the SDK teleoperation session for autonomous control..."
if tmux has-session -t r1lite_teleop 2>/dev/null; then
  tmux kill-session -t r1lite_teleop
  echo "Stopped r1lite_teleop."
else
  echo "r1lite_teleop is not running."
fi

echo
"${DUOJIN_ARM_CHECK}" left

echo
echo "=== Duojin environment is ready ==="
echo "Opening a shell with the SDK underlay and project overlay loaded."
echo "Preview the current arm test in the ready shell with:"
echo "  ./scripts/run_arm_ik_test.sh"
echo
echo "You can also run your own ros2 launch/ros2 run command directly."
echo "Use ./stop.sh from another terminal to stop the SDK and remaining ROS 2 processes."

set +u
source "${DUOJIN_SDK_SETUP}"
source "${DUOJIN_OVERLAY}"
set -u

cd "${DUOJIN_WORKSPACE}"
export PS1='[duojin] \u@\h:\w\$ '
exec bash --noprofile --norc -i
