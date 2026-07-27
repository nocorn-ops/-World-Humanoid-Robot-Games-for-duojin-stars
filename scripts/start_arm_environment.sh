#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_WORKSPACE="$(cd "${DUOJIN_SCRIPT_DIR}/.." && pwd)"
readonly GALAXEA_SETUP="${HOME}/galaxea/install_430/setup.bash"
readonly DUOJIN_SETUP="${DUOJIN_WORKSPACE}/install/setup.bash"

if (( $# > 1 )); then
  echo "Usage: $0 [left|right]" >&2
  exit 2
fi

readonly ARM="${1:-left}"
if [[ "${ARM}" != "left" && "${ARM}" != "right" ]]; then
  echo "Usage: $0 [left|right]" >&2
  exit 2
fi

for required_file in "${GALAXEA_SETUP}" "${DUOJIN_SETUP}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required environment file not found: ${required_file}" >&2
    if [[ "${required_file}" == "${DUOJIN_SETUP}" ]]; then
      echo "Build the workspace first: ${DUOJIN_WORKSPACE}/scripts/build_robot.sh" >&2
    fi
    exit 1
  fi
done

# ROS 2/ament setup files are not nounset-safe on every Humble installation.
set +u
source "${GALAXEA_SETUP}"
source "${DUOJIN_SETUP}"
set -u

readonly TRACKER_NODE="/r1_lite_jointTracker_demo_node"
readonly IK_NODE="/relaxed_ik_${ARM}"
readonly FEEDBACK_TOPIC="/hdas/feedback_arm_${ARM}"
readonly TARGET_TOPIC="/motion_target/target_pose_arm_${ARM}"
readonly CURRENT_POSE_TOPIC="/relaxed_ik/motion_control/pose_ee_arm_${ARM}"

node_is_running() {
  local expected_node="$1"
  ros2 node list 2>/dev/null | grep -Fxq "${expected_node}"
}

echo "Checking the R1 Lite ${ARM}-arm SDK environment..."

echo "Checking live arm feedback on ${FEEDBACK_TOPIC}..."
if ! timeout 10s ros2 topic echo "${FEEDBACK_TOPIC}" --once >/dev/null 2>&1; then
  echo "No ${ARM}-arm feedback received within 10 seconds." >&2
  echo "Check HDAS, robot power, emergency stop, and ROS_DOMAIN_ID." >&2
  exit 1
fi

for required_node in "${TRACKER_NODE}" "${IK_NODE}"; do
  if ! node_is_running "${required_node}"; then
    echo "Required SDK node is not running: ${required_node}" >&2
    echo "Start the complete SDK with ${DUOJIN_WORKSPACE}/scripts/start_robot_sdk.sh." >&2
    echo "This project intentionally does not start individual vendor nodes." >&2
    exit 1
  fi
done

echo "Waiting for Relaxed IK output on ${CURRENT_POSE_TOPIC}..."
if ! timeout 10s ros2 topic echo "${CURRENT_POSE_TOPIC}" --once >/dev/null 2>&1; then
  echo "Relaxed IK did not publish the current end-effector pose." >&2
  echo "Review the SDK tmux sessions and verify ${FEEDBACK_TOPIC}." >&2
  exit 1
fi

echo
echo "R1 Lite ${ARM}-arm SDK and project overlay are ready."
echo "  feedback: ${FEEDBACK_TOPIC}"
echo "  IK target: ${TARGET_TOPIC}"
echo
echo "In another terminal, preview the target without moving the arm:"
echo "  cd ${DUOJIN_WORKSPACE}"
if [[ "${ARM}" == "left" ]]; then
  echo "  ./scripts/run_arm_ik_test.sh"
else
  echo "  ./scripts/run_arm_ik_test.sh --ros-args -p arm:=right"
fi
echo

if command -v tmux >/dev/null 2>&1 && tmux has-session -t r1lite_teleop 2>/dev/null; then
  echo
  echo "WARNING: r1lite_teleop is running and may compete for arm target topics."
  echo "Stop that session before executing autonomous arm motion."
fi

echo "This check started no vendor nodes."
