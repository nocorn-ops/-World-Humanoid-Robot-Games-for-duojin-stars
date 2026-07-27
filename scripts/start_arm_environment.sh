#!/usr/bin/env bash
set -euo pipefail

readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly GALAXEA_SETUP="/home/r1lite/galaxea/install/setup.bash"
readonly DUOJIN_SETUP="/home/r1lite/duojin_ws/install/setup.bash"

if (( $# > 1 )); then
  echo "Usage: $0 [left|right]" >&2
  exit 2
fi

readonly ARM="${1:-left}"
if [[ "${ARM}" != "left" && "${ARM}" != "right" ]]; then
  echo "Usage: $0 [left|right]" >&2
  exit 2
fi

for required_file in "${ROS_SETUP}" "${GALAXEA_SETUP}" "${DUOJIN_SETUP}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required environment file not found: ${required_file}" >&2
    if [[ "${required_file}" == "${DUOJIN_SETUP}" ]]; then
      echo "Build the workspace first: /home/r1lite/duojin_ws/scripts/build_robot.sh" >&2
    fi
    exit 1
  fi
done

# ROS 2/ament setup files are not nounset-safe on every Humble installation.
set +u
source "${ROS_SETUP}"
source "${GALAXEA_SETUP}"
source "${DUOJIN_SETUP}"
set -u

readonly TRACKER_NODE="/r1_lite_jointTracker_demo_node"
readonly IK_NODE="/relaxed_ik_${ARM}"
readonly FEEDBACK_TOPIC="/hdas/feedback_arm_${ARM}"
readonly TARGET_TOPIC="/motion_target/target_pose_arm_${ARM}"
readonly CURRENT_POSE_TOPIC="/relaxed_ik/motion_control/pose_ee_arm_${ARM}"

STARTED_PIDS=()

node_is_running() {
  local expected_node="$1"
  ros2 node list 2>/dev/null | grep -Fxq "${expected_node}"
}

stop_started_nodes() {
  local exit_code=$?
  trap - EXIT INT TERM

  if (( ${#STARTED_PIDS[@]} > 0 )); then
    echo
    echo "Stopping SDK nodes started by this script..."
    local pid
    for pid in "${STARTED_PIDS[@]}"; do
      kill -INT "${pid}" 2>/dev/null || true
    done
    for pid in "${STARTED_PIDS[@]}"; do
      wait "${pid}" 2>/dev/null || true
    done
  fi

  exit "${exit_code}"
}

trap stop_started_nodes EXIT INT TERM

echo "Loading R1 Lite ${ARM}-arm control environment..."

echo "Checking live arm feedback on ${FEEDBACK_TOPIC}..."
if ! timeout 10s ros2 topic echo "${FEEDBACK_TOPIC}" --once >/dev/null 2>&1; then
  echo "No ${ARM}-arm feedback received within 10 seconds." >&2
  echo "Check HDAS, robot power, emergency stop, and ROS_DOMAIN_ID." >&2
  exit 1
fi

if node_is_running "${TRACKER_NODE}"; then
  echo "Joint Tracker already running; reusing ${TRACKER_NODE}."
else
  echo "Starting Joint Tracker with fast_mode=false..."
  ros2 launch mobiman r1_lite_jointTrackerdemo_launch.py fast_mode:=false &
  STARTED_PIDS+=("$!")
fi

if node_is_running "${IK_NODE}"; then
  echo "${ARM^}-arm Relaxed IK already running; reusing ${IK_NODE}."
else
  echo "Starting ${ARM}-arm Relaxed IK..."
  ros2 launch mobiman "r1_lite_${ARM}_arm_relaxed_ik_launch.py" &
  STARTED_PIDS+=("$!")
fi

echo "Waiting up to 20 seconds for SDK nodes..."
readonly READY_DEADLINE=$((SECONDS + 20))
while (( SECONDS < READY_DEADLINE )); do
  if node_is_running "${TRACKER_NODE}" && node_is_running "${IK_NODE}"; then
    break
  fi
  sleep 1
done

if ! node_is_running "${TRACKER_NODE}" || ! node_is_running "${IK_NODE}"; then
  echo "SDK nodes did not become ready. Review the launch output above." >&2
  exit 1
fi

echo "Waiting for Relaxed IK output on ${CURRENT_POSE_TOPIC}..."
if ! timeout 10s ros2 topic echo "${CURRENT_POSE_TOPIC}" --once >/dev/null 2>&1; then
  echo "Relaxed IK started but did not publish the current end-effector pose." >&2
  echo "Review its launch output and verify ${FEEDBACK_TOPIC}." >&2
  exit 1
fi

echo
echo "R1 Lite ${ARM}-arm environment is ready."
echo "  feedback: ${FEEDBACK_TOPIC}"
echo "  IK target: ${TARGET_TOPIC}"
echo
echo "In another terminal, preview the target without moving the arm:"
echo "  cd /home/r1lite/duojin_ws"
if [[ "${ARM}" == "left" ]]; then
  echo "  ./scripts/run_arm_ik_test.sh"
else
  echo "  ./scripts/run_arm_ik_test.sh --ros-args -p arm:=right"
fi
echo

if (( ${#STARTED_PIDS[@]} == 0 )); then
  echo "All required SDK nodes were already running; this script started nothing."
  trap - EXIT INT TERM
  exit 0
fi

echo "Keep this terminal open. Press Ctrl+C to stop only the nodes started here."
wait -n "${STARTED_PIDS[@]}"
echo "A managed SDK launch process exited; shutting down the remaining managed nodes." >&2
