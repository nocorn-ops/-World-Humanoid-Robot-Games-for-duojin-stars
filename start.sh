#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_SDK_START="${DUOJIN_WORKSPACE}/scripts/start_robot_sdk.sh"
readonly DUOJIN_CONTROL_CHECK="${DUOJIN_WORKSPACE}/scripts/check_robot_control_chains.sh"
readonly DUOJIN_ARM_API_START="${DUOJIN_WORKSPACE}/scripts/start_arm_api.sh"
readonly DUOJIN_SDK_SETUP="${HOME}/galaxea/install_430/setup.bash"
readonly DUOJIN_OVERLAY="${DUOJIN_WORKSPACE}/install/setup.bash"
readonly DUOJIN_ARM_API_SESSION="duojin_arm_api"

stop_ehi_gateway() {
  local gateway_pattern
  local quiet_checks=0
  local saw_gateway=false
  local -a gateway_pids=()
  gateway_pattern='(^|[[:space:]/])uvicorn[[:space:]]+ehi_gateway\.main:app([[:space:]]|$)'

  for _attempt in {1..75}; do
    mapfile -t gateway_pids < <(
      pgrep -u "$(id -u)" -f "${gateway_pattern}" 2>/dev/null || true
    )
    if (( ${#gateway_pids[@]} == 0 )); then
      quiet_checks=$((quiet_checks + 1))
      if [[ "${saw_gateway}" == "false" ]] && (( quiet_checks >= 15 )); then
        echo "EHI gateway remained stopped for 3 seconds."
        return 0
      fi
    else
      quiet_checks=0
      if [[ "${saw_gateway}" == "false" ]]; then
        echo "Stopping EHI gateway arm-target publisher: ${gateway_pids[*]}"
        saw_gateway=true
      fi
      kill -TERM "${gateway_pids[@]}" 2>/dev/null || true
    fi
    sleep 0.2
  done

  mapfile -t gateway_pids < <(
    pgrep -u "$(id -u)" -f "${gateway_pattern}" 2>/dev/null || true
  )
  if (( ${#gateway_pids[@]} == 0 )); then
    echo "EHI gateway remained suppressed throughout the 15-second observation."
    return 0
  fi

  echo "EHI gateway kept respawning during the 15-second observation." >&2
  echo "Stop its owning launcher before autonomous arm control." >&2
  return 1
}

if (( $# > 1 )) || { (( $# == 1 )) && [[ "$1" != "--enable-arm-motion" ]]; }; then
  echo "Usage: $0 [--enable-arm-motion]" >&2
  exit 2
fi

DUOJIN_ARM_API_MODE="preview"
if (( $# == 1 )); then
  DUOJIN_ARM_API_MODE="execute"
fi
readonly DUOJIN_ARM_API_MODE

for required_file in \
  "${DUOJIN_SDK_START}" \
  "${DUOJIN_CONTROL_CHECK}" \
  "${DUOJIN_ARM_API_START}" \
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

if tmux has-session -t "${DUOJIN_ARM_API_SESSION}" 2>/dev/null; then
  echo "Arm API tmux session already exists: ${DUOJIN_ARM_API_SESSION}" >&2
  echo "Use ./stop.sh before starting another complete environment." >&2
  exit 1
fi

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
echo "Removing the EHI gateway from autonomous arm control ownership..."
stop_ehi_gateway

echo
if [[ "${DUOJIN_ARM_API_MODE}" == "execute" ]]; then
  "${DUOJIN_CONTROL_CHECK}" --arm-motion
else
  "${DUOJIN_CONTROL_CHECK}"
fi

set +u
source "${DUOJIN_SDK_SETUP}"
source "${DUOJIN_OVERLAY}"
set -u

echo
if [[ "${DUOJIN_ARM_API_MODE}" == "execute" ]]; then
  echo "WARNING: starting the arm API with real execution permission."
  echo "Each goal still requires execute=true; keep the emergency stop ready."
else
  echo "Starting the arm API in preview-only mode (no motion targets)."
fi
tmux new-session -d -s "${DUOJIN_ARM_API_SESSION}" \
  "exec bash '${DUOJIN_ARM_API_START}' '${DUOJIN_ARM_API_MODE}'"

arm_api_is_ready() {
  local graph_interfaces required
  local -a required_interfaces=(
    "/duojin/arm/left/move_to [duojin_interfaces/action/MoveArmPose]"
    "/duojin/arm/left/move_by [duojin_interfaces/action/MoveArmRelative]"
    "/duojin/arm/left/move_joints [duojin_interfaces/action/MoveArmJoints]"
    "/duojin/arm/right/move_to [duojin_interfaces/action/MoveArmPose]"
    "/duojin/arm/right/move_by [duojin_interfaces/action/MoveArmRelative]"
    "/duojin/arm/right/move_joints [duojin_interfaces/action/MoveArmJoints]"
    "/duojin/arm/left/current_pose [geometry_msgs/msg/PoseStamped]"
    "/duojin/arm/right/current_pose [geometry_msgs/msg/PoseStamped]"
  )
  graph_interfaces="$(ros2 action list -t 2>/dev/null || true)"
  graph_interfaces+=$'\n'"$(ros2 topic list -t 2>/dev/null || true)"
  for required in "${required_interfaces[@]}"; do
    grep -Fxq "${required}" <<<"${graph_interfaces}" || return 1
  done
}

arm_api_ready=false
for _attempt in {1..20}; do
  if arm_api_is_ready; then
    arm_api_ready=true
    break
  fi
  if ! tmux has-session -t "${DUOJIN_ARM_API_SESSION}" 2>/dev/null; then
    break
  fi
  sleep 0.5
done

if [[ "${arm_api_ready}" != true ]]; then
  echo "Arm API did not expose six Actions and two pose topics within 10 seconds." >&2
  echo "Inspect: tmux capture-pane -pt ${DUOJIN_ARM_API_SESSION}" >&2
  exit 1
fi

echo
echo "=== Duojin environment is ready ==="
echo "The SDK, project overlay, and one arm API server are running."
echo "Arm API mode: ${DUOJIN_ARM_API_MODE}; tmux session: ${DUOJIN_ARM_API_SESSION}."
echo "Python/ROS callers can now run from the ready shell. Python defaults to preview."
echo "Pose display: ros2 run duojin_robot_interface arm_pose_display --ros-args -p arm:=left"
echo "Real motion needs both ./start.sh --enable-arm-motion and goal execute=true."
echo
echo "Arm API guide: docs/runbooks/arm-motion-api.md"
echo "You can also run the read-only diagnostic ./scripts/run_arm_ik_test.sh or another ROS command."
echo "Use ./stop.sh from another terminal to stop the SDK and remaining ROS 2 processes."

cd "${DUOJIN_WORKSPACE}"
export PS1='[duojin] \u@\h:\w\$ '
exec bash --noprofile --norc -i
