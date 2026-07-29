#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_GALAXEA_ROOT="${HOME}/galaxea/install_430"
readonly DUOJIN_GALAXEA_SETUP="${DUOJIN_GALAXEA_ROOT}/setup.bash"
readonly DUOJIN_SDK_STARTUP_DIR="${DUOJIN_GALAXEA_ROOT}/startup_config/share/startup_config/script"
readonly DUOJIN_ROBOT_STARTUP="${DUOJIN_SDK_STARTUP_DIR}/robot_startup.sh"
readonly DUOJIN_CURRENT_UID="$(id -u)"

echo "=== Stopping Duojin and ROS 2 ==="
echo "The hardware emergency stop remains the primary safety mechanism."

if [[ -f "${DUOJIN_GALAXEA_SETUP}" ]]; then
  echo "Sending a best-effort chassis stop and brake command..."
  (
    set +u
    source "${DUOJIN_GALAXEA_SETUP}"
    set -u

    timeout 5s ros2 topic pub --once \
      /motion_target/target_speed_chassis \
      geometry_msgs/msg/TwistStamped \
      '{twist: {linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}}' \
      >/dev/null 2>&1 || true

    timeout 5s ros2 topic pub --once \
      /motion_target/brake_mode \
      std_msgs/msg/Bool \
      '{data: true}' \
      >/dev/null 2>&1 || true
  )
else
  echo "SDK setup not found; skipping ROS safety commands: ${DUOJIN_GALAXEA_SETUP}" >&2
fi

if [[ -x "${DUOJIN_ROBOT_STARTUP}" ]]; then
  echo "Stopping the complete vendor SDK and its tmux server..."
  (
    set +e
    if [[ -f "${DUOJIN_GALAXEA_SETUP}" ]]; then
      set +u
      source "${DUOJIN_GALAXEA_SETUP}"
      set -u
    fi
    cd "${DUOJIN_SDK_STARTUP_DIR}"
    ./robot_startup.sh kill
  ) || true
else
  echo "Vendor stop script not found: ${DUOJIN_ROBOT_STARTUP}" >&2
fi

declare -A DUOJIN_EXCLUDED_PIDS=()
DUOJIN_ANCESTOR_PID="$$"
while [[ "${DUOJIN_ANCESTOR_PID}" =~ ^[0-9]+$ ]] \
  && (( DUOJIN_ANCESTOR_PID > 1 )); do
  DUOJIN_EXCLUDED_PIDS["${DUOJIN_ANCESTOR_PID}"]=1
  DUOJIN_ANCESTOR_PID="$(
    ps -o ppid= -p "${DUOJIN_ANCESTOR_PID}" 2>/dev/null | tr -d ' '
  )"
done

declare -A DUOJIN_ROS_PID_SET=()

add_ros_pid() {
  local candidate_pid="$1"
  [[ "${candidate_pid}" =~ ^[0-9]+$ ]] || return 0
  [[ -z "${DUOJIN_EXCLUDED_PIDS[${candidate_pid}]:-}" ]] || return 0
  [[ -d "/proc/${candidate_pid}" ]] || return 0
  [[ "$(stat -c %u "/proc/${candidate_pid}" 2>/dev/null || true)" == "${DUOJIN_CURRENT_UID}" ]] || return 0
  DUOJIN_ROS_PID_SET["${candidate_pid}"]=1
}

for process_environment in /proc/[0-9]*/environ; do
  [[ -r "${process_environment}" ]] || continue
  process_pid="${process_environment#/proc/}"
  process_pid="${process_pid%/environ}"
  if grep -zFxq 'ROS_VERSION=2' "${process_environment}" 2>/dev/null; then
    add_ros_pid "${process_pid}"
  fi
done

while IFS= read -r process_pid; do
  add_ros_pid "${process_pid}"
done < <(
  pgrep -u "${DUOJIN_CURRENT_UID}" -f \
    '(^|/)(ros2|_ros2_daemon)([[:space:]]|$)' 2>/dev/null || true
)

if (( ${#DUOJIN_ROS_PID_SET[@]} > 0 )); then
  echo "Stopping remaining ROS 2 processes owned by the current user..."
  for process_pid in "${!DUOJIN_ROS_PID_SET[@]}"; do
    process_description="$(ps -p "${process_pid}" -o args= 2>/dev/null || true)"
    echo "  TERM ${process_pid} ${process_description}"
    kill -TERM "${process_pid}" 2>/dev/null || true
  done

  sleep 3

  for process_pid in "${!DUOJIN_ROS_PID_SET[@]}"; do
    if kill -0 "${process_pid}" 2>/dev/null; then
      echo "  KILL ${process_pid}"
      kill -KILL "${process_pid}" 2>/dev/null || true
    fi
  done
else
  echo "No remaining ROS 2 processes were found for the current user."
fi

echo "ROS 2 shutdown complete."
echo "The current terminal may still contain sourced environment variables; this does not mean nodes are running."
