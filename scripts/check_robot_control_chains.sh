#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_CHECK_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DUOJIN_CHECK_SDK_ROOT="${DUOJIN_CHECK_SDK_ROOT:-${HOME}/galaxea/install_430}"
readonly DUOJIN_CHECK_SDK_SETUP="${DUOJIN_CHECK_SDK_ROOT}/setup.bash"
readonly DUOJIN_CHECK_OVERLAY="${DUOJIN_CHECK_WORKSPACE}/install/setup.bash"
readonly DUOJIN_CHECK_TIMEOUT_SECONDS="${DUOJIN_CHECK_TIMEOUT_SECONDS:-12}"

if (( $# > 1 )); then
  echo "Usage: $0 [--arm-motion]" >&2
  exit 2
fi

DUOJIN_CHECK_PROFILE="full"
if (( $# == 1 )); then
  if [[ "$1" != "--arm-motion" ]]; then
    echo "Usage: $0 [--arm-motion]" >&2
    exit 2
  fi
  DUOJIN_CHECK_PROFILE="arm-motion"
fi
readonly DUOJIN_CHECK_PROFILE

for required_file in "${DUOJIN_CHECK_SDK_SETUP}" "${DUOJIN_CHECK_OVERLAY}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required environment file not found: ${required_file}" >&2
    exit 1
  fi
done

if ! command -v timeout >/dev/null 2>&1; then
  echo "Required command not found: timeout" >&2
  exit 1
fi

set +u
source "${DUOJIN_CHECK_SDK_SETUP}"
source "${DUOJIN_CHECK_OVERLAY}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is unavailable after loading the SDK and project environments." >&2
  exit 1
fi

echo "Checking R1 Lite device control chains (profile: ${DUOJIN_CHECK_PROFILE}, read-only)..."
echo "No motion command will be published by this check."

duojin_check_failures=0
duojin_node_list="$(ros2 node list 2>/dev/null || true)"
duojin_full_profile_required=true
if [[ "${DUOJIN_CHECK_PROFILE}" == "arm-motion" ]]; then
  duojin_full_profile_required=false
fi

report_unavailable() {
  local label="$1"
  local detail="$2"
  local required="$3"

  if [[ "${required}" == "true" ]]; then
    printf '  [FAIL] %-27s %s\n' "${label}" "${detail}" >&2
    duojin_check_failures=$((duojin_check_failures + 1))
  else
    printf '  [WARN] %-27s %s (not required by this profile)\n' \
      "${label}" "${detail}" >&2
  fi
}

check_node() {
  local label="$1"
  local node_name="$2"
  local required="${3:-true}"

  if grep -Fxq "${node_name}" <<<"${duojin_node_list}"; then
    printf '  [OK]   %-27s %s\n' "${label}" "${node_name}"
  else
    report_unavailable "${label}" "missing node ${node_name}" "${required}"
  fi
}

subscription_count() {
  local topic="$1"
  ros2 topic info "${topic}" 2>/dev/null \
    | awk '/^Subscription count:/ {print $3; exit}'
}

check_subscription() {
  local label="$1"
  local topic="$2"
  local required="${3:-true}"
  local count
  count="$(subscription_count "${topic}" || true)"

  if [[ "${count}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
    printf '  [OK]   %-27s %s (%s subscriber(s))\n' "${label}" "${topic}" "${count}"
  else
    report_unavailable "${label}" "no subscriber on ${topic}" "${required}"
  fi
}

echo
echo "Controller nodes:"
check_node "chassis controller" "/r1_lite_chassis_control_node" "${duojin_full_profile_required}"
check_node "torso controller" "/mobiman_torso_control_example" "${duojin_full_profile_required}"
check_node "gripper controller" "/r1_gripper_controller" "${duojin_full_profile_required}"
check_node "end-effector pose" "/r1_lite_eepose_pub_node" "${duojin_full_profile_required}"
check_node "arm joint tracker" "/r1_lite_jointTracker_demo_node" "${duojin_full_profile_required}"
check_node "left Relaxed IK" "/relaxed_ik_left" "${duojin_full_profile_required}"
check_node "right Relaxed IK" "/relaxed_ik_right" "${duojin_full_profile_required}"

echo
echo "Public command inputs:"
check_subscription "chassis target" "/motion_target/target_speed_chassis" "${duojin_full_profile_required}"
check_subscription "torso speed target" "/motion_target/target_speed_torso" "${duojin_full_profile_required}"
check_subscription "torso joint target" "/motion_target/target_joint_state_torso" "${duojin_full_profile_required}"
check_subscription "left Cartesian target" "/motion_target/target_pose_arm_left"
check_subscription "right Cartesian target" "/motion_target/target_pose_arm_right"
check_subscription "left joint target" "/motion_target/target_joint_state_arm_left"
check_subscription "right joint target" "/motion_target/target_joint_state_arm_right"
check_subscription "left gripper target" "/motion_target/target_position_gripper_left" "${duojin_full_profile_required}"
check_subscription "right gripper target" "/motion_target/target_position_gripper_right" "${duojin_full_profile_required}"

echo
echo "Controller-to-HDAS actuator links:"
check_subscription "chassis actuator" "/motion_control/control_chassis" "${duojin_full_profile_required}"
check_subscription "torso actuator" "/motion_control/control_torso" "${duojin_full_profile_required}"
check_subscription "left arm actuator" "/motion_control/control_arm_left"
check_subscription "right arm actuator" "/motion_control/control_arm_right"
check_subscription "left gripper actuator" "/motion_control/control_gripper_left" "${duojin_full_profile_required}"
check_subscription "right gripper actuator" "/motion_control/control_gripper_right" "${duojin_full_profile_required}"

declare -a duojin_topic_labels=()
declare -a duojin_topic_names=()
declare -a duojin_topic_pids=()
declare -a duojin_topic_required=()

start_topic_check() {
  local label="$1"
  local topic="$2"
  local required="${3:-true}"

  timeout "${DUOJIN_CHECK_TIMEOUT_SECONDS}s" \
    ros2 topic echo "${topic}" --once \
      --qos-reliability best_effort \
      --qos-durability volatile \
      >/dev/null 2>&1 &
  duojin_topic_labels+=("${label}")
  duojin_topic_names+=("${topic}")
  duojin_topic_pids+=("$!")
  duojin_topic_required+=("${required}")
}

echo
echo "Live device feedback (up to ${DUOJIN_CHECK_TIMEOUT_SECONDS}s, checked in parallel):"
start_topic_check "chassis feedback" "/hdas/feedback_chassis" "${duojin_full_profile_required}"
start_topic_check "torso feedback" "/hdas/feedback_torso" "${duojin_full_profile_required}"
start_topic_check "left arm feedback" "/hdas/feedback_arm_left"
start_topic_check "right arm feedback" "/hdas/feedback_arm_right"
start_topic_check "left gripper feedback" "/hdas/feedback_gripper_left" "${duojin_full_profile_required}"
start_topic_check "right gripper feedback" "/hdas/feedback_gripper_right" "${duojin_full_profile_required}"
start_topic_check "chassis IMU" "/hdas/imu_chassis" "${duojin_full_profile_required}"
start_topic_check "torso IMU" "/hdas/imu_torso" "${duojin_full_profile_required}"
start_topic_check "battery/BMS" "/hdas/bms" "${duojin_full_profile_required}"
start_topic_check "left current EE pose" "/relaxed_ik/motion_control/pose_ee_arm_left"
start_topic_check "right current EE pose" "/relaxed_ik/motion_control/pose_ee_arm_right"
duojin_camera_required="${duojin_full_profile_required}"
if [[ "${duojin_camera_required}" == "false" ]]; then
  echo "Camera streams are reported but are not prerequisites for manual arm-motion validation."
fi
start_topic_check "head left color" "/hdas/camera_head/left_raw/image_raw_color/compressed" "${duojin_camera_required}"
start_topic_check "head right color" "/hdas/camera_head/right_raw/image_raw_color/compressed" "${duojin_camera_required}"
start_topic_check "left wrist color" "/hdas/camera_wrist_left/color/image_raw/compressed" "${duojin_camera_required}"
start_topic_check "left wrist depth" "/hdas/camera_wrist_left/aligned_depth_to_color/image_raw" "${duojin_camera_required}"
start_topic_check "right wrist color" "/hdas/camera_wrist_right/color/image_raw/compressed" "${duojin_camera_required}"
start_topic_check "right wrist depth" "/hdas/camera_wrist_right/aligned_depth_to_color/image_raw" "${duojin_camera_required}"

for index in "${!duojin_topic_pids[@]}"; do
  if wait "${duojin_topic_pids[${index}]}"; then
    printf '  [OK]   %-27s %s\n' \
      "${duojin_topic_labels[${index}]}" "${duojin_topic_names[${index}]}"
  else
    if [[ "${duojin_topic_required[${index}]}" == "true" ]]; then
      printf '  [FAIL] %-27s no message on %s\n' \
        "${duojin_topic_labels[${index}]}" "${duojin_topic_names[${index}]}" >&2
      duojin_check_failures=$((duojin_check_failures + 1))
    else
      printf '  [WARN] %-27s no message on %s (not required by this profile)\n' \
        "${duojin_topic_labels[${index}]}" "${duojin_topic_names[${index}]}" >&2
    fi
  fi
done

echo
if (( duojin_check_failures > 0 )); then
  echo "Robot control-chain check failed: ${duojin_check_failures} item(s) unavailable." >&2
  echo "Inspect the SDK tmux sessions and the listed nodes/topics before autonomous control." >&2
  exit 1
fi

echo "All control chains required by the ${DUOJIN_CHECK_PROFILE} profile are online."
