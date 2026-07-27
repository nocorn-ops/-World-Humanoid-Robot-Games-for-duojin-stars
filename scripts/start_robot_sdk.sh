#!/usr/bin/env bash
set -euo pipefail

readonly SETUP_CAN_SCRIPT="${HOME}/setup_can.sh"
readonly CAN_SCRIPT="${HOME}/can.sh"
readonly GALAXEA_ROOT="${HOME}/galaxea/install_430"
readonly GALAXEA_SETUP="${GALAXEA_ROOT}/setup.bash"
readonly STARTUP_DIR="${GALAXEA_ROOT}/startup_config/share/startup_config/script"
readonly SESSION_CONFIG="../sessions.d/ATCStandard/R1LITEBody.d/"

for required_file in \
  "${SETUP_CAN_SCRIPT}" \
  "${CAN_SCRIPT}" \
  "${GALAXEA_SETUP}" \
  "${STARTUP_DIR}/robot_startup.sh"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required robot file not found: ${required_file}" >&2
    exit 1
  fi
done

echo "Configuring CAN-FD with the two required robot scripts..."
bash "${SETUP_CAN_SCRIPT}"
bash "${CAN_SCRIPT}"

echo "Starting the complete install_430 SDK..."
cd "${STARTUP_DIR}"
./robot_startup.sh boot "${SESSION_CONFIG}"

echo "Waiting 30 seconds for the SDK sessions..."
sleep 30

set +u
source "${GALAXEA_SETUP}"
set -u

if ! ros2 topic list | grep -Fq "/motion_target/"; then
  echo "SDK startup returned, but no /motion_target topics were discovered." >&2
  echo "Inspect the SDK tmux sessions before starting duojin_ws." >&2
  exit 1
fi

echo "SDK motion-target topics are online."
echo "Run scripts/start_arm_environment.sh to perform the arm-specific checks."
echo "Note: R1LITEBody.d also starts r1lite_teleop; stop that tmux session before autonomous arm control."
