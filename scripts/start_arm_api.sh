#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_WORKSPACE="$(cd "${DUOJIN_SCRIPT_DIR}/.." && pwd)"
readonly DUOJIN_SDK_SETUP="${HOME}/galaxea/install_430/setup.bash"
readonly DUOJIN_OVERLAY="${DUOJIN_WORKSPACE}/install/setup.bash"

if (( $# > 1 )); then
  echo "Usage: $0 [preview|execute]" >&2
  exit 2
fi

readonly DUOJIN_ARM_API_MODE="${1:-preview}"
case "${DUOJIN_ARM_API_MODE}" in
  preview)
    readonly DUOJIN_ARM_API_EXECUTE=false
    ;;
  execute)
    readonly DUOJIN_ARM_API_EXECUTE=true
    ;;
  *)
    echo "Usage: $0 [preview|execute]" >&2
    exit 2
    ;;
esac

for required_file in "${DUOJIN_SDK_SETUP}" "${DUOJIN_OVERLAY}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required environment file not found: ${required_file}" >&2
    exit 1
  fi
done

set +u
source "${DUOJIN_SDK_SETUP}"
source "${DUOJIN_OVERLAY}"
set -u

cd "${DUOJIN_WORKSPACE}"
exec ros2 launch duojin_robot_interface arm_motion.launch.py \
  execute:="${DUOJIN_ARM_API_EXECUTE}"
