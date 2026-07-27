#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_WORKSPACE="$(cd "${DUOJIN_SCRIPT_DIR}/.." && pwd)"
readonly GALAXEA_SETUP="${HOME}/galaxea/install_430/setup.bash"

if [[ ! -f "${GALAXEA_SETUP}" ]]; then
  echo "Galaxea SDK environment not found: ${GALAXEA_SETUP}" >&2
  exit 1
fi

if [[ ! -d "${DUOJIN_WORKSPACE}/src" ]]; then
  echo "Workspace source directory not found: ${DUOJIN_WORKSPACE}/src" >&2
  exit 1
fi

# install_430 is the fixed vendor underlay. Its generated setup file loads the
# Humble prefix it was built against, so do not source a second SDK workspace.
set +u
source "${GALAXEA_SETUP}"
set -u

for required_command in ros2 colcon; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "${required_command} is unavailable after sourcing ${GALAXEA_SETUP}." >&2
    echo "Build this workspace on the robot IPC with ROS 2 Humble installed." >&2
    exit 1
  fi
done

cd "${DUOJIN_WORKSPACE}"
colcon build --symlink-install

echo "Build complete. Load the overlay with:"
echo "source ${DUOJIN_WORKSPACE}/install/setup.bash"
