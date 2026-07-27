#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 r1lite@ROBOT_IP" >&2
  exit 2
fi

readonly ROBOT_TARGET="$1"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

rsync -av \
  --exclude build/ \
  --exclude install/ \
  --exclude log/ \
  --exclude __pycache__/ \
  --exclude '*.pyc' \
  "${WORKSPACE_DIR}/" \
  "${ROBOT_TARGET}:/home/r1lite/duojin_ws/"

echo "Source synchronized to ${ROBOT_TARGET}:/home/r1lite/duojin_ws/"
