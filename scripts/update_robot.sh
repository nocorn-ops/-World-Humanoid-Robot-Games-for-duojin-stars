#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_WORKSPACE="/home/r1lite/duojin_ws"

if [[ ! -d "${DUOJIN_WORKSPACE}/.git" ]]; then
  echo "Not a Git checkout: ${DUOJIN_WORKSPACE}" >&2
  exit 1
fi

cd "${DUOJIN_WORKSPACE}"

if [[ -n "$(git status --porcelain)" ]]; then
  echo "The robot checkout has uncommitted changes; refusing to update." >&2
  git status --short >&2
  exit 1
fi

git pull --ff-only
exec "${DUOJIN_WORKSPACE}/scripts/build_robot.sh"
