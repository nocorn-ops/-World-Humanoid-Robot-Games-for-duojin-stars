#!/usr/bin/env bash
set -euo pipefail

readonly DUOJIN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DUOJIN_WORKSPACE="$(cd "${DUOJIN_SCRIPT_DIR}/.." && pwd)"

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
