#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_ROOT="${MANYMOVE_WS:-/opt/manymove_ws}"
SRC_DIR="${WORKSPACE_ROOT}/src"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"

log() {
  echo "[setup-workspace] $*"
}

if [[ ! -d "${SRC_DIR}" ]]; then
  log "Nothing to do: ${SRC_DIR} does not exist."
  exit 0
fi

if [[ -z "$(ls -A "${SRC_DIR}")" ]]; then
  log "Workspace source tree is empty at ${SRC_DIR}; clone ManyMove before re-running."
  exit 0
fi

log "Using ROS distro '${ROS_DISTRO}'."

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [[ "$(id -u)" -eq 0 && ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log "Initializing rosdep databases."
  if ! rosdep init; then
    log "Warning: rosdep init failed. Continuing anyway."
  fi
fi

log "Updating rosdep indexes."
if ! rosdep update; then
  log "Warning: rosdep update failed. Continuing anyway."
fi

log "Installing system dependencies with rosdep."
if ! rosdep install --from-paths "${SRC_DIR}" --ignore-src --rosdistro "${ROS_DISTRO}" -y -r; then
  log "Warning: rosdep install encountered issues. Check the output above."
fi

BUILD_CMD=("colcon" "build" "--symlink-install")
if [[ $# -gt 0 ]]; then
  BUILD_CMD=("colcon" "build" "$@")
fi

log "Invoking ${BUILD_CMD[*]}."
if command -v gosu >/dev/null 2>&1 && id "${USERNAME:-}" >/dev/null 2>&1 && [[ "$(id -u)" -eq 0 ]]; then
  gosu "${USERNAME}" bash -lc "set -eo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash
cd \"${WORKSPACE_ROOT}\"
${BUILD_CMD[*]}"
else
  cd "${WORKSPACE_ROOT}"
  "${BUILD_CMD[@]}"
fi

if [[ -f "${INSTALL_SETUP}" ]]; then
  log "Build complete. Source the overlay with: source ${INSTALL_SETUP}"
else
  log "Build finished but ${INSTALL_SETUP} was not generated."
fi
