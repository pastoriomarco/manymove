#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_ROOT="${MANYMOVE_WS:-/opt/manymove_ws}"
SRC_DIR="${WORKSPACE_ROOT}/src"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
GROOT_DIR_DEFAULT="Groot"
GROOT_DIR="${GROOT_SOURCE_DIR:-${SRC_DIR}/${GROOT_DIR_DEFAULT}}"
GROOT_REPO_DEFAULT="https://github.com/pastoriomarco/Groot.git"
GROOT_REPO_URL="${GROOT_REPO:-${GROOT_REPO_DEFAULT}}"
GROOT_BRANCH_DEFAULT="master"
GROOT_BRANCH_NAME="${GROOT_BRANCH:-${GROOT_BRANCH_DEFAULT}}"
GROOT_PINNED_COMMIT="${GROOT_COMMIT:-}"

log() {
  echo "[setup-workspace] $*"
}

run_ws_cmd() {
  local -a prefix=()
  if command -v gosu >/dev/null 2>&1 && [[ -n "${USERNAME:-}" ]] && id "${USERNAME}" >/dev/null 2>&1 && [[ "$(id -u)" -eq 0 ]]; then
    prefix=(gosu "${USERNAME}")
  fi
  if [[ ${#prefix[@]} -gt 0 ]]; then
    "${prefix[@]}" "$@"
  else
    "$@"
  fi
}

ensure_groot() {
  local groot_dir="${GROOT_DIR}"
  local repo="${GROOT_REPO_URL}"
  local branch="${GROOT_BRANCH_NAME}"
  local commit="${GROOT_PINNED_COMMIT}"
  local repo_name
  repo_name="$(basename "${groot_dir}")"
  local just_cloned=false

  if [[ ! -d "${groot_dir}" ]]; then
    log "Cloning Groot from ${repo}${branch:+ (branch ${branch})}."
    run_ws_cmd mkdir -p "$(dirname "${groot_dir}")"
    if [[ -n "${branch}" ]]; then
      run_ws_cmd git -C "$(dirname "${groot_dir}")" clone --recurse-submodules --branch "${branch}" "${repo}" "${repo_name}"
    else
      run_ws_cmd git -C "$(dirname "${groot_dir}")" clone --recurse-submodules "${repo}" "${repo_name}"
    fi
    just_cloned=true
  else
    log "Groot already present at ${groot_dir}; skipping clone."
  fi

  if [[ -d "${groot_dir}/.git" ]]; then
    if [[ "${just_cloned}" == true && -n "${commit}" ]]; then
      log "Checking out Groot commit ${commit}."
      run_ws_cmd git -C "${groot_dir}" fetch --depth 1 origin "${commit}"
      run_ws_cmd git -C "${groot_dir}" checkout --detach "${commit}"
      run_ws_cmd git -C "${groot_dir}" submodule update --init --recursive
    else
      log "Updating Groot submodules."
      run_ws_cmd git -C "${groot_dir}" submodule update --init --recursive
      if [[ -n "${commit}" && "${just_cloned}" == false ]]; then
        log "Groot commit override requested (${commit}) but repository already exists; leaving checkout unchanged."
      fi
    fi
  else
    log "Warning: Groot repository not initialized correctly at ${groot_dir}."
  fi

  if [[ ! -d "${groot_dir}" ]]; then
    log "Error: Groot directory ${groot_dir} is missing."
    return 1
  fi

  log "Configuring Groot build directory."
  run_ws_cmd cmake -S "${groot_dir}" -B "${groot_dir}/build"

  log "Building Groot."
  run_ws_cmd cmake --build "${groot_dir}/build" --parallel

  if [[ -d "${groot_dir}/.git" ]]; then
    local groot_head=""
    groot_head="$(git -C "${groot_dir}" rev-parse HEAD 2>/dev/null || true)"
    if [[ -n "${groot_head}" ]]; then
      printf '%s\n' "${groot_head}" | run_ws_cmd tee "${WORKSPACE_ROOT}/.groot_commit" >/dev/null
    fi
  fi
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

if ! ensure_groot; then
  log "Groot setup failed."
  exit 1
fi

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
