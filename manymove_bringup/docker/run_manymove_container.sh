#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run_manymove_container.sh <humble|jazzy> [--pull-latest] [--force-rebuild] [--build-only] [--] [additional docker run args...]

Builds (if needed) and launches the ManyMove development container for the
requested ROS 2 distribution.

Options:
  --pull-latest   Fetch the latest ManyMove commit before rebuilding; skip the rebuild if already current.
  --build-only      Build or refresh the image but do not start a container.
  --force-rebuild   Rebuild the image even if the cached context looks current.

Arguments after "--" are passed straight to "docker run".
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

DISTRO="$1"
shift

case "${DISTRO}" in
  humble|jazzy)
    ;;
  *)
    echo "Unsupported ROS 2 distribution '${DISTRO}'. Use 'humble' or 'jazzy'." >&2
    exit 1
    ;;
esac

PULL_LATEST=false
BUILD_ONLY=false
FORCE_REBUILD=false
EXTRA_DOCKER_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pull-latest)
      PULL_LATEST=true
      shift
      ;;
    --build-only)
      BUILD_ONLY=true
      shift
      ;;
    --force-rebuild)
      FORCE_REBUILD=true
      shift
      ;;
    --)
      shift
      EXTRA_DOCKER_ARGS+=("$@")
      break
      ;;
    *)
      EXTRA_DOCKER_ARGS+=("$1")
      shift
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="${SCRIPT_DIR}"
DOCKERFILE="${DOCKER_DIR}/Dockerfile.manymove"

if [[ ! -f "${DOCKERFILE}" ]]; then
  echo "Unified Dockerfile not found at ${DOCKERFILE}" >&2
  exit 1
fi

resolve_manymove_branch() {
  if [[ -n "${MANYMOVE_BRANCH:-}" ]]; then
    printf 'env\n%s\n' "${MANYMOVE_BRANCH}"
    return
  fi

  local repo_root=""
  repo_root="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
  if [[ -n "${repo_root}" ]]; then
    local current_branch=""
    current_branch="$(git -C "${repo_root}" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
    if [[ -n "${current_branch}" && "${current_branch}" != "HEAD" ]]; then
      printf 'repo\n%s\n' "${current_branch}"
      return
    fi
  fi

  printf 'fallback\nmain\n'
}

mapfile -t MANYMOVE_BRANCH_INFO < <(resolve_manymove_branch)
MANYMOVE_BRANCH_SOURCE="${MANYMOVE_BRANCH_INFO[0]:-fallback}"
MANYMOVE_BRANCH_DEFAULT="${MANYMOVE_BRANCH_INFO[1]:-main}"

IMAGE_TAG="manymove:${DISTRO}"
CONTAINER_USER="${USER:-manymove}"
CONTAINER_UID="$(id -u)"
CONTAINER_GID="$(id -g)"

MANYMOVE_REPO_DEFAULT="https://github.com/pastoriomarco/manymove.git"
TARGET_COMMIT=""
LABEL_COMMIT=""

case "${MANYMOVE_BRANCH_SOURCE}" in
  env)
    echo "Using ManyMove branch '${MANYMOVE_BRANCH_DEFAULT}' from MANYMOVE_BRANCH env var."
    ;;
  repo)
    echo "Using ManyMove branch '${MANYMOVE_BRANCH_DEFAULT}' detected from local git checkout."
    ;;
  *)
    echo "Using ManyMove branch '${MANYMOVE_BRANCH_DEFAULT}' (fallback to default 'main')."
    ;;
esac

IMAGE_PRESENT=false
LABEL_KEY="manymove.context.sha"
COMMIT_LABEL_KEY="manymove.commit"
USER_LABEL_KEY="manymove.user.name"
USER_UID_LABEL_KEY="manymove.user.uid"
USER_GID_LABEL_KEY="manymove.user.gid"
EXISTING_COMMIT=""
EXISTING_USER_NAME=""
EXISTING_USER_UID=""
EXISTING_USER_GID=""

if docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
  IMAGE_PRESENT=true
  EXISTING_COMMIT="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${COMMIT_LABEL_KEY}\" }}" 2>/dev/null || true)"
  EXISTING_USER_NAME="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${USER_LABEL_KEY}\" }}" 2>/dev/null || true)"
  EXISTING_USER_UID="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${USER_UID_LABEL_KEY}\" }}" 2>/dev/null || true)"
  EXISTING_USER_GID="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${USER_GID_LABEL_KEY}\" }}" 2>/dev/null || true)"
  if [[ -n "${EXISTING_COMMIT}" ]]; then
    LABEL_COMMIT="${EXISTING_COMMIT}"
  fi
fi

REMOTE_COMMIT=""
if [[ "${PULL_LATEST}" == true || -z "${LABEL_COMMIT}" ]]; then
  REMOTE_REF="$(git ls-remote --heads "${MANYMOVE_REPO_DEFAULT}" "${MANYMOVE_BRANCH_DEFAULT}" 2>/dev/null || true)"
  if [[ -z "${REMOTE_REF}" ]]; then
    if [[ "${PULL_LATEST}" == true ]]; then
      echo "Failed to query latest commit for ${MANYMOVE_BRANCH_DEFAULT} from ${MANYMOVE_REPO_DEFAULT}" >&2
      exit 1
    else
      echo "Warning: unable to determine latest ManyMove commit; labeling build as 'unknown'." >&2
    fi
  else
    REMOTE_COMMIT="${REMOTE_REF%%[[:space:]]*}"
    if [[ "${PULL_LATEST}" == true ]]; then
      TARGET_COMMIT="${REMOTE_COMMIT}"
      echo "Latest ManyMove commit on '${MANYMOVE_BRANCH_DEFAULT}': ${TARGET_COMMIT}"
    fi
    if [[ -z "${LABEL_COMMIT}" || "${PULL_LATEST}" == true ]]; then
      LABEL_COMMIT="${REMOTE_COMMIT}"
    fi
  fi
fi

if [[ -z "${LABEL_COMMIT}" ]]; then
  LABEL_COMMIT="unknown"
fi

CONTEXT_HASH="$(
  {
    printf '%s\n' "${DISTRO}";
    if [[ -n "${TARGET_COMMIT}" ]]; then
      printf '%s\n' "${TARGET_COMMIT}";
    fi
    cat \
      "${DOCKERFILE}" \
      "${DOCKER_DIR}/scripts/setup_workspace.sh" \
      "${DOCKER_DIR}/scripts/auto_source_overlay.sh";
  } | sha256sum | awk '{print $1}'
)"

NEEDS_BUILD=false

if [[ "${FORCE_REBUILD}" == true ]]; then
  NEEDS_BUILD=true
elif [[ "${IMAGE_PRESENT}" == false ]]; then
  NEEDS_BUILD=true
elif [[ "${PULL_LATEST}" == true ]]; then
  if [[ -n "${TARGET_COMMIT}" ]]; then
    if [[ "${EXISTING_COMMIT}" != "${TARGET_COMMIT}" ]]; then
      echo "Latest ManyMove commit differs (have ${EXISTING_COMMIT:-unknown}, want ${TARGET_COMMIT}); rebuilding."
    else
      echo "ManyMove sources already up to date (commit ${TARGET_COMMIT}); rebuilding to pull latest base layers."
    fi
  else
    echo "Unable to resolve latest ManyMove commit; rebuilding to ensure freshness."
  fi
  NEEDS_BUILD=true
fi

if [[ "${IMAGE_PRESENT}" == true ]]; then
  if [[ -z "${EXISTING_USER_NAME}" || -z "${EXISTING_USER_UID}" || -z "${EXISTING_USER_GID}" ]]; then
    echo "Rebuilding to record container user metadata needed for passwordless sudo."
    NEEDS_BUILD=true
  elif [[ "${EXISTING_USER_NAME}" != "${CONTAINER_USER}" ]]; then
    echo "Rebuilding because baked-in container user '${EXISTING_USER_NAME}' does not match current user '${CONTAINER_USER}'."
    NEEDS_BUILD=true
  elif [[ "${EXISTING_USER_UID}" != "${CONTAINER_UID}" ]]; then
    echo "Rebuilding because baked-in UID '${EXISTING_USER_UID}' does not match current UID '${CONTAINER_UID}'."
    NEEDS_BUILD=true
  elif [[ "${EXISTING_USER_GID}" != "${CONTAINER_GID}" ]]; then
    echo "Rebuilding because baked-in GID '${EXISTING_USER_GID}' does not match current GID '${CONTAINER_GID}'."
    NEEDS_BUILD=true
  fi
fi

if [[ "${NEEDS_BUILD}" == true ]]; then
  if [[ "${IMAGE_PRESENT}" == true ]]; then
    echo "Rebuilding image '${IMAGE_TAG}'."
  else
    echo "Building image '${IMAGE_TAG}' from ${DOCKERFILE}"
  fi
  BUILD_CMD=(
    docker build
    "--build-arg" "ROS_DISTRO=${DISTRO}"
    "--build-arg" "USERNAME=${CONTAINER_USER}"
    "--build-arg" "USER_UID=${CONTAINER_UID}"
    "--build-arg" "USER_GID=${CONTAINER_GID}"
    "--label" "${LABEL_KEY}=${CONTEXT_HASH}"
    "--label" "${COMMIT_LABEL_KEY}=${LABEL_COMMIT}"
    "--label" "${USER_LABEL_KEY}=${CONTAINER_USER}"
    "--label" "${USER_UID_LABEL_KEY}=${CONTAINER_UID}"
    "--label" "${USER_GID_LABEL_KEY}=${CONTAINER_GID}"
  )
  if [[ -n "${MANYMOVE_COLCON_WORKERS:-}" ]]; then
    BUILD_CMD+=("--build-arg" "MANYMOVE_COLCON_WORKERS=${MANYMOVE_COLCON_WORKERS}")
  fi
  if [[ "${PULL_LATEST}" == true ]]; then
    BUILD_CMD+=("--pull")
  fi
  if [[ "${FORCE_REBUILD}" == true ]]; then
    BUILD_CMD+=("--no-cache")
  fi
  if [[ "${LABEL_COMMIT}" != "unknown" ]]; then
    BUILD_CMD+=("--build-arg" "MANYMOVE_COMMIT=${LABEL_COMMIT}")
  fi
  BUILD_CMD+=("--build-arg" "MANYMOVE_BRANCH=${MANYMOVE_BRANCH_DEFAULT}")
  BUILD_CMD+=(
    "-f" "${DOCKERFILE}"
    "-t" "${IMAGE_TAG}"
    "${DOCKER_DIR}"
  )
  "${BUILD_CMD[@]}"
fi

RUN_ARGS=(
  "--rm"
  "-it"
  "--network" "host"
  "--ipc" "host"  # share /dev/shm so Fast DDS shared memory works across shells/containers
)

if [[ -n "${DISPLAY:-}" ]]; then
  RUN_ARGS+=("-e" "DISPLAY=${DISPLAY}")
  if [[ -d /tmp/.X11-unix ]]; then
    RUN_ARGS+=("-v" "/tmp/.X11-unix:/tmp/.X11-unix:rw")
  fi
fi

if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
  RUN_ARGS+=("-e" "XAUTHORITY=${XAUTHORITY}")
  RUN_ARGS+=("-v" "${XAUTHORITY}:${XAUTHORITY}:rw")
fi

# Optional GPU disable flag
if [[ -z "${MANYMOVE_NO_GPU:-}" ]] && command -v nvidia-smi >/dev/null 2>&1; then
  RUN_ARGS+=("--gpus" "all")
else
  echo "GPU support disabled (MANYMOVE_NO_GPU set or nvidia-smi missing)"
fi

if [[ "${BUILD_ONLY}" == true ]]; then
  exit 0
fi

echo "Launching container '${IMAGE_TAG}'"
docker run "${RUN_ARGS[@]}" "${EXTRA_DOCKER_ARGS[@]}" "${IMAGE_TAG}" bash
