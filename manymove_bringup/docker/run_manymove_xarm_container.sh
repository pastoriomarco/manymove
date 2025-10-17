#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run_manymove_xarm_container.sh <humble|jazzy> [--pull-latest] [--build-only] [--] [additional docker run args...]

Builds (if needed) and launches the ManyMove + xArm ROS 2 development container.

Options:
  --pull-latest   Fetch the latest commits for ManyMove and xarm_ros2 before rebuilding; skips rebuilds if already current.
  --build-only    Build or refresh the image but do not start a container.

Arguments after "--" are forwarded to "docker run".
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
BASE_RUN_SCRIPT="${DOCKER_DIR}/run_manymove_container.sh"
DOCKERFILE="${DOCKER_DIR}/Dockerfile.manymove_xarm"

if [[ ! -f "${DOCKERFILE}" ]]; then
  echo "xArm Dockerfile not found at ${DOCKERFILE}" >&2
  exit 1
fi

if [[ ! -x "${BASE_RUN_SCRIPT}" ]]; then
  echo "Helper script '${BASE_RUN_SCRIPT}' is missing or not executable." >&2
  exit 1
fi

BASE_ARGS=("${DISTRO}")
if [[ "${PULL_LATEST}" == true ]]; then
  BASE_ARGS+=("--pull-latest")
fi
BASE_ARGS+=("--build-only")

"${BASE_RUN_SCRIPT}" "${BASE_ARGS[@]}"

BASE_IMAGE_TAG="manymove:${DISTRO}"
if ! docker image inspect "${BASE_IMAGE_TAG}" >/dev/null 2>&1; then
  echo "Expected base image '${BASE_IMAGE_TAG}' was not built successfully." >&2
  exit 1
fi

BASE_IMAGE_ID="$(docker image inspect "${BASE_IMAGE_TAG}" --format '{{ .Id }}')"

XARM_REPO_DEFAULT="https://github.com/pastoriomarco/xarm_ros2.git"
XARM_BRANCH_DEFAULT="${DISTRO}"
TARGET_XARM_COMMIT=""
LABEL_XARM_COMMIT=""

IMAGE_TAG="manymove-xarm:${DISTRO}"
LABEL_KEY="manymove.xarm.context.sha"
COMMIT_LABEL_KEY="manymove.xarm.commit"
BASE_LABEL_KEY="manymove.base.id"

IMAGE_PRESENT=false
EXISTING_HASH=""
EXISTING_COMMIT=""
EXISTING_BASE_ID=""

if docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
  IMAGE_PRESENT=true
  EXISTING_HASH="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${LABEL_KEY}\" }}" 2>/dev/null || true)"
  EXISTING_COMMIT="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${COMMIT_LABEL_KEY}\" }}" 2>/dev/null || true)"
  EXISTING_BASE_ID="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${BASE_LABEL_KEY}\" }}" 2>/dev/null || true)"
  if [[ -n "${EXISTING_COMMIT}" ]]; then
    LABEL_XARM_COMMIT="${EXISTING_COMMIT}"
  fi
fi

REMOTE_XARM_COMMIT=""
if [[ "${PULL_LATEST}" == true || -z "${LABEL_XARM_COMMIT}" ]]; then
  REMOTE_REF="$(git ls-remote --heads "${XARM_REPO_DEFAULT}" "${XARM_BRANCH_DEFAULT}" 2>/dev/null || true)"
  if [[ -z "${REMOTE_REF}" ]]; then
    if [[ "${PULL_LATEST}" == true ]]; then
      echo "Failed to query latest commit for ${XARM_BRANCH_DEFAULT} from ${XARM_REPO_DEFAULT}" >&2
      exit 1
    else
      echo "Warning: unable to determine latest xarm_ros2 commit; labeling build as 'unknown'." >&2
    fi
  else
    REMOTE_XARM_COMMIT="${REMOTE_REF%%[[:space:]]*}"
    if [[ "${PULL_LATEST}" == true ]]; then
      TARGET_XARM_COMMIT="${REMOTE_XARM_COMMIT}"
      echo "Latest xarm_ros2 commit on '${XARM_BRANCH_DEFAULT}': ${TARGET_XARM_COMMIT}"
    fi
    if [[ -z "${LABEL_XARM_COMMIT}" || "${PULL_LATEST}" == true ]]; then
      LABEL_XARM_COMMIT="${REMOTE_XARM_COMMIT}"
    fi
  fi
fi

if [[ -z "${LABEL_XARM_COMMIT}" ]]; then
  LABEL_XARM_COMMIT="unknown"
fi

CONTEXT_HASH="$(
  {
    printf '%s\n' "${DISTRO}";
    printf '%s\n' "${BASE_IMAGE_ID}";
    if [[ -n "${TARGET_XARM_COMMIT}" ]]; then
      printf '%s\n' "${TARGET_XARM_COMMIT}";
    fi
    cat "${DOCKERFILE}";
  } | sha256sum | awk '{print $1}'
)"

NEEDS_BUILD=false

if [[ "${IMAGE_PRESENT}" == false ]]; then
  NEEDS_BUILD=true
elif [[ "${EXISTING_HASH}" != "${CONTEXT_HASH}" ]]; then
  NEEDS_BUILD=true
elif [[ "${EXISTING_BASE_ID}" != "${BASE_IMAGE_ID}" ]]; then
  NEEDS_BUILD=true
elif [[ -n "${TARGET_XARM_COMMIT}" && "${EXISTING_COMMIT}" != "${TARGET_XARM_COMMIT}" ]]; then
  NEEDS_BUILD=true
fi

if [[ "${NEEDS_BUILD}" == false ]]; then
  if [[ "${PULL_LATEST}" == true && -n "${TARGET_XARM_COMMIT}" ]]; then
    echo "xarm_ros2 sources already up to date (commit ${TARGET_XARM_COMMIT}); skipping rebuild."
  fi
else
  if [[ "${IMAGE_PRESENT}" == true ]]; then
    echo "Rebuilding image '${IMAGE_TAG}' (context or xarm_ros2 sources changed)."
  else
    echo "Building image '${IMAGE_TAG}' from ${DOCKERFILE}"
  fi
  BUILD_CMD=(
    docker build
    "--build-arg" "ROS_DISTRO=${DISTRO}"
    "--build-arg" "XARM_BRANCH=${XARM_BRANCH_DEFAULT}"
    "--label" "${LABEL_KEY}=${CONTEXT_HASH}"
    "--label" "${BASE_LABEL_KEY}=${BASE_IMAGE_ID}"
    "--label" "${COMMIT_LABEL_KEY}=${LABEL_XARM_COMMIT}"
  )
  if [[ "${LABEL_XARM_COMMIT}" != "unknown" ]]; then
    BUILD_CMD+=("--build-arg" "XARM_COMMIT=${LABEL_XARM_COMMIT}")
  fi
  BUILD_CMD+=(
    "-f" "${DOCKERFILE}"
    "-t" "${IMAGE_TAG}"
    "${DOCKER_DIR}"
  )
  "${BUILD_CMD[@]}"
fi

if [[ "${BUILD_ONLY}" == true ]]; then
  exit 0
fi

RUN_ARGS=(
  "--rm"
  "-it"
  "--network" "host"
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

if command -v nvidia-smi >/dev/null 2>&1; then
  RUN_ARGS+=("--gpus" "all")
fi

echo "Launching container '${IMAGE_TAG}'"
docker run "${RUN_ARGS[@]}" "${EXTRA_DOCKER_ARGS[@]}" "${IMAGE_TAG}" bash
