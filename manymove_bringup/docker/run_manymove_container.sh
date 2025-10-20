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

MANYMOVE_BRANCH_SOURCE="fallback"

resolve_manymove_branch() {
  if [[ -n "${MANYMOVE_BRANCH:-}" ]]; then
    MANYMOVE_BRANCH_SOURCE="env"
    echo "${MANYMOVE_BRANCH}"
    return
  fi

  local repo_root=""
  repo_root="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
  if [[ -n "${repo_root}" ]]; then
    local current_branch=""
    current_branch="$(git -C "${repo_root}" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
    if [[ -n "${current_branch}" && "${current_branch}" != "HEAD" ]]; then
      MANYMOVE_BRANCH_SOURCE="repo"
      echo "${current_branch}"
      return
    fi
  fi

  echo "main"
}

IMAGE_TAG="manymove:${DISTRO}"
CONTAINER_USER="${USER:-manymove}"

MANYMOVE_REPO_DEFAULT="https://github.com/pastoriomarco/manymove.git"
MANYMOVE_BRANCH_DEFAULT="$(resolve_manymove_branch)"
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
    echo "Using ManyMove branch '${MANYMOVE_BRANCH_DEFAULT}' (fallback to main)."
    ;;
esac

IMAGE_PRESENT=false
EXISTING_HASH=""
LABEL_KEY="manymove.context.sha"
COMMIT_LABEL_KEY="manymove.commit"
EXISTING_COMMIT=""

if docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
  IMAGE_PRESENT=true
  EXISTING_HASH="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${LABEL_KEY}\" }}" 2>/dev/null || true)"
  EXISTING_COMMIT="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${COMMIT_LABEL_KEY}\" }}" 2>/dev/null || true)"
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

NEEDS_BUILD=${FORCE_REBUILD}

if [[ "${IMAGE_PRESENT}" == false ]]; then
  NEEDS_BUILD=true
elif [[ "${EXISTING_HASH}" != "${CONTEXT_HASH}" ]]; then
  NEEDS_BUILD=true
elif [[ -n "${TARGET_COMMIT}" && "${EXISTING_COMMIT}" != "${TARGET_COMMIT}" ]]; then
  NEEDS_BUILD=true
fi

if [[ "${NEEDS_BUILD}" == false ]]; then
  if [[ "${PULL_LATEST}" == true && -n "${TARGET_COMMIT}" ]]; then
    echo "ManyMove sources already up to date (commit ${TARGET_COMMIT}); skipping rebuild."
  fi
else
  if [[ "${IMAGE_PRESENT}" == true ]]; then
    echo "Rebuilding image '${IMAGE_TAG}' (context or ManyMove sources changed)."
  else
    echo "Building image '${IMAGE_TAG}' from ${DOCKERFILE}"
  fi
  BUILD_CMD=(
    docker build
    "--build-arg" "ROS_DISTRO=${DISTRO}"
    "--build-arg" "USERNAME=${CONTAINER_USER}"
    "--build-arg" "USER_UID=$(id -u)"
    "--build-arg" "USER_GID=$(id -g)"
    "--label" "${LABEL_KEY}=${CONTEXT_HASH}"
    "--label" "${COMMIT_LABEL_KEY}=${LABEL_COMMIT}"
  )
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

if [[ "${BUILD_ONLY}" == true ]]; then
  exit 0
fi

echo "Launching container '${IMAGE_TAG}'"
docker run "${RUN_ARGS[@]}" "${EXTRA_DOCKER_ARGS[@]}" "${IMAGE_TAG}" bash
