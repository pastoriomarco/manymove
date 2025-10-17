#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run_manymove_container.sh <humble|jazzy> [--] [additional docker run args...]

Builds (if needed) and launches the ManyMove development container for the
requested ROS 2 distribution. Any arguments after "--" are passed directly to
"docker run".
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

if [[ $# -gt 0 && "$1" == "--" ]]; then
  shift
else
  # no explicit "--", fall through with existing args
  :
fi

EXTRA_DOCKER_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="${SCRIPT_DIR}"
DOCKERFILE="${DOCKER_DIR}/Dockerfile.${DISTRO}"

if [[ ! -f "${DOCKERFILE}" ]]; then
  echo "Dockerfile for distro '${DISTRO}' not found at ${DOCKERFILE}" >&2
  exit 1
fi

IMAGE_TAG="manymove:${DISTRO}"
CONTAINER_USER="${USER:-manymove}"

CONTEXT_HASH="$(
  cat \
    "${DOCKERFILE}" \
    "${DOCKER_DIR}/scripts/setup_workspace.sh" \
    "${DOCKER_DIR}/scripts/auto_source_overlay.sh" \
    | sha256sum | awk '{print $1}'
)"
IMAGE_PRESENT=false
EXISTING_HASH=""
LABEL_KEY="manymove.context.sha"

if docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
  IMAGE_PRESENT=true
  EXISTING_HASH="$(docker image inspect "${IMAGE_TAG}" --format "{{ index .Config.Labels \"${LABEL_KEY}\" }}" 2>/dev/null || true)"
fi

if [[ "${IMAGE_PRESENT}" == false || "${EXISTING_HASH}" != "${CONTEXT_HASH}" ]]; then
  if [[ "${IMAGE_PRESENT}" == true ]]; then
    echo "Rebuilding image '${IMAGE_TAG}' (container build context changed)."
  else
    echo "Building image '${IMAGE_TAG}' from ${DOCKERFILE}"
  fi
  docker build \
    --build-arg ROS_DISTRO="${DISTRO}" \
    --build-arg USERNAME="${CONTAINER_USER}" \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    --label "${LABEL_KEY}=${CONTEXT_HASH}" \
    -f "${DOCKERFILE}" \
    -t "${IMAGE_TAG}" \
    "${DOCKER_DIR}"
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
