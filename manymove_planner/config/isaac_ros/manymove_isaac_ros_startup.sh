#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROS_WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"

# Ensure base directories
mkdir -p "${ISAAC_ROS_WS}/src" "${ISAAC_ROS_WS}/isaac_ros_assets"

cd "${ISAAC_ROS_WS}/src"

# Helper to clone or update a repo idempotently
ensure_repo() {
  # Usage: ensure_repo <dest_dir> <git_url> [extra git clone args...]
  local dest="$1"; shift
  local url="$1"; shift || true
  if [ -d "$dest/.git" ]; then
    echo "[startup] Updating repo: $dest"
    git -C "$dest" pull --ff-only || true
  elif [ ! -e "$dest" ]; then
    echo "[startup] Cloning $url into $dest"
    git clone "$@" "$url" "$dest"
  else
    echo "[startup] Skipping $dest (exists but not a git repo)"
  fi
}

# Repos (release-3.2+ where applicable)
ensure_repo isaac_ros_common https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git -b release-3.2
ensure_repo isaac_ros_cumotion https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git --recurse-submodules -b release-3.2
ensure_repo isaac_ros_object_detection https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git -b release-3.2
ensure_repo isaac_ros_pose_estimation https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git -b release-3.2
ensure_repo isaac_ros_custom_bringup https://github.com/pastoriomarco/isaac_ros_custom_bringup.git
ensure_repo xarm_ros2 https://github.com/pastoriomarco/xarm_ros2.git --recursive -b humble_no_gazebo
ensure_repo manymove https://github.com/pastoriomarco/manymove.git -b main
ensure_repo isaac_sim_custom_examples https://github.com/pastoriomarco/isaac_sim_custom_examples.git
ensure_repo third_party https://github.com/pastoriomarco/moveit_calibration.git

# Sync submodules for xarm_ros2 if present
if [ -d xarm_ros2/.git ]; then
  (cd xarm_ros2 && git submodule sync && git submodule update --init --remote)
fi

# Copy Dockerfiles and scripts into isaac_ros_common
cp "${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/isaac_ros/Dockerfile.manymove" \
   "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/"
cp "${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/isaac_ros/build-manymove.sh" \
   "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts"
cp "${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/isaac_ros/.isaac_ros_common-config" \
   "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"
cp "${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/isaac_ros/Dockerfile.aarch64" \
   "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/"

# ------------------------------------------------------------
# Asset downloads (FoundationPose + YOLOv8) for Isaac ROS 3.2
# ------------------------------------------------------------
command -v curl >/dev/null 2>&1 || { echo "[startup] curl not found; skipping assets download"; exit 0; }
command -v jq >/dev/null 2>&1 || { echo "[startup] jq not found; skipping assets download"; exit 0; }

NGC_ORG="nvidia"
NGC_TEAM="isaac"
MAJOR_VERSION=3
MINOR_VERSION=2
ASSETS_DIR="${ISAAC_ROS_WS}/isaac_ros_assets"

download_quickstart_tar() {
  # Args: <resource> <filename> <dest_dir>
  local resource="$1"; local filename="$2"; local dest="$3"
  local version_req_url="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$resource&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
  local available_versions
  available_versions=$(curl -fsSL -H "Accept: application/json" "$version_req_url") || true
  local latest_version_id
  latest_version_id=$(echo "$available_versions" | jq -r "
      .recipeVersions[]
      | .versionId as \$v
      | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
      | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
      | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
      | \$v
      " | sort -V | tail -n 1)

  if [ -z "${latest_version_id:-}" ] || [ "${latest_version_id}" = "null" ]; then
    echo "[startup] No NGC version matched for $resource (Isaac ROS $MAJOR_VERSION.$MINOR_VERSION)"
    echo "[startup] Available versions:" && echo "$available_versions" | jq -r '.recipeVersions[].versionId' || true
    return 0
  fi

  mkdir -p "$dest"

  local file_req_url="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$resource/versions/$latest_version_id/files/$filename"
  echo "[startup] Downloading $resource:$latest_version_id ($filename)"
  if ! curl -fSLo "$filename" --request GET "$file_req_url"; then
    echo "[startup] Failed to download $filename from $resource:$latest_version_id"
    return 1
  fi

  case "$filename" in
    *.tar.gz|*.tgz)
      tar -xzf "$filename" -C "$dest" ;;
    *.zip)
      if command -v unzip >/dev/null 2>&1; then
        unzip -oq "$filename" -d "$dest"
      else
        echo "[startup] unzip not found; using python to extract zip"
        python3 - <<'PY'
import sys, zipfile
zf = zipfile.ZipFile(sys.argv[1])
zf.extractall(sys.argv[2])
zf.close()
PY
        "$filename" "$dest"
      fi
      ;;
    *) echo "[startup] Unknown archive type: $filename" ;;
  esac
  rm -f "$filename"
}

# FoundationPose assets (skip if already present)
if [ ! -d "${ASSETS_DIR}/isaac_ros_foundationpose" ]; then
  download_quickstart_tar "isaac_ros_foundationpose_assets" "quickstart.tar.gz" "$ASSETS_DIR" || true
else
  echo "[startup] Skipping FoundationPose assets (already present)"
fi

# YOLOv8 assets (provides quickstart interface specs). Skip if present.
if [ ! -d "${ASSETS_DIR}/isaac_ros_yolov8" ]; then
  # Try common filenames; first tar.gz, then zip
  if ! download_quickstart_tar "isaac_ros_yolov8_assets" "quickstart.tar.gz" "$ASSETS_DIR"; then
    download_quickstart_tar "isaac_ros_yolov8_assets" "quickstart.zip" "$ASSETS_DIR" || true
  fi
else
  echo "[startup] Skipping YOLOv8 assets (already present)"
fi

echo "[startup] Startup script completed"
