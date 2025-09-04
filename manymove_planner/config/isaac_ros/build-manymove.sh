#!/usr/bin/env bash

set -eo pipefail

# The host workspace is mounted here:
WORKDIR="${ISAAC_ROS_WS}"
SRC="${WORKDIR}/src"
INSTALL_SETUP="${WORKDIR}/install/setup.bash"

echo "--- [build-manymove] starting ---"

# Ensure src dir exists
mkdir -p "${SRC}"

cd "${WORKDIR}"

# If we already have an overlay, do nothing
if [ -f "${INSTALL_SETUP}" ]; then
  echo "[build-manymove] install/ already present, skipping build."
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source ${INSTALL_SETUP}
  # This script is sourced by the container entrypoint; use return, not exit
  return 0
fi

# echo "[build-manymove] applying mesh/config tweaks…"
# mkdir -p src/xarm_ros2/xarm_description/meshes/other
# cp -r src/manymove/manymove_object_manager/meshes/custom_end_tools/* \
#       src/xarm_ros2/xarm_description/meshes/other/
# cp src/manymove/manymove_planner/config/xarm_user_params.yaml \
#    src/xarm_ros2/xarm_api/config/ || true

echo "[build-manymove] installing rosdep dependencies…"

# Resolve dependencies across the whole workspace once (safer for cross-tree deps like xarm_msgs)
apt update
rosdep update || true
rosdep install --from-paths "${SRC}" --ignore-src -y -r || true

echo "[build-manymove] building via colcon as normal user…"

# Source the distro first; then drop privileges *only* for colcon.
source /opt/ros/${ROS_DISTRO}/setup.bash

# if [[ "$(id -u)" -eq 0 && -n "${USERNAME}" ]]; then
#   gosu "${USERNAME}" bash -c "
#     set -eo pipefail
#     source /opt/ros/${ROS_DISTRO}/setup.bash
#     colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
#   "
if [[ "$(id -u)" -eq 0 && -n "${USERNAME}" ]]; then
  gosu "${USERNAME}" bash -c "
    set -eo pipefail
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${ISAAC_ROS_WS}
    colcon build \
      --symlink-install \
      --packages-up-to \
        manymove_cpp_trees \
        manymove_py_trees \
        manymove_hmi \
        manymove_bringup \
        isaac_ros_cumotion \
        isaac_ros_cumotion_examples \
        isaac_ros_yolov8 \
        isaac_ros_foundationpose \
        isaac_ros_custom_bringup \
        xarm_msgs \
        xarm_moveit_config
  "
else
  # Fallback: we are already the normal user (or USERNAME unset)
  # colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  cd ${ISAAC_ROS_WS}
  colcon build \
    --symlink-install \
    --packages-up-to \
      manymove_cpp_trees \
      manymove_py_trees \
      manymove_hmi \
      manymove_bringup \
      isaac_ros_cumotion \
      isaac_ros_cumotion_examples \
      isaac_ros_yolov8 \
      isaac_ros_foundationpose \
      isaac_ros_custom_bringup \
      xarm_msgs \
      xarm_moveit_config
fi

source ${INSTALL_SETUP}

echo "[build-manymove] build complete — install/ now available"
