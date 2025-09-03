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
  return 0
fi

# echo "[build-manymove] applying mesh/config tweaks…"
# mkdir -p src/xarm_ros2/xarm_description/meshes/other
# cp -r src/manymove/manymove_object_manager/meshes/custom_end_tools/* \
#       src/xarm_ros2/xarm_description/meshes/other/
# cp src/manymove/manymove_planner/config/xarm_user_params.yaml \
#    src/xarm_ros2/xarm_api/config/ || true

echo "[build-manymove] installing rosdep dependencies…"

rosdep update  || true
rosdep install --from-paths src --ignore-src -y

echo "[build-manymove] building via colcon as normal user…"

# Source the distro first; then drop privileges *only* for colcon.
source /opt/ros/${ROS_DISTRO}/setup.bash

if [[ "$(id -u)" -eq 0 && -n "${USERNAME}" ]]; then
  gosu "${USERNAME}" bash -c "
    set -eo pipefail
    source /opt/ros/${ROS_DISTRO}/setup.bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  "
else
  # Fallback: we are already the normal user (or USERNAME unset)
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

source ${INSTALL_SETUP}

echo "[build-manymove] build complete — install/ now available"
