#!/usr/bin/env bash
#
# on container start (after workspace-entrypoint.sh sets up the ROS env),
# this script will clone & build *once*, leaving install/, build/, log/
# under your host-mounted workspace.

set -eo pipefail

# The host workspace is mounted here:
WORKDIR="${ISAAC_ROS_WS}"
SRC="${WORKDIR}/src"
INSTALL_SETUP="${WORKDIR}/install/setup.bash"

echo "--- [build-manymove] starting ---"

# ensure src dir exists
mkdir -p "${SRC}"

cd "${WORKDIR}"

# if we already have an overlay, do nothing
if [ -f "${INSTALL_SETUP}" ]; then
  echo "[build-manymove] install/ already present, skipping build."
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source ${INSTALL_SETUP}
  return 0
fi

echo "[build-manymove] applying mesh/config tweaks…"
mkdir -p src/xarm_ros2/xarm_description/meshes/other
cp -r src/manymove/manymove_object_manager/meshes/custom_end_tools/* \
      src/xarm_ros2/xarm_description/meshes/other/
cp src/manymove/manymove_planner/config/xarm_user_params.yaml \
   src/xarm_ros2/xarm_api/config/ || true

echo "[build-manymove] installing rosdep dependencies…"
# always OK to re-update sources
rosdep update  || true
rosdep install --from-paths src --ignore-src -y

echo "[build-manymove] building via colcon…"
# source the underlying ROS distro
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${INSTALL_SETUP}
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[build-manymove] build complete — install/ now available"
