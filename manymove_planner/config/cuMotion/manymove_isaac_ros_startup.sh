mkdir ${ISAAC_ROS_WS}/src

cd ${ISAAC_ROS_WS}/src && \
   git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common && \
   git clone --recurse-submodules -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git isaac_ros_cumotion && \
   git clone -b release-3.2 https://github.com/pastoriomarco/isaac_ros_object_detection.git isaac_ros_object_detection
   git clone https://github.com/pastoriomarco/xarm_ros2.git --recursive -b humble_no_gazebo && \
   git clone --branch=humble https://github.com/pastoriomarco/manymove.git
   git clone https://github.com/pastoriomarco/moveit_calibration.git third_party

cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
   git pull && \

cd ${ISAAC_ROS_WS}/src/isaac_ros_cumotion && \
    git pull && \

cd ${ISAAC_ROS_WS}/src/xarm_ros2 && \
   git pull && \
   git submodule sync && \
   git submodule update --init --remote

cd ${ISAAC_ROS_WS}/src/manymove && \
   git pull

cp ${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/cuMotion/Dockerfile.manymove ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/
cp ${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/cuMotion/build-manymove.sh ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts
cp ${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/cuMotion/.isaac_ros_common-config ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/
cp ${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/cuMotion/Dockerfile.aarch64 ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/
