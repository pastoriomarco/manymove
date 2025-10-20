## Step 1: Developer Environment Setup

First, set up the compute and developer environment by following Nvidia’s instructions:

* [Compute Setup](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html)
* [Developer Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)

It’ very important that you completely follow the above setup steps for the platform you are going to use. Don’t skip the steps for [Jetson Platform](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html#jetson-platforms) if that’s what you are using, including [VPI](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/jetson_vpi.html).

## Step 2: Download and run the setup script

Download [manymove_isaac_ros_startup.sh](https://github.com/pastoriomarco/manymove/blob/main/manymove_planner/config/isaac_ros/manymove_isaac_ros_startup.sh), make it executable and run it. It will download all the packages tested with ManyMove and Isaac ROS.

> **Developers on `dev`**: export `MANYMOVE_BRANCH=dev` before running the script so it checks out the development branch instead of `main`.

## Step 3: Check isaac_ros_common-config

Since I’m working with realsense cameras, the current config includes the realsense package. If you are using them too, skip this step.

If you don’t need it, modify the following file with your favorite editor:

```
${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/.isaac_ros_common-config
```

Remove `.realsense` step from the config line from:

```
CONFIG_IMAGE_KEY=ros2_humble.realsense.manymove
```

To:

```
CONFIG_IMAGE_KEY=ros2_humble.manymove
```

## Step 4: Launch the Docker container

Launch the docker container using the `run_dev.sh` script:

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
   ./scripts/run_dev.sh
```

## Step 5: Run the examples:

### Franka Panda:

1. In **TERMINAL 1** start cumotion_planner_node and wait for it to warm up:

```
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/isaac_ros/franka.xrdf \
  -p urdf_path:=/opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf
```

2. In **TERMINAL 2**, first enter the the docker container:

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh
```

3. From inside the docker container in **TERMINAL 2**, source the workspace and run the example:

```
source ${ISAAC_ROS_WS}/install/setup.bash
ros2 launch manymove_bringup panda_cumotion_movegroup_fake_cpp_trees.launch.py
```

### Ufactory Lite6:

1. In **TERMINAL 1** start cumotion_planner_node and wait for it to warm up:

```
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/isaac_ros/lite6_gr.xrdf \
  -p urdf_path:=${ISAAC_ROS_WS}/src/manymove/manymove_planner/config/isaac_ros/lite6.urdf
```

2. In **TERMINAL 2**, first enter the the docker container:

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh
```

3. From inside the docker container in **TERMINAL 2**, source the workspace and run the example:

```
source ${ISAAC_ROS_WS}/install/setup.bash
ros2 launch manymove_bringup lite_cumotion_movegroup_fake_cpp_trees.launch.py
```

Let me know if you find some issues running this!
