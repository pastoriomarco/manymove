# ManyMove project

![ManyMove example](https://github.com/pastoriomarco/manymove/blob/dfc9c7f00e11d70d5e27fd2e0c13bfcf5de6de54/media/manymove_example.gif)

## DISCLAIMER

This repository is intended primarily for my own use as a roboticist, to deploy cobots in production using ROS2.
This is HIGHLY EXPERIMENTAL and comes with no warranty of stability or safety: USE AT YOUR OWN RISK.
IMPORTANT SAFETY NOTICE: this repository does not cover safety functions. Safety MUST be implemented using the internal safety system of the robot's controller and/or an appropriate safety controller, in compliance with your country’s regulations. Before deploying on a real robot, ensure that safety mechanisms are correctly configured.

## Description

The `manymove` project is meant for roboticists to ease the transition to ROS2 coming from the classic frameworks of major manifacturers.
It provides a simplified and generalized framework to build robotic manipulator control logic using ROS 2 and MoveIt 2. 
This series of packages was created around Ufactory Lite6 and UF850 cobots, but is as generalized as possible and also contains an example configuration of Franka Emika Panda, which is the default demo model for Moveit in ROS2 Humble.

## Prerequisites

- **Install ROS2 Humble, Moveit2 and xarm_ros2**:
  - You can follow the instructions on the Humble branch of [xarm_ros2 on github](https://github.com/xArm-Developer/xarm_ros2/tree/humble) to install all the required packages.

## Quick start

- **Define your `workspace dir`**:
  - You can use for example ~/dev_ws as in xarm_ros2 repo, or define an appropriate workspace.
  - From here on I'll refer to the installation directory of the workspace as `<workspace_dir>`
- **Clone `manymove` humble branch**:
  - From `<workspace_dir>/src`:
  ```bash
  git clone --branch=humble https://github.com/pastoriomarco/manymove.git
  ```
-  **Install the dependencies**
  - From `<workspace_dir>`:
  ```bash
  rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
  ```
- **Copy the auxiliary files to run all examples with the right configuration**:
  - Create the `other` folder in xarm_description/meshes. From `<workspace_dir>/src`:
  ```bash
  mkdir -p ./xarm_ros2/xarm_description/meshes/other
  ```
  - Copy pneumatic gripper's mesh. From `<workspace_dir>/src`:
  ```bash
  cp ./manymove/manymove_object_manager/meshes/gimatic_pq2516/* ./xarm_ros2/xarm_description/meshes/other/
  ```
  - Copy the user param file in xarm_api/config. From `<workspace_dir>/src`:
  ```bash
  cp ./manymove/manymove_planner/config/xarm_user_params.yaml ./xarm_ros2/xarm_api/config/
  ```
- **Build the packages**: 
  ```bash
  colcon build
  ```
- **Don't forget to source!**
  - From `<workspace_dir>`:
  ```bash
  source ./install/setup.bash
  ```

## Examples

### Launching the Action Server

- **Lite 6 manipulator example launcher**:
  ```bash
  ros2 launch manymove_planner lite_micpp_fake_cpp_trees.launch.py
  ```
  
- **Dual robot: Lite 6 and UF850 manipulators launcher example**:
  ```bash
  ros2 launch manymove_planner dual_micpp_fake_cpp_trees.launch.py
  ```

- **Panda Manipulator**:
- To use with a moveit2 humble demo with Panda robot launcher in another terminal, refer to [Moveit2 tutorials getting started](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html):
- In terminal 1:
  ```bash
  ros2 launch moveit2_tutorials demo.launch.py
  ```
- In terminal 2:
  ```bash
  ros2 launch manymove_planner panda_action_server_node.launch.py
  ```
  
## Credits

- **To create the control logic, `manymove` leverages some great behaviortree repos:**:
  - [BehaviorTree.cpp v3.8](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/v3.8), installed through dependencies
    -     ros-humble-behaviortree-cpp-v3)
    - [Groot](https://github.com/BehaviorTree/Groot), BehaviorTree.cpp v3.8's visualizer, to install from instructions in the github page.
  - [py_trees_ros](https://github.com/splintered-reality/py_trees_ros), installed through dependencies:
    -     ros-jazzy-py-trees \
          ros-jazzy-py-trees-ros-interfaces \
          ros-jazzy-py-trees-ros \
          ros-jazzy-py-trees-ros-tutorials \
          ros-jazzy-py-trees-ros-viewer
