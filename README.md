# ManyMove project — ROS 2 Humble & Jazzy

![ManyMove example](https://github.com/pastoriomarco/manymove/blob/dfc9c7f00e11d70d5e27fd2e0c13bfcf5de6de54/media/manymove_example.gif)

## DISCLAIMER

This software is released under the BSD-3-Clause license (see `LICENSE` for details).

**Safety notice:** The package does **not** implement functional safety. You **must** integrate appropriate safety measures compliant with local regulations before deploying on real hardware.

---

## Description

The `manymove` project is meant for roboticists to ease the transition to ROS 2 coming from the classic frameworks of major manufacturers.  
It provides a simplified, generalized framework for building robotic manipulator control logic using ROS 2 and MoveIt 2.  
This series of packages was created around Ufactory Lite6 and UF850 cobots, but it is structured so you can extend it to other robots. MoveIt demos for Franka Emika Panda are also included.

---

## Prerequisites

- Install **ROS 2 Humble** or **ROS 2 Jazzy** from the official docs:
  - [ROS 2 Humble](https://docs.ros.org/en/ros2_documentation/humble/Installation.html)
  - [ROS 2 Jazzy](https://docs.ros.org/en/ros2_documentation/jazzy/Installation.html)
- `rosdep`, Gazebo and MoveIt 2 are required. When using `rosdep install` (see below) they will be pulled automatically.
- Recommended: ensure the environment variable `ROS_DISTRO` is exported (`echo $ROS_DISTRO`) before following the quick start procedure.

---

## Quick start (Humble & Jazzy)

The commands below are distro-agnostic and rely on the `ROS_DISTRO` environment variable. When you source `/opt/ros/<distro>/setup.bash`, the variable is exported automatically.

**1. Define your workspace path**
```bash
export MANYMOVE_ROS_WS=~/workspaces/dev_ws
```

**2. Create the workspace folders (skip if already present)**
```bash
cd ~
mkdir -p ${MANYMOVE_ROS_WS}/src
```

**3. Source ROS 2**
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

**4. Clone ManyMove and xarm_ros2**
- Do not omit `--recursive`, otherwise the dependent submodules are not downloaded.
- `xarm_ros2` uses distro-specific branches, while `manymove` keeps all compatibility work on `main`.
```bash
cd ${MANYMOVE_ROS_WS}/src
git clone https://github.com/pastoriomarco/xarm_ros2.git --recursive -b $ROS_DISTRO
git clone https://github.com/pastoriomarco/manymove.git -b main
```

**5. Update repositories (optional refresh)**
```bash
cd ${MANYMOVE_ROS_WS}/src/xarm_ros2
git submodule update --init --recursive
git pull --recurse-submodules
cd ${MANYMOVE_ROS_WS}/src/manymove
git pull
```

**6. Install dependencies**
```bash
cd ${MANYMOVE_ROS_WS}
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

**7. (OPTIONAL) Install Groot** for visualising behavior trees
```bash
cd ${MANYMOVE_ROS_WS}/src/
git clone --recurse-submodules https://github.com/pastoriomarco/Groot.git
cd Groot
cmake -S . -B build
cmake --build build
cd ${MANYMOVE_ROS_WS}
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

**8. Build the workspace**
```bash
cd ${MANYMOVE_ROS_WS} && colcon build
```

**9. Source the overlay**
```bash
source ${MANYMOVE_ROS_WS}/install/setup.bash
```

That’s it! Repeat steps 3–9 whenever you open a new shell.

---

## Bring your own robot

If you want to create a new application in ManyMove, once you have a working moveit config package you just need the following information:
- Base frame/link of the robot
- TCP (end effector) frame/link
- traj_controller action server name
- Robot's name/model
- Planner type: moveitcpp or movegroup

When using an actuated gripper you also need:
- The name of the action server to control the gripper
- The list of the gripper's links to exclude from collision checking

When using multiple robots with a single URDF, you'll need a *prefix* for each robot (see the dual_* examples).

These values will be used to start up the following nodes:
- action_server_node: the core of the planners
- manymove_py_trees_node: the BT logic node
- manymove_hmi_node: the HMI interface
- manymove_signals: on a real robot, handles the service calls to get/set signals (currently taylored for Ufactory robots)

A little side note on Panda examples: the reference TCP frame there is not actually centered between gripper's fingers, as 'panda_link8' represents the center of the flange. But since it's aligned to the ideal TCP, you can just offset the poses when you need to refer to the TCP, without having to create a new link. In the Panda example, the **-0.102** in the following line represents this offset:

```
blackboard->set("pick_pre_transform_xyz_rpy_1_key", std::vector<double>{-0.102, 0.0, 0.0, 0.0, 1.57, 0.0});
```

Right now the most extensive executable examples are found in manymove_cpp_trees repo: you can start from one of the executables there, taking care to use valid joint targets and poses for your robot. Also take some time setting up *move.hpp* coherently with the speed limits of your robot and planning pipeline of your choice.
When you go through the code, you'll notice I explain what each section does and how to use it: I tried to keep it updated while modifying the repo, but some comments may be outdated or not relevant anymore. **Please let me know if something is not clear**!

---

## Project Structure
  
![ManyMove structure](media/manymove_structure.png)

This repository is composed of several sub-packages, each handling different responsibilities in the overall robotic application:

1. **`manymove_msgs`**  
   - Holds all the **custom action definitions** (e.g. `MoveManipulator`, `AttachDetachObject`, `GetInput`, etc.) and custom messages required for robot manipulation, collision object management, and I/O signaling.  
   - These definitions are shared across other packages to keep interfaces consistent.

2. **`manymove_planner`**  
   - Implements the **motion-planning logic** using MoveIt 2 and ROS 2 action servers.  
   - Offers the **Action Server** `move_manipulator` so other modules can request motion plans and execute trajectories.  
   - Contains configuration files for MoveIt 2 (e.g., `moveit_cpp.yaml`) and example launch files for single or dual robot setups.

3. **`manymove_object_manager`**  
   - Manages **collision objects** in the planning scene.  
   - Provides actions like `AddCollisionObject`, `RemoveCollisionObject`, `AttachDetachObject`, etc., which can be called by higher-level logic to handle objects in the environment.  
   - Includes mesh files and YAML configuration for objects.

4. **`manymove_signals`**  
   - Handles **digital I/O signals** and checks the robot’s state via dedicated actions (e.g., `CheckRobotState`, `SetOutput`, `GetInput`).  
   - Useful for toggling end-effector tools or reading sensor inputs in a flexible, standardized way.

5. **`manymove_cpp_trees`**  
   - A C++ **BehaviorTree.CPP** framework that integrates with the actions exposed by the planner, object manager, and signals packages.  
   - Offers custom BT nodes (e.g., planning, object manipulation, signal I/O, conditions, etc.) so you can compose robotic behaviors in a modular, visual manner.  
   - Contains a **BT client** nodes (e.g. `bt_client.cpp`) that demonstrates how to build, run, and manage complex behavior trees at runtime.

6. **`manymove_py_trees`**  
   - A Python-based alternative using **py_trees** to build or test similar control flows.  
   - Useful if you prefer Python or need quick scripting for behavior logic.  
   - Mirrors some capabilities found in the C++ trees package.

7. **`manymove_hmi`**  
   - Implements a **basic Human–Machine Interface (HMI)**.  
   - Provides a GUI (`hmi_gui`) and related tools (`ros2_worker`) so that operators can issue commands, monitor status, or set parameters.  
   - Can be integrated with the behavior trees (or any other logic) to pause, stop, or resume execution.

8. **`manymove_bringup`**   
   - Contains the launchers for the complete multi-package examples listed below (and more).

---

## Tutorials

The first tutorial is now available [`HERE: tutorial_01`](./manymove_cpp_trees/tutorials/tutorial_01.md).

Starting from an empty scene, you'll delop a pick and place application with ManyMove:

<img src="./manymove_cpp_trees/tutorials/media/tutorial_01.gif" alt="Tutorial_01" width="640"/>

## Examples

### Launching the Examples

- **Lite6, uf850 and xarm7 manipulators** 
  with MoveItCPP and BehaviorTree.CPP:
  ```bash
  ros2 launch manymove_bringup lite_moveitcpp_fake_cpp_trees.launch.py
  ```
  with MoveItCPP and BehaviorTree.CPP:
  ```bash
  ros2 launch manymove_bringup uf850_moveitcpp_fake_cpp_trees.launch.py
  ```
  with MoveItCPP and BehaviorTree.CPP:
  ```bash
  ros2 launch manymove_bringup xarm7_moveitcpp_fake_cpp_trees.launch.py
  ```
  
  with MoveGroupInterface and BehaviorTree.CPP:
  ```bash
  ros2 launch manymove_bringup lite_movegroup_fake_cpp_trees.launch.py
  ```
  ```bash
  ros2 launch manymove_bringup uf850_movegroup_fake_cpp_trees.launch.py
  ```
  ```bash
  ros2 launch manymove_bringup xarm7_movegroup_fake_cpp_trees.launch.py
  ```
  
  with MoveGroupInterface and py_trees (minimal):
  ```bash
  ros2 launch manymove_bringup lite_movegroup_fake_py_trees.launch.py
  ```
- **Dual robot (Lite 6 + UF850)**  
  ```bash
  ros2 launch manymove_bringup dual_moveitcpp_fake_cpp_trees.launch.py
  ```

- **Panda Manipulator** (requires the installation of [moveit2_tutorials](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html) )

  Standalone launchers for MoveItCPP or MoveGroup with Panda robot (you need the moveit2_tutorials package sourced):

    with BehaviorTree.CPP:
     ```bash
     ros2 launch manymove_bringup panda_moveitcpp_fake_cpp_trees.launch.py
     ```
     ```bash
     ros2 launch manymove_bringup panda_movegroup_fake_cpp_trees.launch.py
     ```
    with py_trees:
     ```bash
     ros2 launch manymove_bringup panda_movegroup_fake_py_trees.launch.py
     ```
  Alternative with standard panda demo launch and manymove started from a separate launcher:
  
  In **terminal 1** (with `moveit2_tutorials` installed and sourced):
     ```bash
     ros2 launch moveit2_tutorials demo.launch.py
     ```
  In **terminal 2**, with MoveGroupInterface and BehaviorTree.CPP:
     ```bash
     ros2 launch manymove_planner panda_fake_cpp_trees.launch.py
     ```

   **Alternative**: **in terminal 2**, with MoveGroupInterface and **py_trees** (minimal):
     ```bash
     ros2 launch manymove_planner panda_fake_py_trees.launch.py
     ```

These launch files spin up the appropriate environment (fake or real) plus the nodes that handle planning, object management, signals, and optional HMI components. You can then interact with these action servers and send them requests using the provided C++ or Python-based behavior tree clients.

For launchers that use **NVIDIA cuMotion** planning library, refer to [THIS README](https://github.com/pastoriomarco/manymove/blob/humble/manymove_planner/config/isaac_ros/cumotion_startup.md).

---

## Credits

- **BehaviorTree.CPP v3.8** installed through ROS dependencies (`ros-humble-behaviortree-cpp-v3`)  
  and its visualizer [Groot](https://github.com/BehaviorTree/Groot).
    - Groot is to be installed manually following the instructions in the [github page](https://github.com/BehaviorTree/Groot?tab=readme-ov-file#dependencies-installation-and-usage)
- **py_trees_ros** from [splintered-reality/py_trees_ros](https://github.com/splintered-reality/py_trees_ros), installed through ROS dependencies, including its visualizer **ros-humble-py-trees-ros-viewer**.
- The MoveIt 2 community and [xarm_ros2 on GitHub](https://github.com/xArm-Developer/xarm_ros2/tree/humble) for the underlying robot drivers and examples.

---

## Notes & Disclaimer

- **Experimental**: This entire project is under development and may change rapidly.
- **No Safety Features**: Robot safety must be handled by separate hardware or controller-level solutions.
- **Feedback Welcome**: Please open an issue or pull request if you find improvements or have suggestions.

Enjoy experimenting with ManyMove in your ROS2 environment, but remember to keep safety a top priority!
