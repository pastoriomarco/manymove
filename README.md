# ManyMove project - `ROS2 humble`

![ManyMove example](https://github.com/pastoriomarco/manymove/blob/dfc9c7f00e11d70d5e27fd2e0c13bfcf5de6de54/media/manymove_example.gif)

## DISCLAIMER

This repository is intended primarily for my own use as a roboticist, to deploy cobots in production using ROS2. 
 
This is **HIGHLY EXPERIMENTAL** and comes with no warranty of stability or safety: **USE AT YOUR OWN RISK**.

**IMPORTANT SAFETY NOTICE**: this repository does not cover safety functions. Safety **MUST** be implemented using the internal safety system of the robot's controller and/or an appropriate safety controller, in compliance with your country’s regulations. Before deploying on a real robot, ensure that safety mechanisms are correctly configured.

---

## Description

The `manymove` project is meant for roboticists to ease the transition to ROS2 coming from the classic frameworks of major manufacturers.  
It provides a simplified, generalized framework for building robotic manipulator control logic using ROS 2 and MoveIt 2.  
This series of packages was created around Ufactory Lite6 and UF850 cobots, but it is structured in a way that can extend to other robots. Also included is an example using Franka Emika Panda, which is the default demo model for Moveit in ROS2 Humble.

---

## Prerequisites

- **Install ROS2 Humble, Moveit2 and xarm_ros2**:
  - You can follow the instructions on the Humble branch of [xarm_ros2 on github](https://github.com/xArm-Developer/xarm_ros2/tree/humble) to install all the required packages.

---

## Quick start

- **Define your `workspace dir`**:
  - You can use for example ~/dev_ws as in xarm_ros2 repo, or define an appropriate workspace.
  - From here on I'll refer to the installation directory of the workspace as `<workspace_dir>`
- **Clone `manymove` humble branch**:
  - From `<workspace_dir>`:
  ```bash
  git clone --branch=humble https://github.com/pastoriomarco/manymove.git
  ```
-  **Install the dependencies**
  - From `<workspace_dir>`:
  ```bash
  rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
  ```
- **Copy the auxiliary files to run all examples with the right configuration**:
  - Create the `other` folder in xarm_description/meshes. From `<workspace_dir>` run:
  ```bash
  mkdir -p ./src/xarm_ros2/xarm_description/meshes/other
  ```
  - Copy pneumatic gripper's mesh. From `<workspace_dir>` run:
  ```bash
  cp ./src/manymove/manymove_object_manager/meshes/custom_end_tools/* ./src/xarm_ros2/xarm_description/meshes/other/
  ```
  - Copy the user param file in xarm_api/config. From `<workspace_dir>` run:
  ```bash
  cp ./src/manymove/manymove_planner/config/xarm_user_params.yaml ./src/xarm_ros2/xarm_api/config/
  ```
- **Build the packages from `<workspace_dir>`**: 
  ```bash
  colcon build
  ```
- **Don't forget to source!**
  - From `<workspace_dir>`:
  ```bash
  source ./install/setup.bash
  ```

---

  ## Project Structure
  
![ManyMove structure](media/manymove_structure.png)

This repository is composed of several sub-packages, each handling different responsibilities in the overall robotic application:

1. **`manymove_msgs`**  
   - Holds all the **custom action definitions** (e.g. `PlanManipulator`, `ExecuteTrajectory`, `AttachDetachObject`, etc.) and custom messages required for robot manipulation, collision object management, and I/O signaling.  
   - These definitions are shared across other packages to keep interfaces consistent.

2. **`manymove_planner`**  
   - Implements the **motion-planning logic** using MoveIt 2 and ROS 2 action servers.  
   - Offers an **Action Server** (e.g., `plan_manipulator`, `execute_trajectory`) so other modules can request motion plans and execute trajectories.  
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
   - Contains a **BT client** node (`bt_client.cpp`) that demonstrates how to build, run, and manage complex behavior trees at runtime.

6. **`manymove_py_trees`**  
   - A Python-based alternative using **py_trees** to build or test similar control flows.  
   - Useful if you prefer Python or need quick scripting for behavior logic.  
   - Mirrors some capabilities found in the C++ trees package.

7. **`manymove_hmi`**  
   - Implements a **basic Human–Machine Interface (HMI)**.  
   - Provides a GUI (`hmi_gui`) and related tools (`ros2_worker`) so that operators can issue commands, monitor status, or set parameters.  
   - Can be integrated with the behavior trees (or any other logic) to pause, stop, or resume execution.

---

## Examples

### Launching the Examples

- **Lite 6 manipulator** 
  with MoveItCPP and BehaviorTree.CPP:
  ```bash
  ros2 launch manymove_planner lite_micpp_fake_cpp_trees.launch.py
  ```
  
  with MoveGroupInterface and BehaviorTree.CPP:
  ```bash
  ros2 launch manymove_planner lite_movegroup_fake_cpp_trees.launch.py
  ```
  
  with MoveGroupInterface and py_trees (minimal):
  ```bash
  ros2 launch manymove_planner lite_movegroup_fake_py_trees.launch.py
  ```

- **Dual robot (Lite 6 + UF850)**  
  ```bash
  ros2 launch manymove_planner dual_micpp_fake_cpp_trees.launch.py
  ```
  - Note: to run the app example with the robots in custom positions you'll have to use the fork of xarm_ros2 modified to handle this kind of scenario. Follow the instructions in the above link from the original repository, but change the instruction on point 4.2 with:
    ```bash
    git clone https://github.com/pastoriomarco/xarm_ros2.git --recursive -b $ROS_DISTRO
    ```


- **Panda Manipulator** (requires the installation of [moveit2_tutorials](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html) )
  1. In terminal 1 (with `moveit2_tutorials` installed):
     ```bash
     ros2 launch moveit2_tutorials demo.launch.py
     ```
  2. In terminal 2:
    with MoveGroupInterface and BehaviorTree.CPP:
     ```bash
     ros2 launch manymove_planner panda_fake_cpp_trees.launch.py
     ```

    with MoveGroupInterface and py_trees (minimal):
     ```bash
     ros2 launch manymove_planner panda_fake_py_trees.launch.py
     ```
- 

These launch files spin up the appropriate environment (fake or real) plus the nodes that handle planning, object management, signals, and optional HMI components. You can then interact with these action servers and send them requests using the provided C++ or Python-based behavior tree clients.

---

## Architecture Flow

In a typical usage scenario:
1. **`manymove_planner`** (motion-planning + trajectory execution) and **`manymove_object_manager`** (collision objects) run their respective action servers.
2. **`manymove_signals`** handles the digital I/O action server.
3. A **behavior tree** (from **`manymove_cpp_trees`** in C++, or **`manymove_py_trees`** in Python) orchestrates these actions:
   - Check if the robot is ready and free of errors.
   - Add or remove objects from the scene.
   - Plan and execute a motion.
   - Send signals to open/close a gripper or read a sensor input, etc.
4. **`manymove_hmi`** can optionally provide a GUI or service-based interface to pause, stop, or resume these behaviors, and to visualize status or debug.

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
