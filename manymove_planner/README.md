# ManyMove Planner

This package provides a **simplified** set of motion planning and trajectory execution action servers for robotic manipulators, built on **ROS 2** and **MoveIt 2**. It is part of the **`manymove`** project and is highly experimental. **Use at your own risk**; see the main repository’s disclaimer regarding safety and stability.

`manymove_planner` exposes action servers that make possible to organize the planning and execution actions in logic builder packages using a unified interface, without having to worry if the underlying implementation uses MoveGroupInterface or MoveitCPP.
It supports diverse robots, with an example configuration for Franka Emika Panda, which is the default demo model for Moveit, but most of the exmples are built around Ufactory Lite6 and UF850, simply because I'm going in production with these first.

## Overview

### Key Functionalities

- **Move Manipulator Action Server**\
  Accepts single-goal planning requests via the `MoveManipulator` action (joint, named, pose, or Cartesian targets).\
  Applies time parameterization to generate smooth, velocity-scaled trajectories.
  Uses cartesian speed limit for all moves.
  Handles soft stop with spring-back action and deceleration time on all moves.

- **Stop Motion**\
  Issues a controlled stop command to smoothly decelerate the manipulator.

- **Load & Unload Controllers**\
  Offers dedicated actions (`LoadTrajController` and `UnloadTrajController`) for dynamic controller management (e.g., loading a new trajectory controller or unloading one at runtime).

### Architecture

1. **Planner Interface**\
   An abstract C++ interface (`planner_interface.hpp`) defining essential methods:

   - `plan(...)`: path planning for joint, named, pose, or Cartesian goals.
   - `applyTimeParameterization(...)`: smoothing and velocity constraint application.
   - `executeTrajectory(...)`: sends trajectories to the robot’s controller.
   - `sendControlledStop(...)`: gently stops motion on demand.

2. **Planner Implementations**

   - **`MoveItCppPlanner`**: Uses MoveItCpp for planning, collision checks, and parametric trajectory generation.
   - **`MoveGroupPlanner`**: (Alternative) leverages `move_group_interface` for those who prefer MoveGroup’s standard pipeline.

3. **Action Servers** (in `action_server.cpp` & `action_server_node.cpp`):

   - **`move_manipulator`**: Receives planning requests, executes the resulting `RobotTrajectory`.
   - **`stop_motion`**: Sends a single-point, decelerating trajectory to stop the manipulator safely.
   - **`load_trajectory_controller` / `unload_trajectory_controller`**: Dynamically loads/unloads a trajectory controller.

4. **ROS 2 Node**

   - The main node (`action_server_node`) loads parameters and spins up the chosen planner plus the action servers.
   - Supports **prefixes** (e.g., `prefix:=L_` or `prefix:=R_`) for multi-robot setups.

---

## Installation & Dependencies

Please refer to the main [**ManyMove README**](../README.md) for overall setup instructions, build steps, and prerequisites.

### Controllers

This package expects a standard `FollowJointTrajectory`-type controller (e.g., a `JointTrajectoryController`). Use the included actions to dynamically load or unload controllers if needed.

---

## Configuration & Parameters

Key parameters (set via launch files):

- **`planner_type`**:

  - Selects the underlying planning approach.
  - Valid options: `"moveitcpp"` (default) or `"movegroup"`.

- **`planning_group`**:

  - Sets the MoveIt planning group name for the manipulator (e.g., `"lite6"`).

- **`base_frame`**:

  - The robot’s base link frame.

- **`tcp_frame`**:

  - The manipulator’s end-effector (TCP) frame.

## Notes & Disclaimer

- This package does **not** provide safety features. Always configure appropriate safety systems on your robot controller.
- It is highly experimental; use with caution.
- See the [manymove main README](../README.md) for disclaimers, licensing, and credits.
