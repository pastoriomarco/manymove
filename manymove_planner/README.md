# ManyMove Planner

This package provides a **simplified yet flexible** set of motion planning and trajectory execution action servers for robotic manipulators, built on **ROS 2** and **MoveIt 2**. It is part of the **`manymove`** project and is highly experimental. **Use at your own risk**; see the main repository’s disclaimer regarding safety and stability.

`manymove_planner` exposes action servers that make possible to organize the planning and execution actions in logic builder packages using a unified interface, without having to worry if the underlying implementation uses MoveGroupInterface or MoveitCPP.
It supports diverse robots, with an example configuration for Franka Emika Panda, which is the default demo model for Moveit, but most of the exmples are built around Ufactory Lite6 and UF850, simply because I'm going in production with these first.

## Overview

### Key Functionalities

- **Plan Manipulator Motions**\
  Accepts single-goal planning requests via the `PlanManipulator` action (joint, named, pose, or Cartesian targets).\
  Applies time parameterization to generate smooth, velocity-scaled trajectories.

- **Execute Trajectory**\
  Executes a pre-computed `RobotTrajectory` using a standard FollowJointTrajectory controller.\
  Provides feedback on collision checks during execution (verifies upcoming waypoints).

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

   - **`plan_manipulator`**: Receives planning requests, returns the resulting `RobotTrajectory`.
   - **`execute_manipulator_traj`**: Executes a given `RobotTrajectory`, checking collisions during execution.
   - **`stop_motion`**: Sends a single-point, decelerating trajectory to stop the manipulator safely.
   - **`load_trajectory_controller`**\*\* / \*\*\*\*`unload_trajectory_controller`\*\*: Dynamically loads/unloads a trajectory controller.

4. **ROS 2 Node**

   - The main node (`action_server_node`) loads parameters and spins up the chosen planner plus the action servers.
   - Supports **prefixes** (e.g., `prefix:=L_` or `prefix:=R_`) for multi-robot setups.

---

## Installation & Dependencies

Please refer to the main [**ManyMove README**](../README.md) for overall setup instructions, build steps, and prerequisites.

### Controllers

This package expects a standard `FollowJointTrajectory`-type controller (e.g., a `JointTrajectoryController`). Use the included actions to dynamically load or unload controllers if needed.

---

## Usage

### Launching a Fake Robot & Action Server

A typical example uses a **UFactory Lite 6** manipulator in a fake simulation:

```bash
ros2 launch manymove_planner lite_micpp_fake_action_server.launch.py
```

This:

- Spawns a mock `ros2_control_node` for the Lite6.
- Launches **RViz** (with MoveIt 2).
- Starts the **`action_server_node`** with `planner_type=moveitcpp`.
- Publishes transforms, joint states, etc.

### Action Topics

Once launched, you should see:

- `/<prefix>plan_manipulator`
- `/<prefix>execute_manipulator_traj`
- `/<prefix>stop_motion`
- `/<prefix>load_trajectory_controller`
- `/<prefix>unload_trajectory_controller`

(`prefix` may be empty or something like `L_`.)

### Sending Goals

**PlanManipulator**

- Send a goal specifying `movement_type` = `"joint"`, `"named"`, `"pose"`, or `"cartesian"`.
- Receives a planned (and time-parameterized) `RobotTrajectory`.

**ExecuteTrajectory**

- Provide a `RobotTrajectory` (often the result of `PlanManipulator`).
- The server executes it and provides feedback/collision checks.

**StopMotion**

- Send a simple goal to `<prefix>stop_motion` for a controlled stop.

**Load/Unload Trajectory Controllers**

- Dynamically manage underlying controllers: note that this part was created as a workaround to a bug that made&#x20;

---

## Configuration & Parameters

Key parameters (often set via launch files):

- **`planner_type`**:

  - Selects the underlying planning approach.
  - Valid options: `"moveitcpp"` (default) or `"movegroup"`.

- **`planning_group`**:

  - Sets the MoveIt planning group name for the manipulator (e.g., `"lite6"`).

- **`base_frame`**:

  - The robot’s base link frame.

- **`tcp_frame`**:

  - The manipulator’s end-effector (TCP) frame.

- **`velocity_scaling_factor`**:

  - Scales overall motion velocity (0.0 to 1.0).

- **`acceleration_scaling_factor`**:

  - Scales overall motion acceleration (0.0 to 1.0).

- **`max_cartesian_speed`**:

  - Limits Cartesian speed in m/s during trajectory execution.

- **`step_size`**:

  - Step size for Cartesian path planning.

- **`jump_threshold`**:

  - Threshold for detecting abrupt jumps in Cartesian planning.

- **`plan_number_target`**:

  - Target number of planning attempts for sampling-based planners.

- **`plan_number_limit`**:

  - Maximum number of planning attempts before giving up.

- **`traj_controller`**:

  - Specifies the name of the FollowJointTrajectory controller for execution.

### `max_cartesian_speed` Parameter

This parameter was introduced to ensure the manipulator’s linear velocity does not exceed a given limit. If a planned trajectory’s computed Cartesian speed surpasses `max_cartesian_speed`, the system automatically scales down the `velocity_scaling_factor` to keep the motion under that threshold. This helps maintain safer, more controlled movements without sacrificing time parameterization.

## Notes & Disclaimer

- This package does **not** provide safety features. Always configure appropriate safety systems on your robot controller.
- It is highly experimental; use with caution.
- See the [manymove main README](../README.md) for disclaimers, licensing, and credits.

---

## Contributing

Contributions and issue reports are welcome. If you encounter bugs or have suggestions, please open an issue or pull request in the main **[manymove](https://github.com/pastoriomarco/manymove)** repository.
