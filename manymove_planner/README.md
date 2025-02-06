# ManyMove Planner

The `manymove_planner` project is meant for roboticists to ease the transition to ROS2 coming from the classic frameworks of major manifacturers.
It provides a simplified and robust framework for planning and executing robotic manipulator movements using ROS 2 and MoveIt 2. 
It supports diverse robots, with example configurations of Franka Emika Panda, which is the default demo model for Moveit, and Ufactory Lite6 and UF850.

## Features

- **Action-Based Motion Control**:
  - Single move goals with joint, position or Cartesian targets.
  - Sequential move goals for executing a series of movements.
- **Integration with MoveIt 2**:
  - Supports motion planning, Cartesian path planning, and time parameterization.
- **Flexible Planner Options**:
  - Choose between MoveItCppPlanner and MoveGroupPlanner for motion planning.
- **Customizable Parameters**:
  - Control velocity, acceleration, Cartesian motion constraints, and more via ROS parameters or launch files.

## Architecture Overview

1. **Planner Interface**:
   - Defines abstract methods for planning, execution, and time parameterization.
   - Implemented by `MoveItCppPlanner` and `MoveGroupPlanner`.
2. **Action Server**:
   - Handles incoming motion requests and delegates tasks to the chosen planner.
   - Provides feedback and manages action lifecycle.
3. **Action Server Node**:
   - Configures and initializes the appropriate planner and action server.
   - Supports runtime parameter configuration.
4. **Client Node**:
   - Demonstrates sending single and sequence motion goals.

## Message and Action Definitions

### **MovementConfig.msg**
Defines the configuration parameters for robotic movements:

**Planning parameters for all movements:**
- `velocity_scaling_factor`: Factor to scale the velocity (0.0 - 1.0), set on planner parameters, potentially modified by `manymove_planner` functions.
- `acceleration_scaling_factor`: Factor to scale the acceleration (0.0 - 1.0), set on planner parameters, potentially modified by `manymove_planner` functions.
- `max_cartesian_speed`: Maximum speed for Cartesian movements (m/s), enforced by `manymove_planner` functions.
- `smoothing_type`: Type of trajectory smoothing (e.g., time-optimal, spline), configured for `manymove_planner` functions.

**Planning parameters only for Cartesian/linear movements:**
- `step_size`: Step size for Cartesian planning.
- `jump_threshold`: Threshold to avoid jumps in Cartesian paths.

**Planning parameters for optimizing path length with repeated planning attempts:**
- `plan_number_target`: Target number of plans for sampling-based planners.
- `plan_number_limit`: Limit on the number of plans for sampling-based planners.

**Execution parameters:**
- `max_exec_tries`: Maximum execution attempts for retrying plans.

### **MoveManipulatorGoal.msg**
Represents a single goal for manipulator movement:

**IMPORTANT:** Only one type of movement is considered for each move, determined by the `movement_type` variable. If other movement fields are specified, they will be ignored unless correctly aligned with `movement_type`.

- `movement_type`: Type of movement (`pose`, `joint`, `named`, or `cartesian`).
- `pose_target`: Desired pose for the manipulator, used for `pose` or `cartesian` moves.
- `named_target`: Predefined named target position, used for `named` joint moves.
- `joint_values`: Specific joint values for the manipulator, used for `joint` moves.

**IMPORTANT:**
`start_joint_values` should be specified only if planning a trajectory that does not start from the current robot position. If a trajectory planned with `start_joint_values` is executed from a different start position, the execution will fail.

- `start_joint_values`: Starting joint configuration for non-default planning.
- `config`: Configuration parameters for movement, defined in `MovementConfig`.


### **MoveManipulator.action**
Defines the structure for single-move actions:
- **Goal**:
  - `goal`: A `MoveManipulatorGoal` object specifying the desired movement.
- **Result**:
  - `success`: Boolean indicating whether the movement succeeded.
  - `message`: Detailed message about the result.
- **Feedback**:
  - `progress`: Progress of the movement as a percentage (0.0 - 100.0).

### **MoveManipulatorSequence.action**
Defines the structure for sequential-move actions:
- **Goal**:
  - `goals`: An array of `MoveManipulatorGoal` objects for the sequence.
- **Result**:
  - `success`: Boolean indicating whether the sequence succeeded.
  - `message`: Detailed message about the result.
- **Feedback**:
  - `progress`: Progress of the sequence as a percentage (0.0 - 100.0).

## Requirements

- ROS 2 (tested on Humble)
- MoveIt 2 & its relative robot package

## Installation

### Dependencies
Ensure ROS 2 and MoveIt 2 are installed. Follow the [MoveIt 2 installation guide](https://moveit.ros.org/install/) for your platform.

### Build Instructions
1. Clone the repository into your workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> manymove_planner
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Launching the Action Server

- **Lite Series Manipulator**:
  ```bash
  ros2 launch manymove_planner lite_action_server_node.launch.py
  ```

- **UF850 Manipulator**:
  ```bash
  ros2 launch manymove_planner uf850_action_server_node.launch.py
  ```

- **Panda Manipulator**:
  ```bash
  ros2 launch manymove_planner panda_action_server_node.launch.py
  ```

- **Custom Configuration**:
  ```bash
  ros2 launch manymove_planner manymove_planner.launch.py
  ```

### Client Example

- **Single Goal**:
  Sends a single motion request (e.g., moving to a pose or joint target).
  ```bash
  ros2 run manymove_planner client_example_node
  ```

- **Sequence of Goals**:
  Demonstrates planning and executing a series of moves with feedback.
  The feedback is needed to understand at what point of the motion sequence the robot currently is, and to use this info to manage an execution failure.

### Configuration Parameters

- **Planner Type**:
  - Choose `moveitcpp` or `movegroup` planners:
    ```bash
    --ros-args -p planner_type:=moveitcpp
    ```

- **Motion Planning Parameters**:
  - Define the planning group, base frame, and tool frame:
    ```bash
    --ros-args -p planning_group:=<group_name> -p base_frame:=<frame_name> -p tcp_frame:=<frame_name>
    ```

- **Velocity and Acceleration**:
  - Configure motion scaling factors:
    ```bash
    --ros-args -p velocity_scaling_factor:=0.5 -p acceleration_scaling_factor:=0.5
    ```

- **Cartesian Constraints**:
  - Limit Cartesian speeds:
    ```bash
    --ros-args -p max_cartesian_speed:=0.5
    ```

### Workflow

1. **Client Sends Request**:
   - A single or sequence motion goal is sent using the `MoveManipulator` or `MoveManipulatorSequence` action.
2. **Action Server Processes Request**:
   - Handles the goal, plans a trajectory, applies time parameterization, and executes it using the configured planner.
3. **Feedback and Results**:
   - For sequences, periodic feedback is provided during execution.
   - Final results are sent to the client on completion or failure.

## Core Components

### **Planner Interface**
Defines a common interface for motion planners, including methods for:
- Planning single and sequence trajectories.
- Executing trajectories with or without feedback.
- Applying time parameterization.

### **MoveItCppPlanner**
Uses MoveItCpp for planning and execution. Supports:
- Flexible configurations.
- Integration with MoveIt trajectory smoothing and parameterization.

### **Action Server**
Manages action lifecycle for single and sequential moves:
- Processes goals and cancellations.
- Executes planned trajectories using the configured planner.

### **Client Example Node**
Demonstrates:
- Sending single or sequence motion goals.
- Configuring movement parameters such as velocity and acceleration.

## Contribution

Contributions are welcome. Please submit issues or pull requests for new features or bug fixes.

## License

This project is licensed under the Apache 2.0 License. See the `LICENSE` file for details.
