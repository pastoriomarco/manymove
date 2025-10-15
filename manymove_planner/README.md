# manymove_planner

`manymove_planner` packages motion-planning action servers that wrap MoveIt 2 so downstream logic (behavior trees, task planners, HMI) can issue consistent actions for industrial robots. It sits alongside the bringup, object management, and message packages in the ManyMove workspace.

Refer to the repository-level [README](../README.md) for workspace setup, build instructions, and global safety guidance.

## Overview

- Hosts action servers that plan and execute manipulator motions via MoveIt 2 with collision checking, time parameterisation, and velocity/acceleration scaling.
- Supports joint, pose, Cartesian, and named-target requests and exposes helper actions for loading/unloading FollowJointTrajectory controllers.
- Provides a controlled stop interface that back-drives trajectory controllers to achieve smooth deceleration.
- Ships ready-to-launch examples for fake hardware, real controllers, and dual-arm deployments.

## Core Components

- **ManipulatorActionServer** (`include/manymove_planner/action_server.hpp`, `src/action_server.cpp`): wraps the Move / Plan / Load / Unload actions from `manymove_msgs`, applies collision validation, and dispatches trajectories to the selected controller.
- **PlannerInterface** (`include/manymove_planner/planner_interface.hpp`): abstract API defining `plan`, `applyTimeParameterization`, trajectory validation, and `sendControlledStop`.
- **MoveGroupPlanner** (`src/move_group_planner.cpp`): implements `PlannerInterface` on top of `MoveGroupInterface`, supporting joint, pose, Cartesian, and named goal types with time parameterisation and Cartesian speed limiting.
- **MoveItCppPlanner** (`src/moveit_cpp_planner.cpp`): `PlannerInterface` implementation that reuses a shared `moveit_cpp::MoveItCpp` instance, providing path-length estimation, motion smoothing, and direct access to PlanningSceneMonitor services.
- **Executables** from `CMakeLists.txt`:
  - `action_server_node`: parameter-driven node that instantiates either planner backend; supports per-robot prefixes (`node_prefix`) and controller names.
  - `moveitcpp_action_server_node`: creates one `MoveItCpp` instance and spins multiple `ManipulatorActionServer`s for multi-robot/dual-arm applications.

## Launch & Usage
- For workspace setup, dependencies, and launch instructions, follow the top-level [ManyMove README](../README.md).

## Interactions

- **Messages and actions:** consumes `manymove_msgs` definitions (`PlanManipulator`, `MoveManipulator`, `LoadTrajController`, `MovementConfig`) for goals and constraints.
- **Planning scene:** pairs with `manymove_object_manager` to populate collision objects; launch files start this node alongside the planner so the MoveIt planning scene stays synchronised.
- **Robot bringup:** relies on bringup packages to load URDF, SRDF, controllers, and ros2_control pipelines before motion commands are accepted.
- **MoveIt dependencies:** uses `moveit_core`, `moveit_ros_planning_interface`, and `moveit_cpp::PlanningComponent` for planning, and `control_msgs/FollowJointTrajectory` for execution feedback.

## Dependencies

Key runtime dependencies declared in [`package.xml`](package.xml) include:

- `rclcpp`, `rclcpp_action`, `action_msgs` for ROS 2 node and action infrastructure.
- `moveit_core`, `moveit_ros_planning_interface`, `geometry_msgs`, `tf2_geometry_msgs`, `moveit_msgs` for planning and scene representation.
- `control_msgs`, `controller_manager_msgs` to command trajectory controllers and manage their lifecycle.
- `manymove_msgs` for custom action/goal definitions shared across the ManyMove stack.

## Extending the Planner

To integrate a new planning backend:

1. Implement `PlannerInterface`, providing planning, time-parameterisation, trajectory validation, and stop logic tailored to your backend.
2. Link the new class into `action_server_node` or a custom executable and expose a `planner_type` parameter so launch files can select it.
3. Add any backend-specific configuration to the `config/` directory and reference it from your launch description.

## License and Maintainers
This package is licensed under BSD-3-Clause. Maintainer: Marco Pastorio <pastoriomarco@gmail.com>.
See main [ManyMove README](../README.md) for `CONTRIBUTION` details.
