# manymove_msgs

## Overview
- ROS 2 interface package that defines the actions, messages, and services used throughout the ManyMove system.
- Supports core workflows including motion planning (`PlanManipulator`/`MoveManipulator`), object management (collision object and pose actions), and robot I/O plus state coordination.
- Refer to the top-level [ManyMove README](../README.md) for installation, build, and full usage guidance.

## Interfaces

### Actions
- `AddCollisionObject.action` – Adds a primitive or mesh object to the planning scene with pose and optional mesh scaling.
- `AttachDetachObject.action` – Attaches or detaches a named collision object to a robot link while managing touch links.
- `CheckObjectExists.action` – Reports whether an object exists in the scene and whether it is currently attached to the robot.
- `CheckRobotState.action` – Checks controller readiness and returns status codes describing the robot’s current mode and state.
- `GetInput.action` – Reads a discrete tool or controller I/O channel and returns its ON/OFF value.
- `GetObjectPose.action` – Retrieves an object pose, applying optional pre/post transforms relative to a specified link.
- `LoadTrajController.action` – Requests the controller manager to load a trajectory controller and reports progress.
- `MoveManipulator.action` – Executes a planned or supplied trajectory and monitors progress and collision feedback.
- `PlanManipulator.action` – Generates a robot trajectory from a `MoveManipulatorGoal` request.
- `RemoveCollisionObject.action` – Removes a named collision object from the planning scene.
- `ResetRobotState.action` – Triggers a robot state reset sequence and reports success or failure.
- `SetOutput.action` – Writes an ON/OFF value to a tool or controller output channel.
- `UnloadTrajController.action` – Unloads a named trajectory controller and reports completion progress.

### Messages
- `MoveManipulatorGoal.msg` – Describes the requested manipulator motion (pose, joint, named target, or cartesian) and associated configuration.
- `MovementConfig.msg` – Captures planning, smoothing, and Cartesian interpolation parameters used to customize motion execution.

### Services
- `SetBlackboardValues.srv` – Updates multiple behavior tree blackboard keys with typed string-encoded values in one request.

## Dependencies
- `geometry_msgs` – Provides `Pose` primitives for target and object pose representations.
- `moveit_msgs` – Supplies `RobotTrajectory` messages exchanged during planning and execution.
- `control_msgs` and `controller_manager_msgs` – Define controller management interfaces used by the trajectory load/unload actions.
- `std_msgs` and `std_srvs` – Contribute common message and service primitives leveraged across the interface set.
- `shape_msgs` – Enables describing collision object geometry beyond basic primitives.
- `tf2`, `tf2_ros`, `tf2_geometry_msgs` – Support expressing object poses relative to different frames via transforms.
- `rosidl_default_generators` and `rosidl_default_runtime` – Generate and provide runtime support for the ROS 2 interfaces in this package.

## Usage
- For workspace setup, dependencies, and launch instructions, follow the top-level [ManyMove README](../README.md).

```

## License and Maintainers
This package is licensed under BSD-3-Clause. Maintainer: Marco Pastorio <pastoriomarco@gmail.com>.
See main [ManyMove README](../README.md) for `CONTRIBUTION` details.
