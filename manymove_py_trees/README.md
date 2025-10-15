# manymove_py_trees

## Overview
- Provides a `py_trees`-based framework for orchestrating manipulation sequences in the ManYMove stack.
- Wraps the `manymove_msgs/MoveManipulator` action in a reusable behaviour so planners can be built as trees instead of bespoke action clients.
- Includes helpers for composing `Move` objects and chaining them into behaviour-tree sequences.
- Works alongside the ManYMove planner action servers launched by other packages; see the top-level README for build and installation instructions.

## Modules
- `move_manipulator_behavior.MoveManipulatorBehavior` – leaf node that sends `MoveManipulator` goals, monitors feedback, handles cancellation/reset signals, and stores the resulting `RobotTrajectory` on the blackboard.
- `tree_helper.create_tree_from_moves()` / `tree_helper.create_tree_from_sequences()` – utilities that turn ordered `Move` lists into `py_trees_ros` behaviour trees with one `MoveManipulatorBehavior` per step.
- `move_definitions.Move` – dataclass describing a single move (pose, joint, named, or cartesian) plus helpers such as `create_move`, `create_pose`, and `define_movement_configs`.
- Example clients `bt_client_fake.py` and `bt_client_panda.py` show how to wire the helpers into full ROS 2 nodes that tick a tree while sharing an HMI service.

## Usage
- For workspace setup, dependencies, and launch instructions, follow the top-level [ManyMove README](../README.md).

## Dependencies
- `rclpy`, `py_trees`, `py_trees_ros`
- `manymove_planner`, `manymove_msgs`

## License and Maintainers
This package is licensed under BSD-3-Clause. Maintainer: Marco Pastorio <pastoriomarco@gmail.com>.
See main [ManyMove README](../README.md) for `CONTRIBUTION` details.
