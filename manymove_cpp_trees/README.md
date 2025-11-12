# ManyMove C++ Trees

`manymove_cpp_trees` extends [ManyMove](../README.md) with a BehaviorTree.CPP toolkit that orchestrates planners, object management, and signal handling for ROS 2 Humble applications. It ships reusable BT nodes plus sample clients that demonstrate how to stitch robotic behaviors together.

**Caution:** the package is experimental and provides no built-in safety features.

## Overview
- BehaviorTree.CPP integration for planning, object manipulation, gripper control, I/O checks, and Isaac Sim bridges.
- Runtime blackboard utilities that keep motion goals, object metadata, and HMI feedback configurable without recompiling.
- Example BT clients that exercise end-to-end pick, place, and dual-arm scenarios while spinning an accompanying HMI service node.

## Behavior Tree Node Families
- `action_nodes_planner.hpp` – plans and executes manipulator moves via `manymove_planner`, caching trajectories and exposing reset hooks.
- `action_nodes_objects.hpp` – creates, attaches, detaches, and queries collision objects through `manymove_object_manager` actions.
- `action_nodes_signals.hpp` – toggles digital I/O, inspects robot state, and (un)loads controllers using `manymove_msgs` actions.
- `action_nodes_gripper.hpp` – wraps `control_msgs` gripper command and trajectory actions plus a utility to publish `sensor_msgs/JointState`.
- `action_nodes_logic.hpp` – provides flow-control helpers (pause/reset decorators, blackboard guards, wait conditions).
- `action_nodes_isaac.hpp` – exchanges poses with Isaac Sim through `simulation_interfaces` services and detection topics.

Support code in `tree_helper.hpp`, `blackboard_utils.hpp`, and `main_imports_helper.hpp` builds XML snippets, seeds default parameters, and registers node types with the BehaviorTree.CPP factory.

## Runtime Services
- [`HMIServiceNode`](./include/manymove_cpp_trees/hmi_service_node.hpp) exposes the `update_blackboard` service and publishes JSON-formatted blackboard snapshots every 250 ms so external HMIs can monitor or update execution state.

## Executables
- `bt_client` – single-arm sample that builds a programmatic tree for pick-and-place, including motion planning, object updates, and signal checks.
- `bt_client_dual` – coordinates two robot prefixes and shared objects to demonstrate cooperative sequencing.
- `bt_client_app_dual` – richer dual-arm application with blackboard-driven parameters, mesh scaling, and HMI messaging.
- `bt_client_isaac` – connects the tree to Isaac Sim entities and publishes joint commands for simulated hardware.
- `collision_test` – regression scenario that stresses collision handling and trajectory resets within the planner pipeline.
- `tutorial_01` / `tutorial_01_complete` – incremental learning examples showing how to assemble and extend the helper utilities.

Each executable spins the BT factory, registers the custom nodes, and runs the `HMIServiceNode` alongside a `MultiThreadedExecutor`. The created ROS node is named `bt_client_node`.

## Usage
- For workspace setup, dependencies, and launch instructions, follow the top-level [ManyMove README](../README.md).

## Dependencies
- BehaviorTree.CPP (`behaviortree_cpp_v3`)
- ManyMove core packages: `manymove_planner`, `manymove_object_manager`, `manymove_msgs`
- Motion and transforms: `trajectory_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- Control and runtime messaging: `rclcpp`, `rclcpp_action`, `control_msgs`, `std_msgs`, `std_srvs`, `topic_based_ros2_control`
- Simulation and perception bridges: `simulation_interfaces`, `vision_msgs`

## Notes
- Robot safety mechanisms (stop buttons, workspace supervision) must be handled externally.
- Review the main README for project-wide disclaimers, contribution guidance, and licensing details.

## License and Maintainers
This package is licensed under BSD-3-Clause. Maintainer: Marco Pastorio <pastoriomarco@gmail.com>.
See main [ManyMove README](../README.md) for `CONTRIBUTION` details.
