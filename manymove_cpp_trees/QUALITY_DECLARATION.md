# Quality Declaration – `manymove_cpp_trees`

This document declares the quality level for the `manymove_cpp_trees` package of the ManyMove project.

## 1. Summary

`manymove_cpp_trees` provides a BehaviorTree.CPP-based toolkit of C++ nodes and helper utilities that orchestrate motion planning (`manymove_planner`), collision object management (`manymove_object_manager`), grippers and I/O signals, and optional bridges to simulation/perception stacks. It also ships example BT clients and an HMI service node for blackboard inspection and updates.

The package follows ROS 2 and MoveIt coding standards and is actively maintained within the ManyMove repository (`dev` branch).

---

## 2. Version Policy

- Versioning: Semantic versioning (MAJOR.MINOR.PATCH) is followed. Current version: `0.1.0` (from `package.xml`).
- Change control: All changes are proposed via pull requests and reviewed on GitHub; automated CI builds validate the workspace.
- API stability: Public APIs (headers, node ports, expected blackboard keys, and XML helpers) are pre-1.0 and may change until `1.0.0`.

---

## 3. Documentation

- The README describes purpose, node families, executables, and dependencies.
- Public headers document available BT nodes and helpers where practical.
- Example clients demonstrate end-to-end trees and HMI interaction.
- Future work: add Doxygen/Sphinx generated API docs and richer usage examples.

---

## 4. Testing

### 4.1 Unit and Component Tests

- GTest-based tests are provided and cover:
  - BT converters, helper utilities, and flow-control nodes.
  - TF/pose utilities and blackboard interactions.
  - Action nodes for objects, signals, and grippers using lightweight fake action servers.
  - An integration test that builds a tree and ticks it against a fake `MoveManipulator` server.
- Tests run with a `MultiThreadedExecutor` where concurrency matters and validate node status transitions and blackboard outputs.
- Run with:
  ```bash
  colcon test --packages-select manymove_cpp_trees
  ```

### 4.2 Linters and Static Analysis

- Enforced via `ament_lint_auto` and `ament_lint_common` with `ament_cmake_uncrustify` configured.
- Typical checks include cpplint/cppcheck/flake8/xmllint/uncrustify where available.

### 4.3 Continuous Integration

- GitHub Actions builds the workspace on every push and pull request (Humble and Jazzy jobs).
- CI runs `colcon build`, `colcon test`, and linters for all packages.

### 4.4 Code Coverage

- No formal coverage reports yet. Target: integrate `gcovr`/Codecov and aim for >90% line coverage for core helpers and nodes.

---

## 5. Dependencies

- Depends on maintained ROS 2 packages and BehaviorTree.CPP, including: `rclcpp`, `rclcpp_action`, `behaviortree_cpp_v3`, `manymove_planner`, `manymove_object_manager`, `manymove_msgs`, `geometry_msgs`, `moveit_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `control_msgs`, `std_msgs`, `std_srvs`, `topic_based_ros2_control`, `simulation_interfaces`, `vision_msgs`.
- Uses stable APIs available in ROS 2 Humble and later.

Related ManyMove packages (same repository):
- `manymove_planner` – motion planning action servers used by action nodes.
- `manymove_object_manager` – planning-scene object management consumed by BT nodes.
- `manymove_hmi` – Qt HMI that visualizes `blackboard_status` and updates keys via `update_blackboard`.

---

## 6. Security and Robustness

- Inputs (BT port values, blackboard keys, and action responses) are validated where feasible.
- Asynchronous operations use timeouts and check action server availability.
- Future work: expand negative tests for malformed port values and unexpected action responses.

---

## 7. Known Limitations and Future Work

| Area                    | Current State                            | Future Improvement                                                  |
| ----------------------- | ---------------------------------------- | ------------------------------------------------------------------- |
| Integration tests       | Basic BT integration against fakes       | Add `launch_testing_ros` with planner/object manager in loop        |
| Documentation           | README + headers                         | Auto-generated API docs and end-to-end tutorials                    |
| Code coverage           | Unreported                               | Add gcovr/Codecov metrics                                           |
| API stability           | Pre-1.0                                  | Freeze public ports/keys and XML helper contracts                   |

---

## 8. Quality Level

| Criteria               | Level               | Notes                                          |
| ---------------------- | ------------------- | ---------------------------------------------- |
| Code style & linters   | ✓ Meets Level 3     | ament linters enforced                         |
| Tests (unit/component) | ✓ Meets Level 3     | broad GTest coverage                           |
| Documentation          | Partial             | examples exist; API docs pending               |
| Dependencies           | ✓ Meets Level 3     | maintained ROS 2 / BT.CPP deps                 |
| Integration & coverage | Pending             | more launch tests and coverage planned         |

Declared Quality Level: Level 3 (pre-release / community quality)

---

## 9. Contacts and Maintenance

- Maintainer: Marco Pastorio <pastoriomarco@gmail.com>
- Repository: https://github.com/pastoriomarco/manymove
- Support: issues via GitHub

---

Last updated: October 2025
