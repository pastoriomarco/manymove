# Quality Declaration – `manymove_planner`

This document declares the quality level for the `manymove_planner` package of the ManyMove project.

## 1. Summary

`manymove_planner` provides motion-planning action servers that wrap MoveIt 2, exposing consistent ROS 2 actions for planning and executing manipulator motions. It supports multiple planner backends (`MoveGroupInterface` and `MoveItCpp`), controller load/unload, and controlled stops.

The package follows ROS 2 and MoveIt coding standards and is actively maintained within the ManyMove repository (`dev`/`main` branch).

---

## 2. Version Policy

- Versioning: Semantic versioning (MAJOR.MINOR.PATCH) is followed. Current version: `0.3.2`.
- Change control: All changes are reviewed via GitHub pull requests; CI validates builds and tests.
- API stability: Public APIs (headers, actions, and parameters) are pre-1.0 and may change until `1.0.0`.

---

## 3. Documentation

- The README describes architecture, planner backends, executables, and dependencies.
- Public headers outline the `PlannerInterface`, `ManipulatorActionServer`, and nodes.
- Example launch files provide typical setups for fake hardware, real controllers, and multi-robot deployments.
- Future work: add auto-generated API reference and user scenarios.

---

## 4. Testing

### 4.1 Unit and Component Tests

- GTest-based tests exercise the `ManipulatorActionServer` using fakes for:
  - `controller_manager` services (load/configure/switch/unload).
  - `control_msgs/FollowJointTrajectory` action server (success, cancel/hold, and failure paths).
  - Planner interface behaviors (plan validity, controlled stop, trajectory validation).
- Tests cover goal handling, success/error propagation, and controller lifecycle flows.
- Run with:
  ```bash
  colcon test --packages-select manymove_planner
  ```

### 4.2 Linters and Static Analysis

- Linters enabled via `ament_lint_auto` and `ament_lint_common` with `ament_cmake_uncrustify` configured when available.
- The CMake logic gracefully skips linting if `ament_cmake_test` is unavailable in the environment.
- Core concurrency hotspots (joint-state caches, action execution state, planner feedback) are annotated with `rcpputils` thread-safety macros, but the default CI path still uses GCC/libstdc++; therefore the thread-safety analysis is not enforced automatically yet.

### 4.3 Continuous Integration

- GitHub Actions builds and tests on Humble and Jazzy for pushes and PRs.
- CI runs `colcon build`, `colcon test`, and linters across the workspace.

### 4.4 Code Coverage

- No formal coverage reporting yet. Target: integrate `gcovr`/Codecov and set coverage goals for core servers and backends.

---

## 5. Dependencies

- Depends on maintained ROS 2 and MoveIt packages, including: `rclcpp`, `rclcpp_action`, `action_msgs`, `moveit_core`, `moveit_ros_planning_interface`, `moveit_msgs`, `tf2_geometry_msgs`, `geometry_msgs`, `control_msgs`, `controller_manager_msgs`, and `manymove_msgs`.
- Uses API surfaces stable in ROS 2 Humble and later.

Related ManyMove packages (same repository):
- `manymove_cpp_trees` – behavior-tree clients that drive these action servers.
- `manymove_object_manager` – collision object management used alongside planning.
- `manymove_hmi` – operator GUI supervising execution and sending control commands.

---

## 6. Security and Robustness

- Validates inputs, checks controller/action availability, and uses timeouts for async operations.
- Defensive checks around planning validity and trajectory execution; controlled stop is exposed and tested.
- Future work: add fuzz/negative tests for malformed goals and controller failures.

---

## 7. Known Limitations and Future Work

| Area                    | Current State                                | Future Improvement                                                  |
| ----------------------- | -------------------------------------------- | ------------------------------------------------------------------- |
| Integration tests       | Unit/component tests against fakes           | Add `launch_testing_ros` with MoveIt + real controllers in loop     |
| Documentation           | README + headers                             | Auto-generated API docs and practical deployment recipes            |
| Code coverage           | Unreported                                   | Add gcovr/Codecov metrics                                           |
| Thread-safety analysis  | Annotations present; clang/libc++ build optional | Add dedicated clang/libc++ CI job to run `-Wthread-safety` checks   |
| API stability           | Pre-1.0                                      | Freeze public actions/params post-1.0                               |

---

## 8. Quality Level

| Criteria               | Level               | Notes                                          |
| ---------------------- | ------------------- | ---------------------------------------------- |
| Code style & linters   | ✓ Meets Level 3     | ament linters enforced (when available)        |
| Tests (unit/component) | ✓ Meets Level 3     | comprehensive server and backend tests         |
| Documentation          | Partial             | API reference pending                          |
| Dependencies           | ✓ Meets Level 3     | maintained ROS 2 / MoveIt deps                 |
| Integration & coverage | Pending             | more launch tests and coverage planned         |

Declared Quality Level: Level 3 (pre-release / community quality)

---

## 9. Contacts and Maintenance

- Maintainer: Marco Pastorio <pastoriomarco@gmail.com>
- Repository: https://github.com/pastoriomarco/manymove
- Support: issues via GitHub

---

Last updated: October 2025
