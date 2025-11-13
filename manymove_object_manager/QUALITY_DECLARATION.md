# Quality Declaration – `manymove_object_manager`

This document declares the quality level for the `manymove_object_manager` package of the ManyMove project.

## 1. Summary

`manymove_object_manager` provides the node and library responsible for adding, removing, attaching, detaching, and querying collision objects in a MoveIt planning scene.
It exposes these capabilities as ROS 2 actions and services, and provides a YAML-driven configuration for object definitions and a standalone collision-spawner utility.

The package follows ROS 2 and MoveIt coding standards and is actively maintained within the **ManyMove** repository (`dev`/`main` branch).

---

## 2. Version Policy

- **Versioning:** Semantic versioning (MAJOR.MINOR.PATCH) will be followed.
  Current version: `0.3.1`.
- **Change control:** All changes are reviewed through pull requests on GitHub and are verified by automated builds in the CI workflow.
- **API stability:** Public API (headers, actions, and parameters) is still in early development and may change until `1.0.0`.

---

## 3. Documentation

- The package includes a README describing purpose, actions, configuration files, and dependencies.
- Each public header has docstrings documenting the interfaces and node behavior.
- Example launch files (`object_manager.launch.py`, `collision_spawner.launch.py`) are provided.
- Further improvement planned: add Doxygen/Sphinx auto-generated documentation and user examples.

---

## 4. Testing

### 4.1 Unit and Component Tests
- The package provides comprehensive **GTest** coverage:
  - Tests cover add/remove/attach/detach/get-pose actions.
  - A **FakePlanningSceneServer** mock simulates MoveIt’s `/get_planning_scene` service.
  - Tests validate error handling (duplicate IDs, non-existent objects, service failures).
  - Execution uses a multi-threaded `rclcpp::Executor` and verifies published messages.
- Executed automatically when `BUILD_TESTING=ON` via:
  ```bash
  colcon test --packages-select manymove_object_manager
  ```

### 4.2 Linters and Static Analysis

* Enforces:

  * `ament_lint_auto`
  * `ament_lint_common`
  * `ament_uncrustify` with project-wide configuration
  * `ament_cpplint`
* Checked both locally (via pre-commit) and in CI.

### 4.3 Continuous Integration

* GitHub Actions build the full workspace on every push and pull request.
* CI runs `colcon build`, `colcon test`, and lint checks across all packages.

### 4.4 Code Coverage

* No formal coverage reports yet.
  Target for future: integrate `gcovr`/`codecov` reporting and aim for >90 % line coverage.

---

## 5. Dependencies

* Depends on stable, quality-level ROS 2 packages:
  `rclcpp`, `rclcpp_action`, `geometry_msgs`, `shape_msgs`, `moveit_ros_planning_interface`, `tf2_ros`, etc.
* Uses only API surfaces considered stable within ROS 2 Humble and later.

Related ManyMove packages (same repository):
* `manymove_cpp_trees` – behavior-tree nodes that call object manager actions/services.
* `manymove_planner` – planning backends that consume the maintained planning scene.
* `manymove_hmi` – GUI that surfaces object-related statuses via the BT blackboard.

---

## 6. Security and Robustness

* All external inputs (YAML configs, action goals) are validated.
* Node retries service connections and handles `ServiceNotAvailable` conditions.
* Future improvement: add fuzz tests for malformed YAMLs and invalid meshes.

---

## 7. Known Limitations and Future Work

| Area                    | Current State                            | Future Improvement                                                  |
| ----------------------- | ---------------------------------------- | ------------------------------------------------------------------- |
| **Integration tests**   | Only unit tests with mock planning scene | Add `launch_testing_ros` integration tests with a real MoveIt scene |
| **Quality declaration** | Present (this file)                      | Align with REP-2004 once stable APIs are frozen                     |
| **Documentation**       | Minimal README                           | Auto-generated API docs, richer examples                            |
| **Code coverage**       | Unreported                               | Add gcovr/Codecov metrics                                           |
| **Versioning**          | 0.3.1                                    | Increment per semantic versioning and maintain CHANGELOG.rst        |

---

## 8. Quality Level

| Criteria               | Level               | Notes                                          |
| ---------------------- | ------------------- | ---------------------------------------------- |
| Code style & linters   | **✓ Meets Level 3** | ament linters enforced                         |
| Tests (unit)           | **✓ Meets Level 3** | extensive component tests                      |
| Documentation          | **Partial**         | needs examples and API reference               |
| Dependencies           | **✓ Meets Level 3** | all dependencies are maintained ROS 2 packages |
| Integration & coverage | **Pending**         | integration tests planned                      |

**Declared Quality Level: *Level 3 (pre-release / community quality)***

---

## 9. Contacts and Maintenance

* **Maintainer:** Marco Pastorio [pastoriomarco@gmail.com](mailto:pastoriomarco@gmail.com)
* **Repository:** [https://github.com/pastoriomarco/manymove](https://github.com/pastoriomarco/manymove)
* **Support:** issues via GitHub

---

*Last updated: October 2025*
