# Quality Declaration – `manymove_hmi`

This document declares the quality level for the `manymove_hmi` package of the ManyMove project.

## 1. Summary

`manymove_hmi` provides the Qt-based Human–Machine Interface used to supervise and steer ManyMove behavior-tree deployments. It renders per‑robot controls (start/stop/reset), displays status/messages, and optionally exposes an application panel (`AppModule`) to view and edit curated blackboard keys. A `Ros2Worker` node bridges the GUI to ROS 2 topics/services.

The package follows ROS 2 and Qt coding standards and is actively maintained within the ManyMove repository (`dev`/`main` branch).

---

## 2. Version Policy

- Versioning: Semantic versioning (MAJOR.MINOR.PATCH) is followed. Current version: `0.3.1`.
- Change control: All changes are proposed via pull requests and reviewed on GitHub; automated CI builds validate the workspace.
- API stability: Public APIs (Qt widgets/signals, expected blackboard keys handled by the default app module) are pre‑1.0 and may change until `1.0.0`.

---

## 3. Documentation

- The README describes the GUI components, ROS 2 interfaces, and runtime parameters.
- Public headers document `HmiGui`, `Ros2Worker`, and `AppModule` where practical.
- Example executables show a minimal control window and a version including the default application panel.
- Future work: add Doxygen-generated API docs and richer usage examples.

---

## 4. Testing

### 4.1 Unit and Component Tests

- GTest-based tests are provided and cover:
  - `HmiGui` button state logic and message styling.
  - `AppModule` validation, scaling, and emission of `keyUpdateRequested`.
- Tests are Qt‑headless via `QT_QPA_PLATFORM=offscreen` and drive widgets through public APIs.
- Run with:
  ```bash
  colcon test --packages-select manymove_hmi
  ```

### 4.2 Linters and Static Analysis

- Enforced via `ament_lint_auto` and `ament_lint_common` with `ament_cmake_uncrustify` configured.
- Typical checks include cpplint/cppcheck/xmllint/uncrustify where available.

### 4.3 Continuous Integration

- GitHub Actions builds the workspace on every push and pull request (Humble and Jazzy jobs).
- CI runs `colcon build`, `colcon test`, and linters for all packages.

### 4.4 Code Coverage

- No formal coverage reports yet. Target: integrate `gcovr`/Codecov and aim for >90% line coverage for GUI logic and the worker’s message parsing.

---

## 5. Dependencies

- Depends on maintained ROS 2 packages and Qt modules, including: `rclcpp`, `std_msgs`, `std_srvs`, `manymove_msgs`, and Qt5 `Core/Widgets/Network`.
- Uses stable APIs available in ROS 2 Humble and later.

Related ManyMove packages (consumers/producers in the same repository):
- `manymove_cpp_trees` – behavior-tree nodes that publish `blackboard_status` and consume `update_blackboard` updates.
- `manymove_planner` – motion planning action servers orchestrated by the BT.
- `manymove_object_manager` – planning-scene object management accessed from BT action nodes.

---

## 6. Security and Robustness

- Inputs (JSON in `blackboard_status`, GUI edits) are parsed/validated conservatively; malformed numeric values are ignored.
- Service clients are created with timeouts and tolerate transient unavailability.
- Future work: expand negative tests for malformed blackboard payloads and GUI edge cases.

---

## 7. Known Limitations and Future Work

| Area                    | Current State                             | Future Improvement                                                  |
| ----------------------- | ----------------------------------------- | ------------------------------------------------------------------- |
| Integration tests       | Unit tests for GUI/logic only             | Add `launch_testing_ros` with a fake BT publisher and service       |
| Documentation           | README + headers                          | Auto-generated API docs and step-by-step usage examples             |
| Code coverage           | Unreported                                | Add gcovr/Codecov metrics                                           |
| API stability           | Pre-1.0                                   | Freeze public widget signals/parameters post-1.0                    |

---

## 8. Quality Level

| Criteria               | Level               | Notes                                          |
| ---------------------- | ------------------- | ---------------------------------------------- |
| Code style & linters   | ✓ Meets Level 3     | ament linters enforced                         |
| Tests (unit/component) | ✓ Meets Level 3     | GUI + widget engine tests                      |
| Documentation          | Partial             | API docs pending                               |
| Dependencies           | ✓ Meets Level 3     | maintained ROS 2 / Qt deps                     |
| Integration & coverage | Pending             | more launch tests and coverage planned         |

Declared Quality Level: Level 3 (pre-release / community quality)

---

## 9. Contacts and Maintenance

- Maintainer: Marco Pastorio <pastoriomarco@gmail.com>
- Repository: https://github.com/pastoriomarco/manymove
- Support: issues via GitHub

---

Last updated: October 2025
