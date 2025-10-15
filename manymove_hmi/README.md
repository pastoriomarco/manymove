# manymove_hmi

`manymove_hmi` provides the Qt-based human-machine interface used to supervise and steer the ManyMove behaviour tree deployments. The package runs a desktop GUI while bridging to ROS 2 so operators can start, stop, reset, and monitor behaviour tree execution across one or more robots.

For installation instructions and the overall project context, see the root [`manymove` README](../README.md).

## Overview
- Qt `QMainWindow` presents one control row per robot with start/stop/reset buttons plus textual status.
- Each row connects to a dedicated ROS 2 worker node that mirrors the behaviour tree blackboard state and sends service requests back to the execution system.
- Optional application widgets (built on `AppModule`) show live blackboard values and allow curated edits (for example tube dimensions) that are written via the `manymove_msgs/SetBlackboardValues` service.
- A lightweight TCP broadcaster shares the latest GUI status JSON on port `5000` for non-ROS dashboards.

## Components
- `Ros2Worker` (`include/manymove_hmi/ros2_worker.hpp`, `src/ros2_worker.cpp`): An `rclcpp::Node` spun in its own thread per robot prefix. It subscribes to `blackboard_status` (`std_msgs/msg/String`) and dispatches GUI updates via `QMetaObject::invokeMethod`. It also creates a client for the `update_blackboard` service, exposes helpers to assert `stop_execution`, `reset`, or resume execution, and synchronises any attached `AppModule` by iterating over its declared blackboard keys.
- `HmiGui` (`include/manymove_hmi/hmi_gui.hpp`, `src/hmi_gui.cpp`): The Qt window layer that builds button rows, displays per-robot messages, and emits `startExecutionRequested`, `stopExecutionRequested`, and `resetProgramRequested` signals. It keeps the window on top and hosts the optional TCP publisher.
- `AppModule` (`include/manymove_hmi/app_module.hpp`, `src/app_module.cpp`): A configurable widget engine that renders a list of blackboard keys, tracks editable overrides, and emits `keyUpdateRequested` whenever the operator presses SEND or toggles certain controls.
- `DefaultAppModule` (`include/manymove_hmi/default_app_hmi.hpp`, `src/default_app_hmi.cpp`): The canned configuration shipped with ManyMove. It exposes tube geometry fields (`tube_length_key`, `tube_diameter_key`, etc.), auto-computes dependent vectors and poses, and keeps those values in sync with the blackboard.
- `main.cpp`: Launches the base HMI (robot controls only). It fetches `robot_prefixes` and `robot_names` parameters, instantiates `HmiGui`, spawns one `Ros2Worker` per prefix, and spins each worker on a background thread.
- `main_app.cpp`: Builds on the above by inserting a `DefaultAppModule` into the GUI layout and wiring its `keyUpdateRequested` signal to the `update_blackboard` client so curated edits propagate to the behaviour tree blackboard.

## ROS 2 Interfaces
- **Subscriptions:** `blackboard_status` (`std_msgs/msg/String`) — JSON-formatted snapshots of the behaviour tree blackboard. Keys such as `<prefix>stop_execution`, `<prefix>reset`, `<prefix>collision_detected`, `<prefix>message`, and `hmi_message` drive GUI button states and labels.
- **Service clients:** `update_blackboard` (`manymove_msgs/srv/SetBlackboardValues`). Called whenever a GUI button is pressed (start/stop/reset) or when `AppModule` submits overrides. Requests contain parallel `key`, `value_type`, and `value_data` arrays matching the behaviour tree blackboard schema.
- **Parameters:** `robot_prefixes` (list of string prefixes for each robot namespace) and `robot_names` (display names). Both executables declare and read these parameters on startup; the list lengths must match.

## Usage
- For workspace setup, dependencies, and launch instructions, follow the top-level [ManyMove README](../README.md).
- The package installs two ROS 2 executables:
  - `manymove_hmi_executable` (plain robot control window).
  - `manymove_hmi_app_executable` (robot controls plus `DefaultAppModule` panel).
- Example manual launch:
  ```bash
  ros2 run manymove_hmi manymove_hmi_app_executable \
    --ros-args -p robot_prefixes:="[\'ur10_\', \'xarm7_\']" \
               -p robot_names:="[\'UR10\', \'xArm7\']"
  ```
  Provide the prefixes with trailing separators to match the keys published in `blackboard_status`.
- In normal operation the HMI is spawned automatically by the bringup launch files under `manymove_bringup/launch`. For instance, `lite_movegroup_fake_py_trees.launch.py` starts `manymove_hmi_executable` alongside the rest of the stack, so manual invocation is rarely required.
- From the GUI:
  - START clears `<prefix>stop_execution` through `SetBlackboardValues`, allowing the behaviour tree to resume.
  - STOP sets `<prefix>stop_execution` to true, which pauses or aborts execution.
  - RESET sets both `<prefix>stop_execution` and `<prefix>reset` to true.
  - The message area updates from `<prefix>message`/`<prefix>message_color`, while the lower panel (if enabled) mirrors arbitrary blackboard keys such as `tube_length_key`.
- If the optional TCP port 5000 connection is used, the last JSON payload sent to the GUI is broadcast to any connected socket client.

## Dependencies
- Runtime ROS 2 dependencies: `rclcpp`, `std_msgs`, `std_srvs`, and `manymove_msgs`.
- Qt 5 modules: `Qt5::Core`, `Qt5::Widgets`, and `Qt5::Network`. The Debian dependency is `qtbase5-dev` for building.
- Build system: `ament_cmake` with Qt automatic MOC/UIC/RCC enabled. Ensure the workspace is configured per the main project instructions before running `colcon build`.

## License and Maintainers
This package is licensed under BSD-3-Clause. Maintainer: Marco Pastorio <pastoriomarco@gmail.com>.
See main [ManyMove README](../README.md) for `CONTRIBUTION` details.
