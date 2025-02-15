# ManyMove HMI

This package is part of the [`manymove`](../README.md) project for **ROS 2** Humble. It provides a **Qt-based** GUI and corresponding ROS 2 node(s) to interact with `manymove` robots, enabling start/stop/reset commands and displaying collision or execution status.

**Use at your own risk**; this repository is experimental and does **not** include safety features.

---

## Overview

### Key Functionalities

- **Qt GUI**  
  A simple interface (`HmiGui`) that shows one row of controls (start/stop/reset buttons) and a collision indicator LED per robot.

- **ROS 2 Worker Nodes**  
  Each robot prefix spawns a `Ros2Worker` node that subscribes to Blackboard status and calls the relevant start/stop/reset services.

- **Status Visualization**  
  Subscribes to `<robot_prefix>blackboard_status` to display collision detection, mission abort flags, or paused states in real time.

- **TCP Broadcast**  
  Optionally publishes the GUI status (JSON) via a TCP server for external clients.

---

## Architecture

1. **`hmi_gui.hpp` / `hmi_gui.cpp`**  
   - Implements the **Qt GUI**, creating a row of buttons and an indicator per robot prefix.
   - Binds each button to signals that trigger start/stop/reset operations.

2. **`ros2_worker.hpp` / `ros2_worker.cpp`**  
   - A minimal **ROS 2 node** that listens for the Blackboard status topic (`<robot_prefix>blackboard_status`).
   - Calls the appropriate services (`start_execution`, `stop_execution`, `reset_program`) with a 1-second wait.

3. **`main.cpp`**  
   - Launches both the Qt application and multiple `Ros2Worker` instances (one per robot prefix).
   - Uses `rclcpp::init` and a `QApplication`.
   - Manages a multi-threaded approach, spinning each `Ros2Worker` in its own thread.

---

## Installation & Dependencies

Follow the main [**ManyMove README**](../README.md) for overall setup of ROS 2 and Qt dependencies.

## Usage

- **Launch the HMI**  
  ```bash
  ros2 run manymove_hmi manymove_hmi_executable
  ```
  By default, it reads a `robot_prefixes` parameter (vector of strings) to create a row of controls for each prefix.

- **Start/Stop/Reset**  
  - Press START: calls `<robot_prefix>start_execution`.
  - Press STOP: calls `<robot_prefix>stop_execution` and updates button states.
  - Press RESET: calls `<robot_prefix>reset_program`.

- **Monitoring**  
  The GUI automatically updates button states and collision LED when it receives updates from `<robot_prefix>blackboard_status`.

---

## Notes & Disclaimer

- **Experimental**: This package is under development; no guarantee of stability.
- **No Safety Features**: Robot safety must be managed externally.
- **Check Main README**: For licensing, disclaimers, and more details.

