# ManyMove Signals

This **`manymove_signals`** package is part of the [**`manymove`**](../README.md) project for **ROS 2** Humble. It provides a **unified signals interface** (digital I/O, robot state checks, and robot reset logic) for various robot models, using xArm or UFactory-like services and topics.

**Use at your own risk**; this repository is experimental and does **not** provide safety features.

---

## Overview

### Key Functionalities

- **Digital I/O**  
  Action servers to set or get I/O pins (`SetOutput`, `GetInput`) for both *tool* and *controller* IO types.
- **Robot State Check & Reset**  
  Action servers to check if the robot is ready (`CheckRobotState`) and to reset it if needed (`ResetRobotState`), wrapping lower-level services (`clean_error`, `motion_enable`, `set_mode`, etc.).
- **Multi-Model Support**  
  Adjusts namespace and service calls depending on the `robot_model` parameter (`lite6`, `uf850`, or `xarm`).

---

## Architecture

1. **`signals_node.cpp`**  
   - The main node (`manymove_signals_node`) sets up:
     - **Service Clients**: `set_tgpio_digital`, `get_tgpio_digital`, `set_cgpio_digital`, `get_cgpio_digital`, plus xArm’s `set_mode`, `set_state`, `clean_error`, `motion_enable`.
     - **Action Servers**: `set_output`, `get_input`, `reset_robot_state`, `check_robot_state`.
     - **Subscription**: E.g., `<robot_prefix>ufactory/robot_states` or `<robot_prefix>xarm/robot_states` for continuous monitoring.
   - Each action server delegates to the relevant service calls to handle the requested logic.

2. **Reset Robot Flow**  
   - Cleans errors, re-enables motion, sets mode/state, and verifies the result by checking the subscription to `robot_states`.

3. **Check Robot State**  
   - Reads the last known `robot_states` message, checking for `err == 0`, `mode == 1`, etc.

---

## Installation & Dependencies

Follow the main [**ManyMove README**](../README.md).

---

## Usage

### Run the Signals Node

```bash
ros2 run manymove_signals manymove_signals_node
```

**Parameters**:
- `robot_model`: e.g., lite6, uf850, or xarm
- `robot_prefix`: optional namespace prefix (e.g., R_)

### Action Servers
- `<robot_prefix>set_output`: Takes an io_type (tool or controller), ionum, and value (0/1).
- `<robot_prefix>get_input`: Returns the digital input for a given pin.
- `<robot_prefix>reset_robot_state`: Cleans error, re-enables motion, sets mode/state, and checks the robot’s readiness.
- `<robot_prefix>check_robot_state`: Returns current error, mode, and state, plus a boolean for “ready.”

### Compatibility
Typically used by `manymove_planner` or `manymove_cpp_trees` for logic that requires direct I/O or resetting the robot at runtime.

---

## Notes & Disclaimer

- **Experimental**: This package is under active development.
- **No Safety Features**: Always configure your hardware’s safety mechanisms separately.
- Check Main README for disclaimers, licensing, and additional info.

