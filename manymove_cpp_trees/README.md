# ManyMove C++ Trees

This **manymove_cpp_trees** package is part of the [**`manymove`**](../README.md) project for **ROS 2** Humble. It offers a **BehaviorTree.CPP**-based framework for composing motion sequences, object manipulation, and I/O signal handling, using the action servers and messages defined throughout `manymove` packages.

**Use at your own risk**; this repository is experimental and does **not** provide safety features.

---

## Overview

### Key Functionalities

- **BehaviorTree.CPP Integration**  
  Provides a set of **custom BT nodes** (planner, object manipulation, signals, etc.) that can be used to build modular robotic behaviors.

- **MoveIt 2 & Action Server Bridges**  
  Easily plan and execute motions, manage collision objects, attach/detach objects, and send/receive I/O signals by leveraging the action servers in `manymove_planner` and `manymove_object_manager`.

- **Reconfigurable at Runtime**  
  Much of the tree logic (target poses, velocities, attach operations, etc.) is dynamically drawn from the Blackboard, allowing for flexible updates without recompiling.

- **HMI Service Node**  
  A dedicated node that exposes services for controlling behavior execution (start, stop, reset) and publishes Blackboard status.

---

## Architecture

1. **Behavior Tree Nodes**  
   - **`action_nodes_planner.hpp`**: Nodes for planning/executing manipulator motions.  
   - **`action_nodes_objects.hpp`**: Nodes for adding/removing collision objects and checking or attaching them.  
   - **`action_nodes_signals.hpp`**: Nodes for sending/reading digital I/O signals and checking robot state.  
   - **`action_nodes_logic.hpp`**: Custom decorator/condition nodes for controlling execution flow (e.g., pausing, aborting, blackboard key checks).

2. **bt_client_node**  
   - A reference implementation (in [`bt_client.cpp`](./src/bt_client.cpp)) that programmatically constructs a complex Behavior Tree.  
   - Demonstrates combining the above custom BT nodes (e.g., for scanning the environment, picking/dropping objects, checking signals, etc.).  
   - Uses a **MultiThreadedExecutor** to spin the tree logic in parallel with the HMI service node.

3. **HMI Service Node**  
   - [`HMIServiceNode`](./include/manymove_cpp_trees/hmi_service_node.hpp) provides services to start/stop/reset execution.  
   - Publishes execution status (e.g., `stop_execution`, `reset`, etc.) at a fixed interval.

---

## Installation & Dependencies

Please refer to the main [**ManyMove README**](../README.md) for overall setup instructions, build steps, and prerequisites.

### Usage

#### Running the bt_client_node

The bt_client_node is an example node that programmatically creates a Behavior Tree:

```bash
ros2 run manymove_cpp_trees bt_client_node
```

Parameters (can be set via the command line or a launch file):
- `robot_model` (default: "lite6")
- `robot_prefix` (default: "")
- `tcp_frame` (default: "")
- `is_robot_real` (default: false), determines if real I/O signals are used or mocked.

#### Defining Your Own Trees

- Include the desired node headers (action_nodes_*) in your custom .cpp.
- Register them with BehaviorTree.CPP (see tree_helper.hpp and action_nodes_planner.cpp for examples).
- Construct an XML tree (or build it programmatically) referencing these node types.
- Create a BT::Tree from the factory and tick it in a loop or an executor.

### Example Features

- **Motion Planning & Execution**  
  `MoveManipulatorAction` node for MoveIt 2-based motion.

- **Collision Object Management**  
  `AddCollisionObjectAction`, `RemoveCollisionObjectAction`, `AttachDetachObjectAction`, etc.

- **I/O Signals & Robot State**  
  `SetOutputAction`, `GetInputAction`, `CheckRobotStateAction`, and so on.

---

### Notes & Disclaimer

- **Experimental**: This package is under development; no warranty of stability.
- **No Safety Features**: Robot safety must be handled externally.
- **Check Main README**: For detailed disclaimers, licensing, and more information.

### Contributing

Feedback and pull requests are welcome. If you discover any issues or have suggestions, please open an issue or PR in the main manymove repository.

