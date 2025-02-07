# Object Manager

![ROS2 Humble](https://img.shields.io/badge/ROS-2%20Humble-orange.svg)

A ROS 2 package that provides a unified way to add, remove, and check the existence of collision objects in a MoveIt 2 planning scene. Unlike other approaches that rely on the `/apply_planning_scene` service, **Object Manager** uses the default MoveIt 2 topics to publish collision objects, and it exposes **action servers** to handle sequential operations (such as verifying whether objects are truly added or removed in the scene).
This approach is meant to ease the use with the manymove packages: it is compatible with both implementations of manymove_planner (MoveGroup and MoveItCPP), and doen't access the planning scene directly to avoid possible racing conditions with the parallel planning and execution of the nodes in manymove_py_trees and manymove_cpp_trees.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
  - [Launching the Nodes](#launching-the-nodes)
  - [Interacting via Action Servers](#interacting-via-action-servers)
    - [AddCollisionObject](#addcollisionobject)
    - [RemoveCollisionObject](#removecollisionobject)
    - [CheckObjectExists](#checkobjectexists)
  - [Collision Spawner YAML Example](#collision-spawner-yaml-example)
- [Configuration](#configuration)
- [Design Choices (Beginner-Friendly)](#design-choices-beginner-friendly)
- [Dependencies](#dependencies)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Overview

The **Object Manager** package provides a robust and unified interface to:

1. **Add** collision objects to a MoveIt planning scene.
2. **Remove** collision objects from the planning scene.
3. **Check** whether a particular object currently exists in the scene.

This approach addresses a common challenge in MoveIt-based applications where different APIs and topics (`/apply_planning_scene`, direct publishing on topics, or MoveItCpp vs. MoveGroup) can complicate collision object management. Here, we rely solely on the default collision object topics, making it more generic and consistent across different MoveIt 2 usages.

---

## Features

- **Topic-Based Collision Object Updates**  
  Publishes collision objects to the default topic (`/collision_object`), avoiding reliance on `/apply_planning_scene`.

- **Action Servers**  
  Three action servers are provided:
  - `add_collision_object`  
  - `remove_collision_object`  
  - `check_object_exists`  

  By using actions, we can:
  - Provide real-time feedback (e.g., "collision object published.")
  - Perform sequential steps such as querying the planning scene to ensure success.
  - Support cancellations and timeouts gracefully.

- **Automatic Object Checking & Verification**  
  Each request to add or remove an object is followed by repeated queries to `/get_planning_scene` to verify the result, providing a more robust approach than “fire-and-forget.”

- **Collision Spawner**  
  An additional node (`collision_spawner`) that reads a YAML file, removes previously spawned objects, and spawns new ones with random or fixed poses. 
  While the object manager node simply provides the action servers, this spawner is meant to populate the scene during startup through the YAML config file.

---

## Architecture

### 1. `object_manager_node`

This node hosts the **action servers**:

1. **AddCollisionObject Action**  
   Publishes the object to the default topic. Then, repeatedly checks if the object is in the planning scene. If confirmed, returns success.

2. **RemoveCollisionObject Action**  
   Publishes a remove operation for a given object ID. Then, repeatedly checks if the object is truly gone from the scene. If confirmed, returns success.

3. **CheckObjectExists Action**  
   Submits a request to `/get_planning_scene` to verify the presence of a specific object ID.

Under the hood, `object_manager_node`:
- Publishes `moveit_msgs/msg/CollisionObject` to `/collision_object`.
- Queries the planning scene via the `/get_planning_scene` service.
- Implements a retry/backoff mechanism to robustly confirm that the scene is updated.

### 2. `collision_spawner`

This node reads a YAML configuration describing multiple objects to spawn. On startup:

1. Removes all existing “_spawner” objects (i.e., previously spawned objects), cleaning up or resetting the scene.
2. Iterates through the YAML list:
   - If a pose is provided, uses it.
   - Otherwise, generates a random pose within specified boundaries.
   - Sends an **AddCollisionObject** action goal to the `object_manager_node` to create each object in the scene.

When done, the node shuts itself down by default.

---

## Installation

1. **Clone the Repository**  
   ```bash
   cd ~/workspaces/ros2_ws/src
   git clone https://github.com/pastoriomarco/object_manager.git
   ```

2. **Install Dependencies**  
   Make sure you have the following installed:
   - **ROS 2 Humble** or later
   - **MoveIt 2** (matching your ROS 2 distribution)
   - **yaml-cpp** (usually available via `apt-get install ros-${ROS_DISTRO}-yaml-cpp`)
   - **tf2** and **tf2_geometry_msgs** for pose transformations

3. **Build the Package**  
   ```bash
   cd ~/workspaces/ros2_ws
   colcon build --packages-select object_manager
   source install/setup.bash
   ```

---

## Usage

### Launching the Nodes

1. **Object Manager Node Only**  
   This will start the node that hosts all the action servers.
   ```bash
   ros2 launch object_manager object_manager.launch.py
   ```
   - This launch file:
     - Starts `object_manager_node` with a default `frame_id` parameter set to `world`.

2. **Collision Spawner**  
   This will start the spawner node, which will read your YAML configuration and attempt to add collision objects.
   ```bash
   ros2 launch object_manager collision_spawner.launch.py
   ```
   - This launch file:
     - Runs the `collision_spawner` node.
     - Loads the YAML file from `object_manager/config/objects.yaml` by default.
     - Removes any “_spawner” objects already in the scene, then adds new ones.

### Interacting via Action Servers

Since this package uses **actions** instead of services to manage collision objects, you can interact with them using the following action names:

#### AddCollisionObject

- **Action Name:** `add_collision_object`
- **Goal Fields:**
  - `id`: Unique ID for the collision object (e.g., `"my_box"`).
  - `shape`: One of `["box", "cylinder", "sphere", "mesh"]`.
  - `dimensions`: Varies by shape.  
    - Box = `[x_size, y_size, z_size]`  
    - Cylinder = `[height, radius]`  
    - Sphere = `[radius]`  
    - Mesh = `[]` (dimensions not used for a mesh)
  - `pose`: A `geometry_msgs/Pose` for the object’s position/orientation.
  - `mesh_file`: A URI (e.g., `"package://object_manager/meshes/my_mesh.stl"`) if `shape` is `"mesh"`.

- **Result Fields:**
  - `success`: `true` if the object was verified to exist.
  - `message`: Additional info.

Example command-line usage (with the [ROS 2 action CLI](https://docs.ros.org/en/foxy/Tutorials/Intermediate/ROS2-Action-Server.html)):

```bash
# Example (pseudo-command) showing how you'd structure it:
ros2 action send_goal add_collision_object object_manager/action/AddCollisionObject "{id: 'my_box', shape: 'box', dimensions: [0.2, 0.1, 0.05], pose: { ... }, mesh_file: ''}"
```

#### RemoveCollisionObject

- **Action Name:** `remove_collision_object`
- **Goal Fields:**
  - `id`: The ID of the collision object to remove.
- **Result Fields:**
  - `success`: `true` if the object was verified to be removed from the scene.
  - `message`: Additional info.

#### CheckObjectExists

- **Action Name:** `check_object_exists`
- **Goal Fields:**
  - `object_id`: The ID to check in the planning scene.
- **Result Fields:**
  - `exists`: `true` if the object is found in the scene.
  - `message`: Additional info.

### Collision Spawner YAML Example

Below is a snippet from the default `objects.yaml`:

```yaml
objects:
  - name: "obstacle_ground"
    type: "box"
    dimensions: [0.8, 0.8, 0.1]
    pose:
      position: {x: 0.0, y: 0.0, z: -0.05}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}

  - name: "mesh_object"
    type: "mesh"
    mesh_file: "package://object_manager/meshes/pneumatic_lite.stl"
    pose:
      position: {x: 0.15, y: -0.05, z: 0.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
```

Each entry in `objects` can specify a shape type, dimensions (for primitive shapes), and optionally a pose. If no pose is provided, the spawner randomly generates one.

---

## Configuration

| Parameter Name | Type   | Default | Description                                                 |
|----------------|--------|---------|-------------------------------------------------------------|
| `frame_id`     | string | `"world"` | Base frame in which collision objects will be placed.        |
| `config_file`  | string | `""`    | Path to the YAML file used by the collision spawner.        |

Configure these parameters either via launch files or command line arguments (e.g., `ros2 launch object_manager collision_spawner.launch.py frame_id:=<your_frame>`).

---

## Design Choices (Beginner-Friendly)

1. **Why Topics Instead of `/apply_planning_scene`?**  
   - Some MoveIt configurations or older tutorials rely on `/apply_planning_scene`. However, certain versions or use cases (e.g., `MoveGroup` vs. `MoveItCpp`) might not expose the same services. By **directly publishing to the default collision object topic** (`/collision_object`), we ensure compatibility across multiple MoveIt usage patterns.

2. **Why Actions Instead of Services?**  
   - We need to **check** whether an object truly exists or has been removed from the planning scene, which can take time to reflect. Actions let us:
     - Provide **feedback** (e.g., “Attempt 1: object not found yet.”).
     - **Cancel** the request if needed.
     - **Retry** in the background until success or failure is confirmed.

3. **Why a Collision Spawner?**  
   - In many applications, we have a set of known environment objects or test objects that we want to spawn automatically. The `collision_spawner` node reads from a YAML file, removing old objects first, and adding new ones. This keeps your scene definition in a **single config file**.

4. **Why a `check_object_exists` Action?**  
   - While we could manually query `/get_planning_scene`, the action-based approach fosters a consistent interface. It also allows you to easily integrate existence checks into your pipelines or state machines without writing extra logic yourself.

5. **Random or Fixed Pose**  
   - If a pose is omitted in the YAML, we generate random positions within certain boundaries (e.g., `x` in [0.15, 0.3], `y` in [-0.25, 0.25], etc.). This is convenient for testing or simulation scenarios where you want objects to appear in slightly different locations each time.

---

## Dependencies

- **ROS 2 (tested on Humble)**  
- **MoveIt 2**  
- **yaml-cpp**  
- **tf2** and **tf2_geometry_msgs**  

Install them via your ROS package manager or system package manager as appropriate.

---

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

1. Fork the repository.
2. Create a feature branch.
3. Commit your changes.
4. Open a pull request describing your work.

---

## Acknowledgements

- [MoveIt 2](https://moveit.ros.org/) community for their extensive planning scene documentation.
- ROS 2 community for building a flexible ecosystem that supports advanced features like actions and multi-threaded executors.

---

Happy building and planning with **Object Manager**!

