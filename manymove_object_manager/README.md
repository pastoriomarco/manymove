# ManyMove Object Manager

This package is part of the [**`manymove`**](../README.md) project for **ROS 2** Humble. It provides a **unified, topic-based** approach for managing collision objects within MoveIt 2. **Use at your own risk**; the repository is experimental and does **not** include safety features.

This package allows to add, remove, and check the existence of an object, and also to attach and detach object to a certain link and to obtain the pose of an object relative to a link. 

Unlike other approaches that rely on the `/apply_planning_scene` service, **Object Manager** uses the default MoveIt 2 topics to publish collision objects, and it exposes **action servers** to handle sequential operations (such as verifying whether objects are truly added or removed in the scene).

This approach is meant to ease the use with the manymove packages: it is compatible with both implementations of manymove_planner (MoveGroup and MoveItCPP), and doen't access the planning scene directly to avoid possible racing conditions with the parallel planning and execution of the nodes in `manymove_py_trees` and `manymove_cpp_trees`.

---

## Overview

### Key Functionalities

- **Topic-Based Collision Updates**
  Publishes collision objects (add/remove) to MoveIt 2’s default `/collision_object` and `/attached_collision_object` topics; no direct calls to `/apply_planning_scene`.

- **Action Interfaces**
  Offers multiple **action servers** for:
  - `/add_collision_object`: Adds a collision object, verifying presence by querying `/get_planning_scene`.
  - `/remove_collision_object`: Publishes a remove operation and confirms its success.
  - `/check_object_exists`: Checks if an object is present or attached.
  - `/attach_detach_object`: Attaches/detaches an object to/from a specified link.
  - `/get_object_pose`: Retrieves an object’s pose relative to a link with optional transformations.

- **Sequential Verification**
  Retries actions until the planning scene is updated, mitigating race conditions.

- **Collision Spawner**
  Reads a YAML file to spawn or remove multiple objects at once, simplifying environment setup.

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

## Installation & Dependencies

Please refer to the main [**ManyMove README**](../README.md) for overall setup instructions, build steps, and prerequisites.

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

---

## Design Choices (Beginner-Friendly)

1. **Why Topics Instead of `/apply_planning_scene`?**  
   - Some MoveIt configurations or tutorials rely on `/apply_planning_scene`. However, certain versions or use cases (e.g., `MoveGroup` vs. `MoveItCpp`) might not expose the same services. By **directly publishing to the default collision object topic** (`/collision_object`), we ensure compatibility across multiple MoveIt usage patterns.

2. **Why Actions Instead of Services?**  
   - We need to **check** whether an object truly exists or has been removed from the planning scene, which can take time to reflect. Actions let us:
     - Provide **feedback** (e.g., “Attempt 1: object not found yet.”).
     - **Cancel** the request if needed.
     - **Retry** in the background until success or failure is confirmed.

3. **Why a Collision Spawner?**  
   - In many applications, we have a set of known environment objects or test objects that we want to spawn automatically. The `collision_spawner` node reads from a YAML file, removing old objects first, and adding new ones. This keeps your scene definition in a **single config file**. 

5. **Random or Fixed Pose**  
   - If a pose is omitted in the YAML, we generate random positions within certain boundaries (e.g., `x` in [0.15, 0.3], `y` in [-0.25, 0.25], etc.). This is convenient for testing or simulation scenarios where you want objects to appear in slightly different locations each time.

---

## Notes & Disclaimer
- **Experimental**: No safety features are provided.
- **Compatibility**: Works with MoveGroup and MoveItCpp in `manymove_planner`.
- **Check Main README** for disclaimers, licensing, and more.
