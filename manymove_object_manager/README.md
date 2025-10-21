# ManyMove Object Manager

## Overview
ManyMove Object Manager provides a ROS 2 node that manages collision objects in a MoveIt planning scene for the ManyMove stack. It exposes action servers to add, remove, attach, detach, and query objects, confirming every change through the MoveIt `GetPlanningScene` service. The node publishes `moveit_msgs/msg/CollisionObject` and `moveit_msgs/msg/AttachedCollisionObject` messages so downstream planners stay synchronized.

## Actions
- `AddCollisionObject` – Adds a collision object by publishing to the planning scene topics and waits until the new object is reported by `GetPlanningScene`.
- `RemoveCollisionObject` – Removes a collision object and checks the planning scene until the object disappears.
- `AttachDetachObject` – Attaches or detaches an object to a link, keeping the attached collision object topic in sync.
- `CheckObjectExists` – Queries the planning scene to confirm whether an object is present or attached.
- `GetObjectPose` – Retrieves an object's pose, optionally aligning it to a target frame using TF2 transforms.

Every action uses the MoveIt `GetPlanningScene` service for verification and leads to collision or attached object updates being republished.

## Launch
- For workspace setup, dependencies, and launch instructions, follow the top-level [ManyMove README](../README.md).

## Interactions and Dependencies
Action definitions come from `manymove_msgs`, and message types are provided by `moveit_msgs`, `geometry_msgs`, `shape_msgs`, and the TF2 stack. The node is designed to cooperate with `manymove_planner`, and `manymove_cpp_trees`, giving these planners a consistent interface for collision management without direct planning scene manipulation.

## Configuration
The file `config/objects.yaml` contains example obstacles and graspable objects that can be pre-loaded into the planning scene by companion tools such as a collision spawner. Customize this list to describe application-specific geometry, including mesh resources under `meshes/`, before invoking the object management actions.

## License and Maintainers
This package is licensed under BSD-3-Clause. Maintainer: Marco Pastorio <pastoriomarco@gmail.com>.
See main [ManyMove README](../README.md) for `CONTRIBUTION` details.
