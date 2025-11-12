========================
ManyMove – Changelog
========================

This is the repository-level changelog that summarizes notable changes across all packages.
Per-package changelogs may still be generated during release, but this file is the canonical
high-level history.

Forthcoming
-----------

- Decouples `manymove_msgs` from MoveIt by switching the `PlanManipulator`/`MoveManipulator`
  actions to `trajectory_msgs/JointTrajectory` so downstream packages no longer pull in
  `moveit_msgs` indirectly.
- Updates `manymove_planner` to accept the slimmer action interface while still using
  `moveit_msgs::msg::RobotTrajectory` internally, ensuring existing planners keep their MoveIt
  features without leaking the dependency to callers.
- Refreshes `manymove_cpp_trees` to store and exchange `trajectory_msgs::msg::JointTrajectory`
  objects on the blackboard, removes MoveIt includes from the BehaviorTree nodes/tests, and
  tightens package dependencies accordingly.

0.2.2 (2025-11-07)
------------------

Summary
^^^^^^^
- Hardens the MoveIt planners with richer trajectory scoring, diagnostics, and multi-turn joint safeguards.
- Refreshes the Universal Robots launchers and configs so MoveGroup/MoveItCpp demos share the same pipeline defaults across Jazzy and Humble.
- Streamlines developer containers and bootstrap scripts with Groot installation, UR/Robotiq dependencies, and optional sudo workflows.

Highlights
^^^^^^^^^^

Planner diagnostics and trajectory quality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Introduces the shared ``trajectory_utils`` helpers so both planners can evaluate candidate paths on duration, TCP motion, and accumulated rotation before selecting a winner.
- Logs detailed joint-limit and collision reasons whenever MoveIt rejects a trajectory, making it easier to track down invalid waypoints.
- Detects multi-revolution joint wraparounds plus duplicate joint targets to avoid dispatching stale solutions and to fall back cleanly when the robot sits near joint limits.

Universal Robots launchers and configs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Reworks the ``ur_movegroup_fake_cpp_trees`` and ``ur_moveitcpp_fake_cpp_trees`` launchers so pipeline arrays, controller selection, and fake hardware toggles are parsed consistently between MoveGroup and MoveItCpp demos.
- Adds the Humble-friendly ``ompl_planning.legacy.yaml`` and wires in the upstream ``ur_moveit_config`` dependency so legacy adapters remain available while Jazzy defaults stay untouched.
- Updates the C++ UR behavior-tree client to share planner knobs (planner IDs, plan budgets, pose builders) across the pick/place sequences, keeping the demos repeatable on hardware or simulation.

Developer containers and bootstrap scripts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Extends the Dockerfile and run scripts to optionally grant sudo inside the container, rebuild all layers on demand, and propagate workspace UID/GID overrides.
- Adds ``docker/scripts/setup_workspace.sh`` which builds Groot from source (with pinning support), runs rosdep, and performs a ``colcon build`` as the unprivileged user.
- Ensures UR and Robotiq dependencies are preinstalled in the bringup images so fake-tree bringup, color-signal clients, and MoveIt demos work out of the box.

0.2.1 (2025-11-06)
------------------

Summary
^^^^^^^
- Ships new Universal Robots behavior-tree examples and walkthroughs.
- Refreshes Dockerfiles and bootstrap scripts for reproducible developer containers.
- ManyMove packages on ROS 2 Jazzy assorted bug fixes.
- Extracts the former ``manymove_py_trees`` package into its own standalone repository.

Highlights
^^^^^^^^^^

Universal Robots examples
~~~~~~~~~~~~~~~~~~~~~~~~~
- Adds end-to-end UR client demos that exercise the behavior-tree planners and workflow scripts.
- Documents the setup steps so users can iterate quickly in simulation or with hardware.

Docker and Jazzy compatibility
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Updates Dockerfiles and workspace bootstrap scripts to align toolchains across Humble and Jazzy.
- Validates Jazzy support with CI runs while tightening dependency pins and colcon metadata.

Bug fixes
~~~~~~~~~
- Resolves planner edge cases uncovered by the UR demos and improves HMI feedback loops.
- Fixes minor regressions in message handlers and launch files encountered during container testing.

Repository cleanup
~~~~~~~~~~~~~~~~~~
- Removes the in-repo ``manymove_py_trees`` package; users should use the new standalone release.

0.2.0 (2025-10-20)
------------------

Summary
^^^^^^^
- First ManyMove release promoted from `dev` to the new `main` branch.
- Aligns the full stack for ROS 2 Humble and Jazzy while standardizing documentation,
  packaging, and quality gates.

Highlights
^^^^^^^^^^

Unified manipulation command stack
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Introduces a single ``MoveManipulator`` action that covers planning and execution across
  planner servers, behavior-tree clients, and message definitions.
- Streamlines action handling with improved preemption, logging, and validation, while updating
  Docker/Isaac ROS flows and tutorials to the unified interface.

Behavior tree libraries
~~~~~~~~~~~~~~~~~~~~~~~
- Expands the BehaviorTree.CPP node catalog, improves HMI integration/logging, and adds
  walkthrough tutorials and example clients.
- Refreshes the Python behavior-tree examples to the new ``MoveManipulator`` action with linting,
  formatting, and unit-test coverage improvements.

Object and scene management
~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Enhances object manager actions for adding/removing/attaching objects, clarifies result messages,
  and refreshes referenced assets and meshes.
- Revises the message package to centralize custom interfaces and decouple legacy dependencies.

Operator experience and bringup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Updates HMI widgets with stronger validation, additional field types, and clearer feedback for
  runtime status flows.
- Delivers new and refined launchers (single- and dual-robot demos), sequential bringup scripts,
  and Docker/Isaac ROS helpers that keep example pipelines reproducible.

Tooling and quality
^^^^^^^^^^^^^^^^^^^
- Standardizes READMEs and maintainer metadata across packages.
- Establishes pre-commit style automation with `ruff`, `isort`, and Uncrustify profiles that pass
  on Humble (0.72) and Jazzy (0.78), tightening CI plus `colcon test` coverage.
- Applies broad formatting, license, and minimum CMake updates to keep the codebase consistent.

Breaking changes / deprecations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- **Behavior/Planner action unification**: the previous split between “plan” and “execute” actions
  is replaced by ``MoveManipulator`` and the updated behavior-tree nodes/clients. Update any custom
  integrations that relied on the legacy actions.

Migration notes
^^^^^^^^^^^^^^^
- Replace references to legacy plan/execute actions with the new ``MoveManipulator`` action.
- Review launchers, parameter names, and tutorials: several examples were normalized during the
  unification effort.
- Install and use the provided formatting/linting hooks so local results match CI expectations.

Contributors
^^^^^^^^^^^^
- Marco Pastorio
