========================
ManyMove – Changelog
========================

This is the repository-level changelog that summarizes notable changes across all packages.
Per-package changelogs may still be generated during release, but this file is the canonical
high-level history.

Forthcoming
-----------

**Scope**

This section covers the *upcoming merge* that promotes `dev` to the new `main` branch,
with the goal of keeping a single codebase compatible with ROS 2 Humble and Jazzy.

Repository-wide
^^^^^^^^^^^^^^^
- Readmes aligned across sub-packages; versions/maintainers made uniform.
- Linting/formatting modernized: `ruff` and `isort` for Python; Uncrustify configs aligned
  to pass on both 0.72 (Humble) and 0.78 (Jazzy); stricter CI checks; fixes for `colcon test`.
- General code cleanup, consistent line-length (≈100), license/copyright headers,
  and minimum CMake uplift where applicable.

Package highlights
^^^^^^^^^^^^^^^^^^

manymove_planner
~~~~~~~~~~~~~~~~
- **New unified motion action**: transition to a single ``MoveManipulator`` action and streamlined
  planning/execution path; improved stop handling, logging, and validation.
- Broader example coverage (MoveItCpp & MoveGroup), with polish to planning pipelines,
  sequential execution examples, and Docker/Isaac ROS helper updates.
- Numerous formatting and linter updates; increased Jazzy compatibility.
  *(See per-package history for finer details.)*

manymove_cpp_trees
~~~~~~~~~~~~~~~~~~
- Expanded BehaviorTree.CPP nodes and helpers; improved HMI integration and logging.
- Added a hands-on tutorial and example clients; foundationpose/Isaac Sim interactions.
- Formatting, uncrustify alignment (Humble & Jazzy), and CI refinements.

manymove_object_manager
~~~~~~~~~~~~~~~~~~~~~~~
- Action improvements for adding/removing/attaching objects; result messages clarified.
- Asset and mesh updates/renames; correctness and robustness improvements.
- Consistent formatting and linter enforcement.

manymove_hmi
~~~~~~~~~~~~
- GUI/HMI improvements (input validation, new field types, clearer styles).
- New service logic to update behavior-tree blackboard keys; better feedback and status flows.
- Uncrustify and CMake updates; fixes to pass `colcon test`.

manymove_msgs
~~~~~~~~~~~~~
- Centralized custom interfaces for manipulation and object management; introduction and
  maturation of ``MoveManipulator`` action with supporting messages.
- Clean separation from signals-related dependencies and other minor interface refinements.

manymove_py_trees
~~~~~~~~~~~~~~~~~
- Python behavior-tree examples updated to the unified ``MoveManipulator`` action.
- Linting, formatting, and testing profile improvements; small correctness fixes.

manymove_bringup
~~~~~~~~~~~~~~~~
- New/updated launchers for single/dual-robot demos; sequential start-up and clearer params.
- Examples expanded (pilz/chomp mixes, STOMP refs, dual-arm scenarios); fixes for `colcon test`.
- Stricter checks, formatting cleanups.

Breaking changes / deprecations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- **Behavior/Planner action unification**: previous split between “plan” and “execute” actions is
  replaced by the single ``MoveManipulator`` action and updated BT nodes/clients. Migrate any user
  code that depended on the older separate actions to the new interface.

Migration notes
^^^^^^^^^^^^^^^
- Replace references to legacy plan/execute actions with the new ``MoveManipulator`` action.
- Review launchers and parameters: several examples and names were normalized.
- Ensure formatting hooks (pre-commit) and `ament_*` linters are installed/used so CI and local
  results match.

Contributors
^^^^^^^^^^^^
- Marco Pastorio
