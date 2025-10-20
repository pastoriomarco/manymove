========================
ManyMove – Changelog
========================

This is the repository-level changelog that summarizes notable changes across all packages.
Per-package changelogs may still be generated during release, but this file is the canonical
high-level history.

Forthcoming
-----------

*TBD.*

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
