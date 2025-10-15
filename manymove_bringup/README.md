# manymove_bringup

## Overview
The `manymove_bringup` package collects the launch files that boot complete Manymove demonstration stacks. Each launch orchestrates the robot state publishers, `ros2_control` controllers, MoveIt motion planning (MoveGroup or MoveItCpp), the Manymove action servers, the object manager, behaviour tree clients (C++ or Python), the HMI interface, and RViz when requested. Event-driven launch sequencing ensures controllers and transforms come up before the planners and task-level nodes, supporting both single-arm and dual-arm configurations.

These launch files ship as data only (no custom Python nodes) and are intended to be run from a properly configured ROS 2 workspace. Refer to the [main README](../README.md) for installation and workspace setup guidance.

## Launch files
All launch descriptions share a core set of arguments for configuring the robots: `robot_type` / `robot_type_{1,2}`, `dof` / `dof_{1,2}`, `prefix` / `prefix_{1,2}`, `base_frame` / `base_frame_{1,2}`, `tcp_frame` / `tcp_frame_{1,2}`, `attach_to(_1/_2)`, `attach_xyz(_1/_2)`, `attach_rpy(_1/_2)`, `add_gripper(_1/_2)`, `add_vacuum_gripper(_1/_2)`, `add_bio_gripper(_1/_2)`, `add_realsense_d435i(_1/_2)`, `ros2_control_plugin`, `use_sim_time`, and `ros_namespace`. Scenario-specific toggles (for example `gripper_action_server`, `contact_links`, `log_level`, sensor meshes, or Isaac plugin topics) are highlighted below.

### Dual-robot scenarios
- `collision_test_movegroup.launch.py` - Dual-arm MoveGroup bringup for collision-testing behaviour trees; starts fake hardware, MoveGroup-based planners, RViz, object manager, the C++ `manymove_cpp_trees` collision test client, and HMI. Defaults to a lite and uf850 arm and exposes the dual-arm argument set (`robot_type_1/2`, `prefix_1/2`, `base_frame_1/2`, `tcp_frame_1/2`, `attach_xyz_1/2`, gripper toggles).
- `collision_test_moveitcpp.launch.py` - Companion scenario that swaps in the MoveItCpp action server while running the same collision-test behaviour tree and dual-arm configuration. Shares the dual-arm arguments and `ros2_control_plugin` selection for fake versus hardware controllers.
- `dual_movegroup_fake_app.launch.py` - Dual-arm MoveGroup demo aligned with the Manymove HMI "app" workflow; launches MoveGroup, two `manymove_planner` action servers, RViz, object manager, C++ behaviour tree client, and HMI. Configure robots with the standard dual-arm arguments (`robot_type_1/2`, `dof_1/2`, `prefix_1/2`, `base_frame_1/2`, `tcp_frame_1/2`, gripper options).
- `dual_movegroup_fake_cpp_trees.launch.py` - Dual MoveGroup bringup with the C++ behaviour tree client instead of the app wrapper, keeping the same sequencing of controllers, object manager, and HMI. Accepts the dual-arm argument set plus gripper toggles.
- `dual_moveitcpp_fake_app.launch.py` - Dual-arm MoveItCpp demo for fake hardware, combining the MoveItCpp action servers with the app-style behaviour tree and HMI. Supports robot mixing (lite/uf850 by default) via `robot_type_1/2`, `dof_1/2`, `prefix_1/2`, `base_frame`/`tcp` arguments, and gripper or sensor flags.
- `dual_moveitcpp_fake_cpp_trees.launch.py` - Dual MoveItCpp launch that drives the C++ behaviour tree client without the app wrapper; exposes the same dual-arm options (`robot_type_1/2`, `prefix_1/2`, frame attachments, gripper controls).

### Lite robot scenarios
- `tutorial_01.launch.py` - Introductory single-lite demo used in documentation; spins up fake hardware, a MoveItCpp action server, RViz, object manager, and C++ behaviour tree with perception meshes enabled. Key arguments include `robot_type`/`dof`, `prefix`, frame attachments, sensor toggles, gripper options, and `ros2_control_plugin`.
- `lite_movegroup_fake_cpp_trees.launch.py` - Single-lite MoveGroup stack with the C++ fake task behaviour tree and HMI; accepts the common single-arm arguments (`robot_type`, `prefix`, `base_frame`, `tcp_frame`, attachment offsets, gripper toggles).
- `lite_movegroup_fake_py_trees.launch.py` - MoveGroup bringup for a lite arm that launches the Python behaviour tree client from `manymove_py_trees`. Uses the standard single-arm argument set and gripper or sensor flags.
- `lite_moveitcpp_fake_cpp_trees.launch.py` - Single-lite MoveItCpp stack with the C++ fake client and RViz; honours the common arguments (`robot_type`, `prefix`, `base_frame`, `tcp_frame`, `attach_xyz`, `add_gripper`, `ros2_control_plugin`).
- `lite_cumotion_movegroup_fake_cpp_trees.launch.py` - MoveGroup configuration augmented with the `isaac_ros_cumotion` planner and the CuMotion-specific behaviour tree client. Requires the common single-arm arguments plus access to the CuMotion configuration resources.
- `lite_foundationpose_movegroup_fake_cpp_trees.launch.py` - MoveGroup demo wired to the Isaac ROS FoundationPose behaviour tree client (`bt_client_foundationpose`) and HMI. Adds arguments for `gripper_action_server`, `contact_links`, and `log_level` in addition to the common bringup options.
- `lite_isaac_sim_movegroup_fake_cpp_trees.launch.py` - MoveGroup stack prepared for Isaac Sim integration; defaults `ros2_control_plugin:=isaac` and includes `gripper_action_server` (Isaac command topic), `contact_links`, and `log_level` arguments alongside the standard robot configuration.
- `mixed_pipelines_lite_movegroup_fake_cpp_trees.launch.py` - Single-lite MoveGroup bringup intended to exercise multiple planning pipelines (OMPL, CHOMP, Pilz) via the `bt_client_mixed_pipelines` C++ client. Uses the common arguments and gripper toggles.
- `mixed_pipelines_lite_moveitcpp_fake_cpp_trees.launch.py` - MoveItCpp counterpart for the mixed pipeline tests, sharing the same behaviour tree client and standard single-arm configuration arguments.

### Panda scenarios
- `panda_movegroup_fake_cpp_trees.launch.py` - Fake-hardware MoveGroup bringup for a Franka Panda, launching the C++ behaviour tree client and HMI. Key arguments cover `planning_group`, `base_frame`, `tcp_frame`, `traj_controller`, `gripper_action_server`, and `contact_links` in addition to the usual gripper toggles.
- `panda_movegroup_fake_py_trees.launch.py` - Panda MoveGroup demo using the Python behaviour tree client; exposes the same argument set (`planning_group`, frame configuration, `traj_controller`, `gripper_action_server`, `contact_links`).
- `panda_moveitcpp_fake_cpp_trees.launch.py` - Panda MoveItCpp launch with the C++ fake client, RViz, and HMI; accepts `planning_group`, frame arguments, `traj_controller`, `gripper_action_server`, and `contact_links`.
- `panda_cumotion_movegroup_fake_cpp_trees.launch.py` - Panda MoveGroup scenario extended with the CuMotion planner (`isaac_ros_cumotion_moveit`) and the `bt_client_panda_cumotion` behaviour tree. Requires the Panda-specific argument set plus CuMotion resources.

### UF850 scenarios
- `uf850_movegroup_fake_cpp_trees.launch.py` - Single UF850 MoveGroup stack with the C++ fake behaviour tree and HMI. Arguments include `robot_type:=uf850`, `prefix`, frame attachments, `gripper_action_server`, and `contact_links` alongside gripper toggles.
- `uf850_moveitcpp_fake_cpp_trees.launch.py` - UF850 MoveItCpp variant launching the corresponding C++ behaviour tree client and RViz; uses the same argument set (`base_frame`, `tcp_frame`, `gripper_action_server`, `contact_links`).

### XArm7 scenarios
- `xarm7_movegroup_fake_cpp_trees.launch.py` - Single xArm7 MoveGroup bringup using fake hardware, the C++ behaviour tree client, and HMI. Exposes `base_frame`, `tcp_frame`, `gripper_action_server`, and `contact_links` in addition to the common gripper toggles.
- `xarm7_moveitcpp_fake_cpp_trees.launch.py` - MoveItCpp configuration for xArm7, reusing the same argument interface and behaviour tree client while swapping in the MoveItCpp action server.

## Usage
Run these launch files from a sourced ROS 2 workspace that already contains the Manymove stack, xArm packages, MoveIt, and any optional NVIDIA Isaac components you plan to use. For example:
```bash
ros2 launch manymove_bringup dual_moveitcpp_fake_app.launch.py \
  robot_type_1:=lite robot_type_2:=uf850 \
  prefix_1:=L_ prefix_2:=R_ \
  base_frame_1:=world base_frame_2:=world \
  add_gripper_1:=true add_realsense_d435i_1:=true use_sim_time:=true
```
Adjust arguments to match your robots (frames, prefixes, degrees of freedom, gripper style, `ros2_control` plugin). When targeting real hardware, point `ros2_control_plugin` at the appropriate hardware interface and supply the correct serial numbers or namespaces. Launch RViz automatically via the provided files or disable its inclusion by editing the launch description. Follow the [main README](../README.md) for environment setup, package dependencies, and sourcing instructions.

## Dependencies
The launch files rely on:
- Manymove runtime packages: `manymove_planner`, `manymove_object_manager`, `manymove_cpp_trees`, `manymove_hmi`, and `manymove_py_trees`.
- UFactory robot stack: `uf_ros_lib`, `xarm_api`, `xarm_controller`, and their configuration resources.
- ROS 2 core tooling: `ros2_control` (`controller_manager`, joint state broadcaster), `robot_state_publisher`, `tf2_ros`, `rviz2`, `moveit_ros_move_group`, and standard MoveIt resources.
- Scenario-specific extras: NVIDIA `isaac_ros_cumotion_moveit`, Isaac Sim control plugins, and (for foundation pose demos) the Isaac ROS FoundationPose stack.

Ensure these packages are built in the workspace or available as installed dependencies before running the bringup launches.

## Quality Declaration
This package is experimental and currently intended for demonstrations and manual testing. A formal ROS 2 quality declaration has not been authored yet. Adding automated launch tests, smoke-test scenarios, or CI jobs that validate controller bringup and behaviour tree execution is recommended before relying on these files in production.

## License and Maintainers
`manymove_bringup` is licensed under the BSD-3-Clause license. The package is maintained by Marco Pastorio (<pastoriomarco@gmail.com>). Contributions and questions should be coordinated through the main Manymove repository.
