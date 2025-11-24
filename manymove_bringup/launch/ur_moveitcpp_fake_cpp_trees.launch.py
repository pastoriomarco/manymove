# Copyright 2025 Flexin Group SRL
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Flexin Group SRL nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch description for a UR MoveItCpp scenario using Manymove CPP trees."""

import os
from copy import deepcopy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetLaunchConfiguration
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from manymove_bringup.pipeline_utils import normalize_pipeline_config
from manymove_bringup.ros_compat import resolve_moveit_controller_config
from manymove_bringup.ros_compat import use_legacy_moveit_adapter_format

import yaml

DEFAULT_UR_KINEMATICS = {
    # Conservative defaults that work across ROS 2 releases when the upstream
    # MoveIt config (e.g. on Jazzy) omits robot_description_kinematics.
    'ur_manipulator': {
        'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_search_resolution': 0.005,
        'kinematics_solver_timeout': 0.005,
        'kinematics_solver_attempts': 3,
    }
}


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def launch_setup(context, *args, **kwargs):
    """Configure LaunchDescription for running a UR robot with MoveItCpp."""
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    controller_spawner_timeout = LaunchConfiguration('controller_spawner_timeout')

    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    safety_limits = LaunchConfiguration('safety_limits')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    moveit_joint_limits_file = LaunchConfiguration('moveit_joint_limits_file')
    srdf_package = LaunchConfiguration('srdf_package')
    gripper_action_server = LaunchConfiguration('gripper_action_server')
    contact_links = LaunchConfiguration('contact_links')

    prefix_raw = LaunchConfiguration('prefix').perform(context)
    planner_type = LaunchConfiguration('planner_type').perform(context)
    planning_group = LaunchConfiguration('planning_group').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    tcp_frame = LaunchConfiguration('tcp_frame').perform(context)
    traj_controller = LaunchConfiguration('traj_controller').perform(context)

    prefix_clean = prefix_raw.strip('"')
    set_tf_prefix = SetLaunchConfiguration('tf_prefix', prefix_clean)

    logger = logging.get_logger('ur_moveitcpp_fake_cpp_trees.launch')

    ur_type_value = ur_type.perform(context)
    use_fake_value = use_fake_hardware.perform(context).lower()
    is_robot_real = use_fake_value not in ('true', '1', 't', 'yes', 'on')

    ros_distro = os.environ.get('ROS_DISTRO', '').strip().lower()
    use_jazzy_fake_minimal = use_fake_value == 'true' and ros_distro == 'jazzy'

    if use_jazzy_fake_minimal and traj_controller == 'scaled_joint_trajectory_controller':
        # Minimal Jazzy layout only spawns JTC; align client-side expectations.
        traj_controller = 'joint_trajectory_controller'

    moveit_config_pkg_name = moveit_config_package.perform(context)

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'config', ur_type, 'joint_limits.yaml']
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'config', ur_type, 'default_kinematics.yaml']
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'config', ur_type, 'physical_parameters.yaml']
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'config', ur_type, 'visual_parameters.yaml']
    )

    description_file_value = description_file.perform(context)
    if '/' in description_file_value:
        description_path = PathJoinSubstitution(
            [FindPackageShare(description_package), description_file]
        )
    else:
        description_path = PathJoinSubstitution(
            [FindPackageShare(description_package), 'urdf', description_file]
        )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            description_path,
            ' ',
            'robot_ip:=',
            robot_ip,
            ' ',
            'joint_limit_params:=',
            joint_limit_params,
            ' ',
            'kinematics_params:=',
            kinematics_params,
            ' ',
            'physical_params:=',
            physical_params,
            ' ',
            'visual_params:=',
            visual_params,
            ' ',
            'safety_limits:=',
            safety_limits,
            ' ',
            'safety_pos_margin:=',
            safety_pos_margin,
            ' ',
            'safety_k_position:=',
            safety_k_position,
            ' ',
            'name:=',
            'ur',
            ' ',
            'ur_type:=',
            ur_type,
            ' ',
            'script_filename:=ros_control.urscript',
            ' ',
            'input_recipe_filename:=rtde_input_recipe.txt',
            ' ',
            'output_recipe_filename:=rtde_output_recipe.txt',
            ' ',
            'prefix:=',
            LaunchConfiguration('prefix'),
            ' ',
            'tf_prefix:=',
            LaunchConfiguration('tf_prefix'),
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'fake_sensor_commands:=false',
            ' ',
            'headless_mode:=false',
            ' ',
            'use_tool_communication:=false',
            ' ',
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    moveit_config_file_value = moveit_config_file.perform(context)
    if moveit_config_file_value.startswith('package://'):
        srdf_path = moveit_config_file_value
    elif moveit_config_file_value.startswith('/') or moveit_config_file_value.startswith('file://'):
        srdf_path = moveit_config_file_value
    elif '/' in moveit_config_file_value:
        srdf_path = PathJoinSubstitution([FindPackageShare(srdf_package), moveit_config_file])
    else:
        srdf_path = PathJoinSubstitution(
            [FindPackageShare(srdf_package), 'srdf', moveit_config_file]
        )

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            srdf_path,
            ' ',
            'name:=',
            'ur',
            ' ',
            'prefix:=',
            LaunchConfiguration('prefix'),
            ' ',
        ]
    )
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    publish_robot_description_semantic_param = {
        'publish_robot_description_semantic': publish_robot_description_semantic
    }

    raw_kinematics_yaml = load_yaml(
        moveit_config_pkg_name, os.path.join('config', 'kinematics.yaml')
    )
    kinematics_data: dict[str, dict] = {}
    if isinstance(raw_kinematics_yaml, dict):
        if 'robot_description_kinematics' in raw_kinematics_yaml:
            kinematics_data = raw_kinematics_yaml['robot_description_kinematics']
        else:
            kinematics_data = (
                raw_kinematics_yaml.get('/**', {})
                .get('ros__parameters', {})
                .get('robot_description_kinematics', {})
            )
            if not kinematics_data:
                kinematics_data = {
                    key: value
                    for key, value in raw_kinematics_yaml.items()
                    if isinstance(value, dict)
                    and (
                        'kinematics_solver' in value
                        or any(k.startswith('kinematics_') for k in value.keys())
                    )
                }

    fallback_candidates = {planning_group, 'ur_manipulator'}
    fallback_candidates.discard('')
    for group_name in fallback_candidates:
        existing_entry = kinematics_data.get(group_name, {})
        needs_solver = not isinstance(existing_entry, dict) or not existing_entry.get(
            'kinematics_solver'
        )
        if not needs_solver:
            continue

        default_config = DEFAULT_UR_KINEMATICS.get(group_name) or DEFAULT_UR_KINEMATICS.get(
            'ur_manipulator'
        )
        if not default_config:
            continue

        merged_entry = deepcopy(default_config)
        if isinstance(existing_entry, dict):
            merged_entry.update(existing_entry)
        kinematics_data[group_name] = merged_entry
        logger.warning(
            (
                f"MoveIt kinematics configuration for group '{group_name}' is missing. "
                'Falling back to KDL defaults to keep pose goals working on ROS 2 Jazzy.'
            )
        )
    robot_description_kinematics = (
        {'robot_description_kinematics': kinematics_data} if kinematics_data else None
    )
    robot_description_planning = {
        'robot_description_planning': load_yaml(
            moveit_config_pkg_name,
            os.path.join('config', moveit_joint_limits_file.perform(context)),
        )
    }

    cartesian_limits = load_yaml(
        'manymove_bringup', os.path.join('config', 'ur', 'pilz_cartesian_limits.yaml')
    )
    if cartesian_limits:
        robot_description_planning['robot_description_planning'].setdefault('cartesian_limits', {})
        robot_description_planning['robot_description_planning']['cartesian_limits'].update(
            cartesian_limits.get('cartesian_limits', {})
        )

    # Resolve planning pipeline YAMLs per package to avoid distro-specific symlinks
    ompl_planning_relpath = os.path.join(
        'config',
        'ur',
        'ompl_planning.legacy.yaml' if use_legacy_moveit_adapter_format() else 'ompl_planning.yaml',
    )
    planning_pipeline_files = {
        'ompl': (
            'manymove_bringup',
            ompl_planning_relpath,
        ),
        'chomp': (
            'moveit_resources_panda_moveit_config',
            os.path.join('config', 'chomp_planning.yaml'),
        ),
        'pilz_industrial_motion_planner': (
            'moveit_resources_panda_moveit_config',
            os.path.join('config', 'pilz_industrial_motion_planner_planning.yaml'),
        ),
    }
    planning_pipeline_config = {
        'planning_pipelines': list(planning_pipeline_files.keys()),
        'default_planning_pipeline': 'ompl',
    }
    for pipeline_name, (pkg, rel_path) in planning_pipeline_files.items():
        planning_pipeline_config[pipeline_name] = normalize_pipeline_config(
            load_yaml(pkg, rel_path)
        )
    try:
        if 'planner_configs' not in planning_pipeline_config.get('ompl', {}):
            ompl_defaults = load_yaml(
                'moveit_configs_utils', os.path.join('default_configs', 'ompl_defaults.yaml')
            )
            if isinstance(ompl_defaults, dict):
                planning_pipeline_config['ompl'].update(ompl_defaults)
    except Exception:
        pass

    for plugin_name, plugin in {
        'chomp': 'chomp_interface/CHOMPPlanner',
        'pilz_industrial_motion_planner': 'pilz_industrial_motion_planner/CommandPlanner',
    }.items():
        pipeline_entry = planning_pipeline_config.get(plugin_name)
        if isinstance(pipeline_entry, dict):
            pipeline_entry['planning_plugin'] = plugin

    planning_pipeline_config = normalize_pipeline_config(planning_pipeline_config)

    MOVEIT_CONTROLLER = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    controllers_yaml = None
    controllers_config_relpath = ''
    if use_jazzy_fake_minimal:
        controllers_config_relpath = os.path.join('config', 'ur', 'controllers_fake_minimal.yaml')
        controllers_yaml = load_yaml('manymove_bringup', controllers_config_relpath)
    else:
        try:
            controllers_config_relpath = resolve_moveit_controller_config(moveit_config_pkg_name)
            controllers_yaml = load_yaml(moveit_config_pkg_name, controllers_config_relpath)
        except (FileNotFoundError, OSError) as exc:
            controllers_config_relpath = os.path.join(
                'config', 'ur', 'controllers_fake_minimal.yaml'
            )
            logger.warning(
                (
                    f'MoveIt controller config is unavailable ({exc}). '
                    f"Falling back to '{controllers_config_relpath}' from 'manymove_bringup'."
                )
            )
            controllers_yaml = load_yaml('manymove_bringup', controllers_config_relpath)
    if (
        use_sim_time.perform(context).lower() == 'true'
        and not use_jazzy_fake_minimal
        and isinstance(controllers_yaml, dict)
        and 'scaled_joint_trajectory_controller' in controllers_yaml
        and 'joint_trajectory_controller' in controllers_yaml
    ):
        controllers_yaml['scaled_joint_trajectory_controller']['default'] = False
        controllers_yaml['joint_trajectory_controller']['default'] = True

    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': MOVEIT_CONTROLLER,
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': os.path.expanduser(warehouse_sqlite_path.perform(context)),
    }

    moveit_cpp_config = load_yaml(
        'manymove_bringup', os.path.join('config', 'ur', 'moveit_cpp.yaml')
    )

    planning_pipeline_names_param = {
        'planning_pipelines.pipeline_names': list(planning_pipeline_files.keys()),
        'planning_pipelines.default_planning_pipeline': 'ompl',
    }

    moveit_parameter_overrides = [
        robot_description,
        robot_description_semantic,
        publish_robot_description_semantic_param,
        robot_description_planning,
        planning_pipeline_config,
        planning_pipeline_names_param,
        moveit_cpp_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        {'use_sim_time': use_sim_time},
        warehouse_ros_config,
    ]
    if robot_description_kinematics:
        moveit_parameter_overrides.append(robot_description_kinematics)

    controllers_file_path = os.path.join(
        get_package_share_directory('manymove_bringup'),
        'config',
        'ur',
        'ur_with_robotiq_controllers.yaml',
    )
    update_rate_config_file = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'config',
        f'{ur_type_value}_update_rate.yaml',
    )

    ros2_control_parameters = [
        robot_description,
        ParameterFile(update_rate_config_file),
        ParameterFile(controllers_file_path, allow_substs=True),
    ]

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=ros2_control_parameters,
        output='screen',
        condition=IfCondition(use_fake_hardware),
    )
    ur_control_node = Node(
        package='ur_robot_driver',
        executable='ur_ros2_control_node',
        parameters=ros2_control_parameters,
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    controllers_active = [
        'joint_state_broadcaster',
        'io_and_status_controller',
        'speed_scaling_state_broadcaster',
        'force_torque_sensor_broadcaster',
        'tcp_pose_broadcaster',
        'ur_configuration_controller',
    ]
    controllers_inactive = [
        'scaled_joint_trajectory_controller',
        'joint_trajectory_controller',
        'forward_velocity_controller',
        'forward_position_controller',
        'force_mode_controller',
        'passthrough_trajectory_controller',
        'freedrive_mode_controller',
        'tool_contact_controller',
    ]

    if use_jazzy_fake_minimal:
        controllers_active = [
            'joint_state_broadcaster',
            'joint_trajectory_controller',
            'robotiq_gripper_controller',
        ]
        controllers_inactive = []

    initial_joint_controller_value = initial_joint_controller.perform(context)
    if not use_jazzy_fake_minimal:
        if initial_joint_controller_value not in controllers_active:
            controllers_active.append(initial_joint_controller_value)
        controllers_inactive = [c for c in controllers_inactive if c not in controllers_active]

    if use_fake_value == 'true' and not use_jazzy_fake_minimal:
        controllers_active = [c for c in controllers_active if c != 'tcp_pose_broadcaster']

    controller_spawner_active = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            controller_spawner_timeout,
            *controllers_active,
        ],
        output='screen',
    )
    controller_spawner_inactive = None
    if controllers_inactive:
        controller_spawner_inactive = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                controller_spawner_timeout,
                '--inactive',
                *controllers_inactive,
            ],
            output='screen',
        )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    gripper_controller_spawners = None
    if not use_jazzy_fake_minimal:
        gripper_controller_spawners = TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=[
                        '--controller-manager',
                        '/controller_manager',
                        '--controller-manager-timeout',
                        controller_spawner_timeout,
                        'robotiq_gripper_controller',
                    ],
                ),
            ],
        )

    rviz_launch_requested = launch_rviz.perform(context).lower() in ('true', '1', 't', 'yes', 'on')
    rviz_node = None
    if rviz_launch_requested:
        rviz_config_file = os.path.join(
            get_package_share_directory('manymove_planner'), 'config', 'micpp_demo.rviz'
        )
        rviz_parameters = [
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            planning_pipeline_config,
            warehouse_ros_config,
            {'use_sim_time': use_sim_time},
        ]
        if robot_description_kinematics:
            rviz_parameters.append(robot_description_kinematics)
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'rviz2:=fatal'],
            parameters=rviz_parameters,
        )

    action_server_node = Node(
        package='manymove_planner',
        executable='action_server_node',
        output='screen',
        parameters=moveit_parameter_overrides
        + [
            {
                'node_prefix': prefix_clean,
                'planner_prefix': prefix_clean,
                'planner_type': planner_type,
                'planning_group': planning_group,
                'base_frame': base_frame,
                'tcp_frame': tcp_frame,
                'traj_controller': traj_controller,
            }
        ],
    )

    object_manager_node = Node(
        package='manymove_object_manager',
        executable='object_manager_node',
        name='object_manager_node',
        output='screen',
        parameters=[{'frame_id': 'world'}],
    )

    manymove_cpp_trees_node = Node(
        package='manymove_cpp_trees',
        executable='bt_client_ur',
        output='screen',
        parameters=[
            {
                'robot_model': ur_type_value,
                'robot_prefix': prefix_clean,
                'tcp_frame': tcp_frame,
                'gripper_action_server': gripper_action_server,
                'contact_links': contact_links,
                'is_robot_real': is_robot_real,
            }
        ],
    )

    manymove_hmi_node = Node(
        package='manymove_hmi',
        executable='manymove_hmi_executable',
        output='screen',
        parameters=[
            {
                'robot_prefixes': [prefix_clean],
                'robot_names': [ur_type_value],
            }
        ],
    )

    launch_actions = [
        set_tf_prefix,
        control_node,
        ur_control_node,
        controller_spawner_active,
    ]
    if controller_spawner_inactive is not None:
        launch_actions.append(controller_spawner_inactive)
    launch_actions.append(robot_state_publisher_node)
    if gripper_controller_spawners is not None:
        launch_actions.append(gripper_controller_spawners)
    launch_actions.append(action_server_node)
    if rviz_launch_requested and rviz_node is not None:
        insert_index = launch_actions.index(action_server_node)
        launch_actions.insert(insert_index, rviz_node)

    handlers = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=action_server_node,
                on_start=[object_manager_node, manymove_cpp_trees_node],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=manymove_cpp_trees_node,
                on_start=[manymove_hmi_node],
            )
        ),
    ]

    return [*launch_actions, *handlers]


def generate_launch_description():
    """Create LaunchDescription entry point."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'ur_type',
                default_value='ur3e',
                description='UR robot variant to launch.',
                choices=[
                    'ur3',
                    'ur5',
                    'ur10',
                    'ur3e',
                    'ur5e',
                    'ur7e',
                    'ur10e',
                    'ur12e',
                    'ur16e',
                    'ur8long',
                    'ur15',
                    'ur18',
                    'ur20',
                    'ur30',
                ],
            ),
            DeclareLaunchArgument(
                'robot_ip',
                default_value='127.0.0.1',
                description='IP address of the UR controller.',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='true',
                description='Use fake hardware interface instead of a physical robot.',
            ),
            DeclareLaunchArgument(
                'initial_joint_controller',
                default_value='scaled_joint_trajectory_controller',
                description='Name of the joint controller to start first.',
            ),
            DeclareLaunchArgument(
                'launch_rviz',
                default_value='true',
                description='Launch RViz configured for MoveItCpp.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation time for launched nodes.',
            ),
            DeclareLaunchArgument(
                'publish_robot_description_semantic',
                default_value='true',
                description='Publish the semantic robot description from MoveIt.',
            ),
            DeclareLaunchArgument(
                'warehouse_sqlite_path',
                default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
                description='Path to the MoveIt warehouse database.',
            ),
            DeclareLaunchArgument(
                'prefix',
                default_value='""',
                description='Prefix applied to UR joints and frames.',
            ),
            DeclareLaunchArgument(
                'tf_prefix',
                default_value='',
                description='tf_prefix applied inside UR controller parameters.',
            ),
            DeclareLaunchArgument(
                'planner_type',
                default_value='moveitcpp',
                description='Planner type for Manymove (movegroup or moveitcpp).',
            ),
            DeclareLaunchArgument(
                'planning_group',
                default_value='ur_manipulator',
                description='MoveIt planning group to be used by Manymove.',
            ),
            DeclareLaunchArgument(
                'base_frame',
                default_value='base_link',
                description='Base frame of the UR robot.',
            ),
            DeclareLaunchArgument(
                'tcp_frame',
                default_value='tool0',
                description='Tool center point frame of the UR robot.',
            ),
            DeclareLaunchArgument(
                'traj_controller',
                default_value='scaled_joint_trajectory_controller',
                description='Follow joint trajectory controller name exposed by the UR driver.',
            ),
            DeclareLaunchArgument(
                'gripper_action_server',
                default_value='robotiq_gripper_controller/gripper_cmd',
                description='Gripper command action server name (prefix applied automatically).',
            ),
            DeclareLaunchArgument(
                'contact_links',
                default_value='["robotiq_85_right_finger_tip_link", '
                '"robotiq_85_left_finger_tip_link"]',
                description='List of links to exclude from collision checking',
            ),
            DeclareLaunchArgument(
                'controller_spawner_timeout',
                default_value='10',
                description='Timeout (s) passed to controller spawner processes.',
            ),
            DeclareLaunchArgument(
                'description_package',
                default_value='manymove_bringup',
                description='Package providing the UR + gripper robot description.',
            ),
            DeclareLaunchArgument(
                'description_file',
                default_value='ur_with_robotiq.xacro',
                description='URDF/Xacro filename (located under the package urdf/ directory).',
            ),
            DeclareLaunchArgument(
                'safety_limits',
                default_value='true',
                description='Enable the UR safety limits in the description.',
            ),
            DeclareLaunchArgument(
                'safety_pos_margin',
                default_value='0.15',
                description='Safety position margin passed to the UR description.',
            ),
            DeclareLaunchArgument(
                'safety_k_position',
                default_value='20',
                description='Safety k-position parameter passed to the UR description.',
            ),
            DeclareLaunchArgument(
                'moveit_config_package',
                default_value='ur_moveit_config',
                description='MoveIt config package providing SRDF and planning parameters.',
            ),
            DeclareLaunchArgument(
                'srdf_package',
                default_value='manymove_bringup',
                description='Package providing the SRDF/Xacro file.',
            ),
            DeclareLaunchArgument(
                'moveit_config_file',
                default_value='srdf/ur_with_robotiq.srdf.xacro',
                description='SRDF/Xacro file describing the UR MoveIt semantics.',
            ),
            DeclareLaunchArgument(
                'moveit_joint_limits_file',
                default_value='joint_limits.yaml',
                description='Joint limits YAML file relative to the MoveIt config package.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
