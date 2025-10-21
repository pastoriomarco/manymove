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

"""Launch description for a UR MoveIt MoveGroup scenario using Manymove CPP trees."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def launch_setup(context, *args, **kwargs):
    """Configure LaunchDescription for running a UR robot with Manymove."""
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_servo = LaunchConfiguration('launch_servo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')

    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    safety_limits = LaunchConfiguration('safety_limits')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    moveit_joint_limits_file = LaunchConfiguration('moveit_joint_limits_file')

    prefix_raw = LaunchConfiguration('prefix').perform(context)
    planner_type = LaunchConfiguration('planner_type').perform(context)
    planning_group = LaunchConfiguration('planning_group').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    tcp_frame = LaunchConfiguration('tcp_frame').perform(context)
    traj_controller = LaunchConfiguration('traj_controller').perform(context)

    prefix_clean = prefix_raw.strip('"')

    ur_type_value = ur_type.perform(context)
    use_fake_value = use_fake_hardware.perform(context).lower()
    is_robot_real = use_fake_value not in ('true', '1', 't', 'yes', 'on')

    moveit_config_pkg_name = moveit_config_package.perform(context)

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), 'config', ur_type, 'joint_limits.yaml']
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), 'config', ur_type, 'default_kinematics.yaml']
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), 'config', ur_type, 'physical_parameters.yaml']
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), 'config', ur_type, 'visual_parameters.yaml']
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
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
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), 'srdf', moveit_config_file]
            ),
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

    robot_description_kinematics_file = ParameterFile(
        PathJoinSubstitution([FindPackageShare(moveit_config_package), 'config', 'kinematics.yaml'])
    )
    robot_description_planning = {
        'robot_description_planning': load_yaml(
            moveit_config_pkg_name,
            os.path.join('config', moveit_joint_limits_file.perform(context)),
        )
    }

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        moveit_config_pkg_name, os.path.join('config', 'ompl_planning.yaml')
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    MOVEIT_CONTROLLER = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'

    controllers_yaml = load_yaml(moveit_config_pkg_name, os.path.join('config', 'controllers.yaml'))
    if use_sim_time.perform(context).lower() == 'true':
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

    moveit_parameter_overrides = [
        robot_description,
        robot_description_semantic,
        publish_robot_description_semantic_param,
        robot_description_kinematics_file,
        robot_description_planning,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        {'use_sim_time': use_sim_time},
        warehouse_ros_config,
    ]

    launch_rviz_value = launch_rviz.perform(context)

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'initial_joint_controller': initial_joint_controller,
            'launch_rviz': 'false',
            'description_package': description_package,
            'description_file': description_file,
            'kinematics_params_file': kinematics_params,
            'tf_prefix': prefix_clean,
        }.items(),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': launch_rviz_value,
            'launch_servo': launch_servo,
            'use_sim_time': use_sim_time,
            'publish_robot_description_semantic': publish_robot_description_semantic,
            'warehouse_sqlite_path': warehouse_sqlite_path,
            'prefix': LaunchConfiguration('prefix'),
            'moveit_config_package': moveit_config_package,
            'moveit_config_file': moveit_config_file,
            'moveit_joint_limits_file': moveit_joint_limits_file,
        }.items(),
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
        ur_control_launch,
        ur_moveit_launch,
        action_server_node,
    ]

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
                description='Launch RViz from the UR MoveIt configuration.',
            ),
            DeclareLaunchArgument(
                'launch_servo',
                default_value='false',
                description='Enable MoveIt Servo from the UR MoveIt configuration.',
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
                'planner_type',
                default_value='movegroup',
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
                'description_package',
                default_value='ur_description',
                description='Package providing the UR robot description.',
            ),
            DeclareLaunchArgument(
                'description_file',
                default_value='ur.urdf.xacro',
                description='URDF/XACRO file describing the UR robot.',
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
                'moveit_config_file',
                default_value='ur.srdf.xacro',
                description='SRDF/XACRO file describing the UR MoveIt semantics.',
            ),
            DeclareLaunchArgument(
                'moveit_joint_limits_file',
                default_value='joint_limits.yaml',
                description='Joint limits YAML file relative to the MoveIt config package.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
