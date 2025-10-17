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

"""Launch description for the panda movegroup fake py trees scenario."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    """Configure launch actions for the panda movegroup fake py trees scenario."""
    planning_group = LaunchConfiguration('planning_group')
    base_frame = LaunchConfiguration('base_frame')
    tcp_frame = LaunchConfiguration('tcp_frame')
    traj_controller = LaunchConfiguration('traj_controller')

    gripper_action_server = (LaunchConfiguration('gripper_action_server'),)
    contact_links = (LaunchConfiguration('contact_links'),)

    ros2_control_hardware_type = DeclareLaunchArgument(
        'ros2_control_hardware_type',
        default_value='mock_components',
        description=(
            'ROS2 control hardware interface type to use for the launch file -- '
            'possible values: [mock_components, isaac]'
        ),
    )

    moveit_configs = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(
            file_path='config/panda.urdf.xacro',
            mappings={
                'ros2_control_hardware_type': LaunchConfiguration('ros2_control_hardware_type')
            },
        )
        .robot_description_semantic(file_path='config/panda.srdf')
        .trajectory_execution(file_path='config/gripper_moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'])
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory('manymove_planner'),
                'config',
                'moveit_cpp.yaml',
            )
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_configs.to_dict()],
    )

    # Define the action_server_node with new parameters
    action_server_node = Node(
        package='manymove_planner',
        executable='action_server_node',
        # Don't use the "name" parameter; the name will be automatically
        # set with {node_prefix}action_server_node to
        # avoid duplicate nodes
        output='screen',
        parameters=[
            moveit_configs.to_dict(),
            {
                'node_prefix': '{}_'.format(planning_group.perform(context)),
                'planner_type': 'movegroup',
                'planning_group': planning_group,
                'base_frame': base_frame,
                'tcp_frame': tcp_frame,
                'traj_controller': traj_controller,
            },
        ],
    )

    # RViz
    rviz_config_file = get_package_share_directory('manymove_planner') + '/config/micpp_demo.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'rviz2:=fatal'],
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_configs.planning_pipelines,
            moveit_configs.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'panda_link0'],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_configs.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_configs.robot_description, ros2_controllers_path],
        output='both',
    )

    # Load controllers
    load_controllers = []
    for controller in [
        'panda_arm_controller',
        'panda_hand_controller',
        'joint_state_broadcaster',
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    # ================================================================
    # launch manymove_object_manager
    # ================================================================

    # Object Manager node
    object_manager_node = Node(
        package='manymove_object_manager',
        executable='object_manager_node',
        name='object_manager_node',
        output='screen',
        parameters=[{'frame_id': 'world'}],
    )

    # ================================================================
    # launch manymove_hmi
    # ================================================================

    # HMI node
    manymove_hmi_node = Node(
        package='manymove_hmi',
        executable='manymove_hmi_executable',
        # name='manymove_hmi_node',
        output='screen',
        parameters=[
            {
                'robot_prefixes': [''],
                'robot_names': ['Franka_Emika_Panda'],
            }
        ],
    )

    # ================================================================
    # launch manymove_cpp_trees
    # ================================================================

    # py_trees node
    manymove_py_trees_node = Node(
        package='manymove_py_trees',
        executable='bt_client_fake_panda',
        output='screen',
        parameters=[
            {
                'robot_model': planning_group,
                'robot_prefix': '',
                'tcp_frame': tcp_frame,
                'gripper_action_server': gripper_action_server,
                'contact_links': contact_links,
                'is_robot_real': False,
            }
        ],
    )

    return [
        ros2_control_hardware_type,
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        action_server_node,
        ros2_control_node,
        object_manager_node,
        manymove_hmi_node,
        manymove_py_trees_node,
    ] + load_controllers


def generate_launch_description():
    """Create the launch description entry point."""
    return LaunchDescription(
        [
            # DeclareLaunchArguments for planning_group, base_frame, tcp_frame
            DeclareLaunchArgument(
                'planning_group',
                default_value='panda_arm',
                description='MoveIt planning group',
            ),
            DeclareLaunchArgument(
                'base_frame',
                default_value='panda_link0',
                description='Base frame of the robot',
            ),
            DeclareLaunchArgument(
                'tcp_frame',
                default_value='panda_link8',
                description='TCP (end effector) frame of the robot',
            ),
            DeclareLaunchArgument(
                'traj_controller',
                default_value='panda_arm_controller',
                description='traj_controller action server name of the robot',
            ),
            DeclareLaunchArgument(
                'gripper_action_server',
                default_value='/panda_hand_controller/gripper_cmd',
                description='Name of the action server to control the gripper',
            ),
            DeclareLaunchArgument(
                'contact_links',
                default_value='["panda_leftfinger", "panda_rightfinger", "panda_hand"]',
                description='List of links to exclude from collision checking',
            ),
            # OpaqueFunction to set up the node
            OpaqueFunction(function=launch_setup),
        ]
    )
