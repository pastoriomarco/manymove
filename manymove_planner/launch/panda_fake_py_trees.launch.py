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

"""Launch description for the panda fake py trees scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    """Configure launch actions for the panda fake py trees scenario."""
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
            'ROS2 control hardware interface type to use for the launch file '
            '-- possible values: [mock_components, isaac]'
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
        .to_moveit_configs()
    )

    # Define the action_server_node with new parameters
    action_server_node = Node(
        package='manymove_planner',
        executable='action_server_node',
        # Don't use the "name" parameter; the name will be automatically
        # set with {node_prefix}action_server_node to avoid duplicate nodes
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
                # 'robot_prefix': prefix,
                'tcp_frame': tcp_frame,
                'gripper_action_server': gripper_action_server,
                'contact_links': contact_links,
                'is_robot_real': False,
            }
        ],
    )

    return [
        ros2_control_hardware_type,
        action_server_node,
        object_manager_node,
        manymove_hmi_node,
        manymove_py_trees_node,
    ]


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
