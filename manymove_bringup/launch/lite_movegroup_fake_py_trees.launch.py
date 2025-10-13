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

"""Launch description for the lite movegroup fake py trees scenario."""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import load_python_launch_file_as_module
from launch.substitutions import LaunchConfiguration  # , PathJoinSubstitution
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    """Configure launch actions for the lite movegroup fake py trees scenario."""
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=True)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=True)
    geometry_type = LaunchConfiguration('geometry_type', default='mesh')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.3)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration(
        'geometry_mesh_filename', default='pneumatic_lite.stl'
    )
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration(
        'geometry_mesh_tcp_xyz', default='"0.03075 0 0.11885"'
    )
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0.52 0"')

    # no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = LaunchConfiguration(
        'ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware'
    )

    base_frame = LaunchConfiguration('base_frame')
    tcp_frame = LaunchConfiguration('tcp_frame')

    xarm_type = '{}{}'.format(
        robot_type.perform(context),
        dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '',
    )

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory('xarm_controller'),
            'config',
            '{}_controllers.yaml'.format(xarm_type),
        ),
        prefix=prefix.perform(context),
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context),
    )

    # from xarm_controller _ros2_control.launch.py
    mod = load_python_launch_file_as_module(
        os.path.join(get_package_share_directory('xarm_api'), 'launch', 'lib', 'robot_api_lib.py')
    )
    generate_robot_api_params = getattr(mod, 'generate_robot_api_params')
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context),
        node_name='ufactory_driver',
    )

    # Initialize MoveIt Configurations
    moveit_configs = (
        MoveItConfigsBuilder(
            context=context,
            controllers_name='fake_controllers',
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            effort_control=effort_control,
            velocity_control=velocity_control,
            model1300=model1300,
            robot_sn=robot_sn,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            mesh_suffix=mesh_suffix,
            kinematics_suffix=kinematics_suffix,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
            add_realsense_d435i=add_realsense_d435i,
            add_d435i_links=add_d435i_links,
            add_other_geometry=add_other_geometry,
            geometry_type=geometry_type,
            geometry_mass=geometry_mass,
            geometry_height=geometry_height,
            geometry_radius=geometry_radius,
            geometry_length=geometry_length,
            geometry_width=geometry_width,
            geometry_mesh_filename=geometry_mesh_filename,
            geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
            geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
            geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
            geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
        )
        .robot_description()
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'])
    ).to_moveit_configs()

    # ================================================================
    # from: xarm_description/launch/_robot_description.launch.launch.py
    # ================================================================

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_configs.robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_robot_moveit_common2.launch.py
    # ================================================================

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_configs.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
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
                'node_prefix': prefix.perform(context),
                'planner_type': 'movegroup',
                'planner_prefix': prefix.perform(context),
                'planning_group': xarm_type,
                'base_frame': base_frame.perform(context),
                'tcp_frame': tcp_frame.perform(context),
                'traj_controller': '{}_traj_controller'.format(xarm_type),
            },
        ],
    )

    # Launch RViz
    rviz_config_file = get_package_share_directory('manymove_planner') + '/config/micpp_demo.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        # name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'rviz2:=fatal'],
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_configs.planning_pipelines,
            moveit_configs.joint_limits,
        ],
    )

    xyz = attach_xyz.perform(context)[1:-1].split(' ')
    rpy = attach_rpy.perform(context)[1:-1].split(' ')
    tf_args = (
        xyz + rpy + [attach_to.perform(context), '{}link_base'.format(prefix.perform(context))]
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=tf_args,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_configs.robot_description,
            ros2_control_params,
            robot_params,
        ],
        output='screen',
    )

    controllers = ['{}{}_traj_controller'.format(prefix.perform(context), xarm_type)]
    if add_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append(
            '{}{}_gripper_traj_controller'.format(
                prefix.perform(context), robot_type.perform(context)
            )
        )
    elif (
        add_gripper.perform(context) in ('True', 'true')
        and robot_type.perform(context) == 'lite'
        and ros2_control_plugin.perform(context) != 'isaac'
    ):
        controllers.append('{}lite_gripper_controller'.format(prefix.perform(context)))
    elif (
        add_bio_gripper.perform(context) in ('True', 'true')
        and robot_type.perform(context) != 'lite'
    ):
        controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[controller, '--controller-manager', '/controller_manager'],
            )
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
                'robot_prefixes': [prefix.perform(context)],
                'robot_names': [xarm_type],
            }
        ],
    )

    # ================================================================
    # launch manymove_cpp_trees
    # ================================================================

    # py_trees node
    manymove_py_trees_node = Node(
        package='manymove_py_trees',
        executable='bt_client_fake',
        output='screen',
        parameters=[
            {
                'robot_model': xarm_type,
                'robot_prefix': prefix.perform(context),
                'tcp_frame': tcp_frame,
                'is_robot_real': False,
            }
        ],
    )

    # ================================================================
    #  EVENT‑DRIVEN START‑UP ORDER
    # ================================================================

    # Create a list of spawner nodes for the controllers
    spawner_nodes = []
    for ctrl in controllers:
        spawner_nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    ctrl,
                    '--controller-manager',
                    f'{ros_namespace}/controller_manager',
                ],
            )
        )

    # Define the launch actions that will start immediately
    launch_actions = [
        robot_state_publisher_node,
        move_group_node,
        static_tf,  # Static transform
        ros2_control_node,  # ros2_control node for hardware interface
        joint_state_broadcaster,  # Joint state broadcaster node
        spawner_nodes[0],  # Start the first controller spawner
    ]

    # Define the event handlers to chain the controllers' spawners
    handlers = []
    # Chain the spawners in a sequence: spawner_nodes[0] → spawner_nodes[1] → ...
    for prev, nxt in zip(spawner_nodes, spawner_nodes[1:]):
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=prev,
                    on_exit=[nxt],
                )
            )
        )

    # When the last spawner exits, launch the MoveItCpp action servers
    # (only after all controllers are loaded)
    handlers.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawner_nodes[-1],
                on_exit=[rviz_node, action_server_node],
            )
        )
    )

    # When the MoveItCpp action server starts, launch the object manager and behavior tree client
    handlers.append(
        RegisterEventHandler(
            OnProcessStart(
                target_action=action_server_node,
                on_start=[object_manager_node, manymove_py_trees_node],
            )
        )
    )

    # When the behavior tree client starts, launch the HMI (Human Machine Interface) node
    handlers.append(
        RegisterEventHandler(
            OnProcessStart(
                target_action=manymove_py_trees_node,
                on_start=[manymove_hmi_node],
            )
        )
    )

    # ================================================================
    #  RETURN LIST
    # ================================================================

    # Return all the launch actions and event handlers to the launch system
    return [
        *launch_actions,
        *handlers,
    ]


def generate_launch_description():
    """Create the launch description entry point."""
    return LaunchDescription(
        [
            # DeclareLaunchArguments for base_frame, tcp_frame
            DeclareLaunchArgument(
                'base_frame',
                default_value='link_base',
                description='Base frame of the robot',
            ),
            DeclareLaunchArgument(
                'tcp_frame',
                default_value='link_tcp',
                description='TCP (end effector) frame of the robot',
            ),
            # OpaqueFunction to set up the node
            OpaqueFunction(function=launch_setup),
        ]
    )


# Defaults to this CLI command (remove the kinematics_suffix:=LS1 if not generated
# from a real robot with prefix LS1):
# ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py \
#   add_realsense_d435i:=true add_d435i_links:=true \
#   add_other_geometry:=true geometry_type:=mesh geometry_mass:=0.3 \
#   geometry_mesh_filename:=pneumatic_lite.stl \
#   geometry_mesh_tcp_xyz:="0.03075 0 0.11885" \
#   geometry_mesh_tcp_rpy:="0 0.52 0" kinematics_suffix:=LS1
