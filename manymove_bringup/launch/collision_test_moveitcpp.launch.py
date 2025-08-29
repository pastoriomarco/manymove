# ================================================================
# from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
# ================================================================

import os
# import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    RegisterEventHandler, 
)
from launch.event_handlers import (
    OnProcessExit,
    OnProcessStart,
)
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration #, PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_dual_ros2_control_params_temp_file

# ================================================================
# from: xarm_controller/launch/_dual_ros2_control.launch.py
# ================================================================

from launch.launch_description_sources import load_python_launch_file_as_module

# ================================================================
# from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
# ================================================================

def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    robot_type_1 = LaunchConfiguration('robot_type_1', default='lite')#default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default='uf850')#default=robot_type)
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    model1300_1 = LaunchConfiguration('model1300_1', default=model1300)
    model1300_2 = LaunchConfiguration('model1300_2', default=model1300)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    robot_sn_1 = LaunchConfiguration('robot_sn_1', default=robot_sn)
    robot_sn_2 = LaunchConfiguration('robot_sn_2', default=robot_sn)
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    kinematics_suffix_1 = LaunchConfiguration('kinematics_suffix_1', default=kinematics_suffix)
    kinematics_suffix_2 = LaunchConfiguration('kinematics_suffix_2', default=kinematics_suffix)

    attach_to_1 = LaunchConfiguration('attach_to_1', default='world')
    attach_to_2 = LaunchConfiguration('attach_to_2', default='world')
    attach_xyz_1 = LaunchConfiguration('attach_xyz_1', default='1.005 -0.233 0.480')
    attach_xyz_2 = LaunchConfiguration('attach_xyz_2', default='0 0 0')
    attach_rpy_1 = LaunchConfiguration('attach_rpy_1', default='0 0 3.14')
    attach_rpy_2 = LaunchConfiguration('attach_rpy_2', default='0 0 0')
    create_attach_link_1 = LaunchConfiguration('create_attach_link_1', default=True)
    create_attach_link_2 = LaunchConfiguration('create_attach_link_2', default=False)

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_realsense_d435i_1 = LaunchConfiguration('add_realsense_d435i_1', default=add_realsense_d435i)
    add_realsense_d435i_2 = LaunchConfiguration('add_realsense_d435i_2', default=add_realsense_d435i)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=False)
    add_d435i_links_1 = LaunchConfiguration('add_d435i_links_1', default=add_d435i_links)
    add_d435i_links_2 = LaunchConfiguration('add_d435i_links_2', default=add_d435i_links)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=True)
    add_other_geometry_1 = LaunchConfiguration('add_other_geometry_1', default=add_other_geometry)
    add_other_geometry_2 = LaunchConfiguration('add_other_geometry_2', default=add_other_geometry)
    geometry_type = LaunchConfiguration('geometry_type', default='mesh')
    geometry_type_1 = LaunchConfiguration('geometry_type_1', default=geometry_type)
    geometry_type_2 = LaunchConfiguration('geometry_type_2', default=geometry_type)
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.3)
    geometry_mass_1 = LaunchConfiguration('geometry_mass_1', default=geometry_mass)
    geometry_mass_2 = LaunchConfiguration('geometry_mass_2', default=geometry_mass)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_height_1 = LaunchConfiguration('geometry_height_1', default=geometry_height)
    geometry_height_2 = LaunchConfiguration('geometry_height_2', default=geometry_height)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_radius_1 = LaunchConfiguration('geometry_radius_1', default=geometry_radius)
    geometry_radius_2 = LaunchConfiguration('geometry_radius_2', default=geometry_radius)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_length_1 = LaunchConfiguration('geometry_length_1', default=geometry_length)
    geometry_length_2 = LaunchConfiguration('geometry_length_2', default=geometry_length)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_width_1 = LaunchConfiguration('geometry_width_1', default=geometry_width)
    geometry_width_2 = LaunchConfiguration('geometry_width_2', default=geometry_width)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='pneumatic_lite.stl')
    geometry_mesh_filename_1 = LaunchConfiguration('geometry_mesh_filename_1', default=geometry_mesh_filename)
    geometry_mesh_filename_2 = LaunchConfiguration('geometry_mesh_filename_2', default='tube_holder.stl')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_xyz_1 = LaunchConfiguration('geometry_mesh_origin_xyz_1', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_xyz_2 = LaunchConfiguration('geometry_mesh_origin_xyz_2', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_origin_rpy_1 = LaunchConfiguration('geometry_mesh_origin_rpy_1', default=geometry_mesh_origin_rpy)
    geometry_mesh_origin_rpy_2 = LaunchConfiguration('geometry_mesh_origin_rpy_2', default=geometry_mesh_origin_rpy)
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0.03075 0 0.11885"')
    geometry_mesh_tcp_xyz_1 = LaunchConfiguration('geometry_mesh_tcp_xyz_1', default=geometry_mesh_tcp_xyz)
    geometry_mesh_tcp_xyz_2 = LaunchConfiguration('geometry_mesh_tcp_xyz_2', default='"0 0 0.257"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0.52 0"')
    geometry_mesh_tcp_rpy_1 = LaunchConfiguration('geometry_mesh_tcp_rpy_1', default=geometry_mesh_tcp_rpy)
    geometry_mesh_tcp_rpy_2 = LaunchConfiguration('geometry_mesh_tcp_rpy_2', default='"0 0 0"')

    # ================================================================
    # from: src/manymove_planner/launch/lite_micpp_fake_action_server.launch.py
    # ================================================================

    base_frame_1 = LaunchConfiguration('base_frame_1')
    tcp_frame_1 = LaunchConfiguration('tcp_frame_1')
    base_frame_2 = LaunchConfiguration('base_frame_2')
    tcp_frame_2 = LaunchConfiguration('tcp_frame_2')

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
    # ================================================================

    # no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    controllers_name = 'fake_controllers'
    xarm_type_1 = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')
    xarm_type_2 = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')

    ros2_control_params = generate_dual_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_1)),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_2)),
        prefix_1=prefix_1.perform(context), 
        prefix_2=prefix_2.perform(context), 
        add_gripper_1=add_gripper_1.perform(context) in ('True', 'true'),
        add_gripper_2=add_gripper_2.perform(context) in ('True', 'true'),
        add_bio_gripper_1=add_bio_gripper_1.perform(context) in ('True', 'true'),
        add_bio_gripper_2=add_bio_gripper_2.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type_1=robot_type_1.perform(context), 
        robot_type_2=robot_type_2.perform(context), 
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py

    # WARNING: MODIFIED!!!

    # Grouped DualMoveItConfigsBuilder to apply the following to enable moveitcpp pipelines:

    # .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    # .planning_pipelines(pipelines=["ompl"])
    # .moveit_cpp(file_path=get_package_share_directory("manymove_planner") + "/config/moveit_cpp.yaml")

    # ================================================================

    moveit_config = (
        DualMoveItConfigsBuilder(
            context=context,
            controllers_name=controllers_name,
            dof_1=dof_1,
            dof_2=dof_2,
            robot_type_1=robot_type_1,
            robot_type_2=robot_type_2,
            prefix_1=prefix_1,
            prefix_2=prefix_2,
            hw_ns=hw_ns,
            limited=limited,
            effort_control=effort_control,
            velocity_control=velocity_control,
            model1300_1=model1300_1,
            model1300_2=model1300_2,
            robot_sn_1=robot_sn_1,
            robot_sn_2=robot_sn_2,
            mesh_suffix=mesh_suffix,
            kinematics_suffix_1=kinematics_suffix_1,
            kinematics_suffix_2=kinematics_suffix_2,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,

            attach_to_1 = attach_to_1,
            attach_to_2 = attach_to_2,
            attach_xyz_1 = attach_xyz_1,
            attach_xyz_2 = attach_xyz_2,
            attach_rpy_1 = attach_rpy_1,
            attach_rpy_2 = attach_rpy_2,
            create_attach_link_1 = create_attach_link_1,
            create_attach_link_2 = create_attach_link_2,
            
            add_gripper_1=add_gripper_1,
            add_gripper_2=add_gripper_2,
            add_vacuum_gripper_1=add_vacuum_gripper_1,
            add_vacuum_gripper_2=add_vacuum_gripper_2,
            add_bio_gripper_1=add_bio_gripper_1,
            add_bio_gripper_2=add_bio_gripper_2,
            add_realsense_d435i_1=add_realsense_d435i_1,
            add_realsense_d435i_2=add_realsense_d435i_2,
            add_d435i_links_1=add_d435i_links_1,
            add_d435i_links_2=add_d435i_links_2,
            add_other_geometry_1=add_other_geometry_1,
            add_other_geometry_2=add_other_geometry_2,
            geometry_type_1=geometry_type_1,
            geometry_type_2=geometry_type_2,
            geometry_mass_1=geometry_mass_1,
            geometry_mass_2=geometry_mass_2,
            geometry_height_1=geometry_height_1,
            geometry_height_2=geometry_height_2,
            geometry_radius_1=geometry_radius_1,
            geometry_radius_2=geometry_radius_2,
            geometry_length_1=geometry_length_1,
            geometry_length_2=geometry_length_2,
            geometry_width_1=geometry_width_1,
            geometry_width_2=geometry_width_2,
            geometry_mesh_filename_1=geometry_mesh_filename_1,
            geometry_mesh_filename_2=geometry_mesh_filename_2,
            geometry_mesh_origin_xyz_1=geometry_mesh_origin_xyz_1,
            geometry_mesh_origin_xyz_2=geometry_mesh_origin_xyz_2,
            geometry_mesh_origin_rpy_1=geometry_mesh_origin_rpy_1,
            geometry_mesh_origin_rpy_2=geometry_mesh_origin_rpy_2,
            geometry_mesh_tcp_xyz_1=geometry_mesh_tcp_xyz_1,
            geometry_mesh_tcp_xyz_2=geometry_mesh_tcp_xyz_2,
            geometry_mesh_tcp_rpy_1=geometry_mesh_tcp_rpy_1,
            geometry_mesh_tcp_rpy_2=geometry_mesh_tcp_rpy_2,
        ).planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"])
        .moveit_cpp(file_path=get_package_share_directory("manymove_planner") + "/config/moveit_cpp.yaml")
    ).to_moveit_configs()

    # robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_common2.launch.py
    # ================================================================

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # ================================================================
    # from: src/manymove_planner/launch/lite_micpp_fake_action_server.launch.py
    # instead of: xarm_moveit_config/launch/_dual_robot_moveit_common2.launch.py
    # ================================================================

    # Start the actual action server node
    moveitcpp_action_servers_node = Node(
        package='manymove_planner',
        executable='moveitcpp_action_server_node',
        # Don't use the "name" parameter, the name will be automatically set with {node_prefix_*}action_server_node to avoid duplicate nodes
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'node_prefixes': [prefix_1.perform(context), prefix_2.perform(context)],
                'planner_prefixes': [prefix_1.perform(context), prefix_2.perform(context)],
                'planning_groups': [xarm_type_1, xarm_type_2], 
                'base_frames': [base_frame_1.perform(context), base_frame_2.perform(context)], 
                'tcp_frames': [tcp_frame_1.perform(context), tcp_frame_2.perform(context)], 
                'traj_controllers': ["{}_traj_controller".format(xarm_type_1), "{}_traj_controller".format(xarm_type_2)],
            }
        ],
    )

    # Launch RViz
    rviz_config_file = (
        get_package_share_directory("manymove_planner") + "/config/micpp_demo_dual_app.rviz"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        # name='rviz2',
        output='screen',
        arguments=[
            "-d", rviz_config_file,
            "--ros-args",
            "--log-level", "rviz2:=fatal"
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_common2.launch.py
    # ================================================================

    xyz_1 = attach_xyz_1.perform(context).split(' ')
    rpy_1 = attach_rpy_1.perform(context).split(' ')
    xyz_2 = attach_xyz_2.perform(context).split(' ')
    rpy_2 = attach_rpy_2.perform(context).split(' ')
    args_1 = [
        '--x', xyz_1[0],
        '--y', xyz_1[1],
        '--z', xyz_1[2],
        '--roll', rpy_1[0],
        '--pitch', rpy_1[1],
        '--yaw', rpy_1[2],
        '--frame-id', attach_to_1.perform(context),
        '--child-frame-id', f"{prefix_1.perform(context)}link_base"
    ]
    args_2 = [
        '--x', xyz_2[0],
        '--y', xyz_2[1],
        '--z', xyz_2[2],
        '--roll', rpy_2[0],
        '--pitch', rpy_2[1],
        '--yaw', rpy_2[2],
        '--frame-id', attach_to_2.perform(context),
        '--child-frame-id', f"{prefix_2.perform(context)}link_base"
    ]

    # Static TF
    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_1.perform(context)),
        output='screen',
        arguments=args_1,
        parameters=[{'use_sim_time': use_sim_time}],
    )
    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_2.perform(context)),
        output='screen',
        arguments=args_2,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    controllers = [
        '{}{}_traj_controller'.format(prefix_1.perform(context), xarm_type_1),
        '{}{}_traj_controller'.format(prefix_2.perform(context), xarm_type_2),
    ]
    if add_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_1.perform(context), robot_type_1.perform(context)))
    elif add_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) == 'lite' and ros2_control_plugin.perform(context) !='isaac':
        controllers.append('{}lite_gripper_controller'.format(prefix_1.perform(context)))
    elif add_bio_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_1.perform(context)))
    if add_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_2.perform(context), robot_type_2.perform(context)))
    elif add_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) == 'lite' and ros2_control_plugin.perform(context) !='isaac':
        controllers.append('{}lite_gripper_controller'.format(prefix_2.perform(context)))
    elif add_bio_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_2.perform(context)))

    # ================================================================
    # from: xarm_controller/launch/_dual_ros2_control.launch.py
    # ================================================================

    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_api'), 'launch', 'lib', 'robot_api_lib.py'))
    generate_robot_api_params = getattr(mod, 'generate_robot_api_params')
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context), node_name='ufactory_driver'
    )

    # ros2 control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            ros2_control_params,
            robot_params,
        ],
        output='screen',
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
    # ================================================================

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    # Load controllers
    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    # ================================================================
    # launch manymove_object_manager
    # ================================================================

    # Object Manager node
    object_manager_node = Node(
        package='manymove_object_manager',
        executable='object_manager_node',
        name='object_manager_node',
        output='screen',
        parameters=[{'frame_id': 'world'}]
    )

    # ================================================================
    # launch manymove_hmi
    # ================================================================

    # HMI node
    manymove_hmi_node = Node(
        package='manymove_hmi',
        executable='manymove_hmi_app_executable',
        # name='manymove_hmi_node',
        output='screen',
        parameters=[{
            'robot_prefixes': [prefix_1.perform(context), prefix_2.perform(context)],
            'robot_names': [xarm_type_1, xarm_type_2],
        }]
    )

    # ================================================================
    # launch manymove_cpp_trees
    # ================================================================

    # behaviortree.cpp node
    manymove_cpp_trees_node = Node(
        package='manymove_cpp_trees',
        executable='collision_test',
        # name='manymove_cpp_tree_node',
        output='screen',
        parameters=[{
            'robot_model_1': xarm_type_1,
            'robot_prefix_1': prefix_1.perform(context),
            'tcp_frame_1': tcp_frame_1,
            'robot_model_2': xarm_type_2,
            'robot_prefix_2': prefix_2.perform(context),
            'tcp_frame_2': tcp_frame_2,
            'is_robot_real': False,
        }]
    )

    # ================================================================
    #  EVENT‑DRIVEN START‑UP ORDER
    # ================================================================

    # 1) start manymove action_server nodes ONLY when the *last* spawner exits
    # final action to run
    on_exit_actions = [ moveitcpp_action_servers_node ]

    # wrapping it in one RegisterEventHandler per spawner, from last→first
    for spawner in reversed(controller_nodes):
        on_exit_actions = [
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawner,
                    on_exit=on_exit_actions,
                )
            )
        ]

    # after the loop, on_exit_actions[0] is a nested RegisterEventHandler that
    # won’t fire until *every* spawner in controller_nodes has exited
    start_action_server_evt = on_exit_actions[0]

    # 2) start object_manager_node *and* cpp_trees when action‑server *starts*
    start_object_mgr_evt = RegisterEventHandler(
        OnProcessStart(
            target_action=moveitcpp_action_servers_node,
            on_start=[object_manager_node, manymove_cpp_trees_node],
        )
    )

    # 3) finally start HMI when cpp_trees has started
    start_hmi_evt = RegisterEventHandler(
        OnProcessStart(
            target_action=manymove_cpp_trees_node,
            on_start=[manymove_hmi_node],
        )
    )

    # ================================================================
    #  RETURN LIST
    #  (Immediate nodes + event‑handlers; late nodes omitted here)
    # ================================================================
    
    return [
        robot_state_publisher_node,
        rviz_node,
        static_tf_1,
        static_tf_2,
        ros2_control_node,
        joint_state_broadcaster,
        *controller_nodes,
        start_action_server_evt,
        start_object_mgr_evt,
        start_hmi_evt,
    ]


def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument('planner_type', default_value='moveitcpp', description='Type of planner to use: moveitcpp or movegroup'), #hardcoded
        DeclareLaunchArgument('velocity_scaling_factor', default_value='0.5', description='Velocity scaling factor'),
        DeclareLaunchArgument('acceleration_scaling_factor', default_value='0.5', description='Acceleration scaling factor'),
        DeclareLaunchArgument('max_exec_retries', default_value='5', description='Maximum number of retries'),
        DeclareLaunchArgument('smoothing_type', default_value='time_optimal', description='Smoothing type'),
        DeclareLaunchArgument('step_size', default_value='0.05', description='Step size'),
        DeclareLaunchArgument('jump_threshold', default_value='0.0', description='Jump threshold'),
        DeclareLaunchArgument('max_cartesian_speed', default_value='0.5', description='Max cartesian speed'),
        DeclareLaunchArgument('plan_number_target', default_value='8', description='Plan number target'),
        DeclareLaunchArgument('plan_number_limit', default_value='16', description='Plan number limit'),

        DeclareLaunchArgument('base_frame_1', default_value='link_base', description='Base frame of the robot 1'),
        DeclareLaunchArgument('tcp_frame_1', default_value='link_tcp', description='TCP (end effector) frame of the robot 1' ),
        DeclareLaunchArgument('base_frame_2', default_value='link_base', description='Base frame of the robot 2'),
        DeclareLaunchArgument('tcp_frame_2', default_value='link_tcp', description='TCP (end effector) frame of the robot 2' ),

        OpaqueFunction(function=launch_setup)
    ])
