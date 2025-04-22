import os
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    planning_group = LaunchConfiguration('planning_group')
    base_frame = LaunchConfiguration('base_frame')
    tcp_frame = LaunchConfiguration('tcp_frame')
    traj_controller = LaunchConfiguration('traj_controller')

    gripper_action_server = LaunchConfiguration('gripper_action_server'),
    contact_links = LaunchConfiguration('contact_links'),

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "dof": "6",
    }

    moveit_configs = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_configs.to_dict()],
    )

    # Define the action_server_node with new parameters
    action_server_node = Node(
        package='manymove_planner',
        executable='action_server_node',
        # Don't use the "name" parameter, the name will be automatically set with {node_prefix}action_server_node to avoid duplicate nodes
        output='screen',
        parameters=[
            moveit_configs.to_dict(),
            {
                'node_prefix': "{}_".format(planning_group.perform(context)),
                'planner_type': "movegroup",
                
                'planning_group': planning_group,
                'base_frame': base_frame,
                'tcp_frame': tcp_frame,
                'traj_controller': traj_controller,
            }
        ],
    )


    # RViz for visualization
    # Get the path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="kinova_moveit_config_demo.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d", rviz_config,
            "--ros-args",
            "--log-level", "rviz2:=fatal"
        ],
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
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", base_frame],
    )

        # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_configs.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_6dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_configs.robot_description, ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_trajectory_controller",
        "robotiq_gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
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
        parameters=[{'frame_id': 'world'}]
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
        parameters=[{
            'robot_prefixes': [""],
            'robot_names': ["Kinova_Gen3"],
        }]
    )

    # ================================================================
    # launch manymove_cpp_trees
    # ================================================================

    # behaviortree.cpp node
    manymove_cpp_trees_node = Node(
        package='manymove_cpp_trees',
        executable='bt_client_kinova',
        # name='manymove_cpp_tree_node',
        output='screen',
        parameters=[{
            'robot_model': planning_group,
            'robot_prefix': "",
            'tcp_frame': tcp_frame,
            'gripper_action_server': gripper_action_server,
            'contact_links': contact_links,
            'is_robot_real': False,
        }]
    )

    return [
        rviz_config_arg,
        ros2_control_hardware_type,
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        action_server_node,
        ros2_control_node,
        object_manager_node,
        manymove_hmi_node,
        manymove_cpp_trees_node
    ] + load_controllers

def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArguments for planning_group, base_frame, tcp_frame
        DeclareLaunchArgument('planning_group', default_value='manipulator', description='MoveIt planning group'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame of the robot'),
        DeclareLaunchArgument('tcp_frame', default_value='end_effector_link', description='TCP (end effector) frame of the robot' ),
        DeclareLaunchArgument('traj_controller', default_value='joint_trajectory_controller', description='traj_controller action server name of the robot' ),
        DeclareLaunchArgument('gripper_action_server', default_value='/robotiq_gripper_controller/gripper_cmd', description='Name of the action server to control the gripper'),
        DeclareLaunchArgument('contact_links', default_value='["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]', description='List of links to exclude from collision checking'),

        # OpaqueFunction to set up the node
        OpaqueFunction(function=launch_setup)
    ])


