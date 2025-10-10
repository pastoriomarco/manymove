"""Launch description for the standalone micpp manymove planner scenario."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def generate_launch_description():
    """Create the launch description entry point."""
    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            controllers_name="fake_controllers",
            ros2_control_plugin="uf_robot_hardware/UFRobotFakeSystemHardware",
            context=LaunchContext(),
            robot_type="lite",
            dof=6,
            add_realsense_d435i=True,
            add_d435i_links=True,
            add_other_geometry=True,
            geometry_type="mesh",
            geometry_mass=0.3,
            geometry_mesh_filename="pneumatic_lite.stl",
            geometry_mesh_tcp_xyz="0.03075 0 0.11885",
            geometry_mesh_tcp_rpy="0 0.52 0",
            # kinematics_suffix="LS1"
        )
        .robot_description()
        .trajectory_execution(file_path="config/lite6/fake_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=get_package_share_directory("manymove_planner") + "/config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    run_moveit_cpp_node = Node(
        package="manymove_planner",
        executable="standalone_micpp_demo",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--log-level", "debug"],
    )

    # RViz
    rviz_config_file = get_package_share_directory("manymove_planner") + "/config/micpp_demo.rviz"

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("xarm_controller"),
        "config",
        "lite6_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lite6_traj_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_moveit_cpp_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
    )
