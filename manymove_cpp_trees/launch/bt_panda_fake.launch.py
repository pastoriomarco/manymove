"""Launch description for the bt panda fake scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description entry point."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="panda_arm",
                description="Model of the robot (e.g., lite6, uf850, xarm)",
            ),
            DeclareLaunchArgument(
                "robot_prefix", default_value="", description="Prefix for the name of the robot arms"
            ),
            DeclareLaunchArgument(
                "tcp_frame",
                default_value="panda_link8",
                description="Name of the link to attach/detach objects to/from",
            ),
            DeclareLaunchArgument(
                "gripper_action_server",
                default_value="/panda_hand_controller/gripper_cmd",
                description="Name of the action server to control the gripper",
            ),
            DeclareLaunchArgument(
                "contact_links",
                default_value='["panda_leftfinger", "panda_rightfinger", "panda_hand"]',
                description="List of links to exclude from collision checking",
            ),
            DeclareLaunchArgument(
                "is_robot_real",
                default_value="false",
                description="Set to true if connected with a real robot exposing the necessary services needed by manymove_signals",
            ),
            Node(
                package="manymove_cpp_trees",
                executable="bt_client",
                name="manymove_cpp_trees_single_robot",
                output="screen",
                parameters=[
                    {
                        "robot_model": LaunchConfiguration("robot_model"),
                        "robot_prefix": LaunchConfiguration("robot_prefix"),
                        "tcp_frame": LaunchConfiguration("tcp_frame"),
                        "gripper_action_server": LaunchConfiguration("gripper_action_server"),
                        "contact_links": LaunchConfiguration("contact_links"),
                        "is_robot_real": LaunchConfiguration("is_robot_real"),
                    }
                ],
            ),
        ]
    )
