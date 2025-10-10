"""Launch description for the bt lite fake scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description entry point."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_model', default_value='lite6', description='Model of the robot (e.g., lite6, uf850, xarm)'
            ),
            DeclareLaunchArgument(
                'robot_prefix', default_value='', description='Prefix for the name of the robot arms'
            ),
            DeclareLaunchArgument('tcp_frame', default_value='link_tcp', description='Name of the link to attach/detach objects to/from'),
            DeclareLaunchArgument(
                'is_robot_real',
                default_value='false',
                description='Set to true if connected with a real robot exposing the necessary services needed by manymove_signals',
            ),
            Node(
                package='manymove_cpp_trees',
                executable='bt_client',
                name='manymove_cpp_trees_single_robot',
                output='screen',
                parameters=[
                    {
                        'robot_model': LaunchConfiguration('robot_model'),
                        'robot_prefix': LaunchConfiguration('robot_prefix'),
                        'tcp_frame': LaunchConfiguration('tcp_frame'),
                        'is_robot_real': LaunchConfiguration('is_robot_real'),
                    }
                ],
            ),
        ]
    )
