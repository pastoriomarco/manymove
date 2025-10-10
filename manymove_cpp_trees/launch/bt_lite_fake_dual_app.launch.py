"""Launch description for the bt lite fake dual app scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description entry point."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_model_1', default_value='lite6', description='Model of the robot (e.g., lite6, uf850, xarm)'
            ),
            DeclareLaunchArgument(
                'robot_prefix_1', default_value='L_', description='Prefix for the name of the robot arms'
            ),
            DeclareLaunchArgument('tcp_frame_1', default_value='link_tcp', description='Name of the link to attach/detach objects to/from'),
            DeclareLaunchArgument(
                'robot_model_2', default_value='uf850', description='Model of the robot (e.g., lite6, uf850, xarm)'
            ),
            DeclareLaunchArgument(
                'robot_prefix_2', default_value='R_', description='Prefix for the name of the robot arms'
            ),
            DeclareLaunchArgument('tcp_frame_2', default_value='link_tcp', description='Name of the link to attach/detach objects to/from'),
            DeclareLaunchArgument(
                'is_robot_real',
                default_value='false',
                description='Set to true if connected with a real robot exposing the necessary services needed by manymove_signals',
            ),
            Node(
                package='manymove_cpp_trees',
                executable='bt_client_app_dual',
                name='manymove_cpp_trees_single_robot',
                output='screen',
                parameters=[
                    {
                        'robot_model_1': LaunchConfiguration('robot_model_1'),
                        'robot_prefix_1': LaunchConfiguration('robot_prefix_1'),
                        'tcp_frame_1': LaunchConfiguration('tcp_frame_1'),
                        'robot_model_2': LaunchConfiguration('robot_model_2'),
                        'robot_prefix_2': LaunchConfiguration('robot_prefix_2'),
                        'tcp_frame_2': LaunchConfiguration('tcp_frame_2'),
                        'is_robot_real': LaunchConfiguration('is_robot_real'),
                    }
                ],
            ),
        ]
    )
