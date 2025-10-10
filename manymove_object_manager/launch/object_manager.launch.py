"""Launch description for the object manager scenario."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description entry point."""
    return LaunchDescription(
        [
            Node(
                package='manymove_object_manager',
                executable='object_manager_node',
                name='object_manager_node',
                output='screen',
                parameters=[{'frame_id': 'world'}],
            )
        ]
    )
