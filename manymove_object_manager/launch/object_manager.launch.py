from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manymove_object_manager',
            executable='object_manager_node',
            name='object_manager_node',
            output='screen',
            parameters=[{'frame_id': 'world'}]
        )
    ])
