from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('manymove_object_manager'), 'config')
    objects_yaml = os.path.join(config_dir, 'objects.yaml')

    return LaunchDescription([
        Node(
            package='manymove_object_manager',
            executable='collision_spawner',
            name='collision_spawner',
            output='screen',
            parameters=[
                {'frame_id': 'world'},
                {'config_file': objects_yaml}
            ]
        )
    ])
