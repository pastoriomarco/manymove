from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='lite6', description='Model of the robot (e.g., lite6, uf850, xarm)'),
        DeclareLaunchArgument('robot_prefix', default_value='', description='Prefix for the name of the robot arms'),
        DeclareLaunchArgument('is_robot_real', default_value='false', description='Set to true if connected with a real robot exposing the necessary services needed by manymove_signals'),

        Node(
            package='manymove_cpp_trees',
            executable='bt_client',
            name='manymove_cpp_trees_single_robot',
            output='screen',
            parameters=[{
                'robot_model': LaunchConfiguration('robot_model'),
                'robot_prefix': LaunchConfiguration('robot_prefix'),
                'is_robot_real': LaunchConfiguration('is_robot_real')
            }]
        )
    ])
