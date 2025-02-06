from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model_1', default_value='lite6', description='Model of the robot (e.g., lite6, uf850, xarm)'),
        DeclareLaunchArgument('robot_prefix_1', default_value='L_', description='Prefix for the name of the robot arms'),
        DeclareLaunchArgument('robot_model_2', default_value='uf850', description='Model of the robot (e.g., lite6, uf850, xarm)'),
        DeclareLaunchArgument('robot_prefix_2', default_value='R_', description='Prefix for the name of the robot arms'),
        DeclareLaunchArgument('is_robot_real', default_value='false', description='Set to true if connected with a real robot exposing the necessary services needed by manymove_signals'),

        Node(
            package='manymove_cpp_trees',
            executable='bt_client_dual',
            name='manymove_cpp_trees_single_robot',
            output='screen',
            parameters=[{
                'robot_model_1': LaunchConfiguration('robot_model_1'),
                'robot_prefix_1': LaunchConfiguration('robot_prefix_1'),
                'robot_model_2': LaunchConfiguration('robot_model_2'),
                'robot_prefix_2': LaunchConfiguration('robot_prefix_2'),
                'is_robot_real': LaunchConfiguration('is_robot_real')
            }]
        )
    ])
