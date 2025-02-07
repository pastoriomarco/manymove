from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):

    velocity_scaling_factor = LaunchConfiguration('velocity_scaling_factor')
    acceleration_scaling_factor = LaunchConfiguration('acceleration_scaling_factor')
    max_exec_retries = LaunchConfiguration('max_exec_retries')
    smoothing_type = LaunchConfiguration('smoothing_type')
    step_size = LaunchConfiguration('step_size')
    jump_threshold = LaunchConfiguration('jump_threshold')
    max_cartesian_speed = LaunchConfiguration('max_cartesian_speed')
    plan_number_target = LaunchConfiguration('plan_number_target')
    plan_number_limit = LaunchConfiguration('plan_number_limit')

    planning_group = LaunchConfiguration('planning_group')
    base_frame = LaunchConfiguration('base_frame')
    tcp_frame = LaunchConfiguration('tcp_frame')
    tcp_frame = LaunchConfiguration('tcp_frame')
    traj_controller = LaunchConfiguration('traj_controller')

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_configs = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
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
                'node_prefix': "{}_".format(planning_group),
                'planner_type': "movegroup",
                'velocity_scaling_factor': velocity_scaling_factor,
                'acceleration_scaling_factor': acceleration_scaling_factor,
                'max_exec_retries': max_exec_retries,
                'smoothing_type': smoothing_type,
                'step_size': step_size,
                'jump_threshold': jump_threshold,
                'max_cartesian_speed': max_cartesian_speed,
                'plan_number_target': plan_number_target,
                'plan_number_limit': plan_number_limit,
                
                'planning_group': planning_group,
                'base_frame': base_frame,
                'tcp_frame': tcp_frame,
                'traj_controller': traj_controller,
            }
        ],
    )

    return [
        ros2_control_hardware_type,
        action_server_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArguments for movements defaults
        DeclareLaunchArgument('velocity_scaling_factor', default_value='0.5', description='Velocity scaling factor'),
        DeclareLaunchArgument('acceleration_scaling_factor', default_value='0.5', description='Acceleration scaling factor'),
        DeclareLaunchArgument('max_exec_retries', default_value='5', description='Maximum number of retries'),
        DeclareLaunchArgument('smoothing_type', default_value='time_optimal', description='Smoothing type'),
        DeclareLaunchArgument('step_size', default_value='0.05', description='Step size'),
        DeclareLaunchArgument('jump_threshold', default_value='0.0', description='Jump threshold'),
        DeclareLaunchArgument('max_cartesian_speed', default_value='0.5', description='Max cartesian speed'),
        DeclareLaunchArgument('plan_number_target', default_value='8', description='Plan number target'),
        DeclareLaunchArgument('plan_number_limit', default_value='32', description='Plan number limit'),
        
        # DeclareLaunchArguments for planning_group, base_frame, tcp_frame
        DeclareLaunchArgument('planning_group', default_value='panda_arm', description='MoveIt planning group'),
        DeclareLaunchArgument('base_frame', default_value='panda_link0', description='Base frame of the robot'),
        DeclareLaunchArgument('tcp_frame', default_value='panda_link8', description='TCP (end effector) frame of the robot' ),
        DeclareLaunchArgument('traj_controller', default_value='panda_arm_controller', description='traj_controller action server name of the robot' ),

        # OpaqueFunction to set up the node
        OpaqueFunction(function=launch_setup)
    ])
