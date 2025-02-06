from dataclasses import dataclass
from typing import List, Optional
from manymove_planner.msg import MovementConfig, MoveManipulatorGoal
from manymove_planner.action import PlanManipulator
from manymove_planner.action import ExecuteTrajectory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion

@dataclass
class Move:
    """
    Dataclass representing a single move.
    """
    movement_type: str  # "pose", "joint", "named", "cartesian"
    pose_target: Optional[Pose] = None
    named_target: Optional[str] = None
    joint_values: Optional[List[float]] = None
    config: MovementConfig = MovementConfig()

def define_movement_configs() -> dict:
    """Define and return movement configurations."""
    max_move_config = MovementConfig()
    max_move_config.velocity_scaling_factor = 1.0
    max_move_config.acceleration_scaling_factor = 1.0
    max_move_config.step_size = 0.01
    max_move_config.jump_threshold = 0.0
    max_move_config.max_cartesian_speed = 0.5
    max_move_config.max_exec_tries = 5
    max_move_config.plan_number_target = 8
    max_move_config.plan_number_limit = 32
    max_move_config.smoothing_type = "time_optimal"

    mid_move_config = MovementConfig()
    mid_move_config.velocity_scaling_factor = max_move_config.velocity_scaling_factor / 2.0 
    mid_move_config.acceleration_scaling_factor = max_move_config.acceleration_scaling_factor / 2.0 
    mid_move_config.step_size = 0.01
    mid_move_config.jump_threshold = 0.0
    mid_move_config.max_cartesian_speed = 0.2
    mid_move_config.max_exec_tries = 5
    mid_move_config.plan_number_target = 8
    mid_move_config.plan_number_limit = 32
    mid_move_config.smoothing_type = "time_optimal"

    slow_move_config = MovementConfig()
    slow_move_config.velocity_scaling_factor = max_move_config.velocity_scaling_factor / 40 
    slow_move_config.acceleration_scaling_factor = max_move_config.acceleration_scaling_factor / 4.0 
    slow_move_config.step_size = 0.01
    slow_move_config.jump_threshold = 0.0
    slow_move_config.max_cartesian_speed = 0.05
    slow_move_config.max_exec_tries = 5
    slow_move_config.plan_number_target = 8
    slow_move_config.plan_number_limit = 32
    slow_move_config.smoothing_type = "time_optimal"

    return {
        "max_move": max_move_config,
        "mid_move": mid_move_config,
        "slow_move": slow_move_config
    }

def create_pose(position: dict, orientation: dict) -> Pose:
    """
    Helper function to create a Pose message from position and orientation dictionaries.
    """
    return Pose(
        position=Point(**position),
        orientation=Quaternion(**orientation)
    )

def create_move(movement_type: str, target=None, named_target=None, joint_values=None, config: MovementConfig = None) -> Move:
    """
    Helper function to create a Move instance.
    """
    if config is None:
        config = MovementConfig()  # Default configuration

    return Move(
        movement_type=movement_type,
        pose_target=target,
        named_target=named_target,
        joint_values=joint_values,
        config=config
    )

def define_single_move_goal(move: Move) -> PlanManipulator.Goal:
    """
    Define and return a single MoveManipulator goal based on the Move object.
    """
    single_goal = PlanManipulator.Goal()
    single_goal.goal.movement_type = move.movement_type

    if move.movement_type in ["pose", "cartesian"]:
        if not isinstance(move.pose_target, Pose):
            raise TypeError(f"Target must be a Pose instance for movement type '{move.movement_type}'.")
        single_goal.goal.pose_target = move.pose_target
    elif move.movement_type == "joint":
        if not isinstance(move.joint_values, list):
            raise TypeError("Target must be a list of joint values for movement type 'joint'.")
        single_goal.goal.joint_values = move.joint_values  # move.joint_values is a list
    elif move.movement_type == "named":
        if not isinstance(move.named_target, str):
            raise TypeError("Target must be a string for movement type 'named'.")
        single_goal.goal.named_target = move.named_target  # move.named_target is a string
    else:
        raise ValueError(f"Unsupported movement type: {move.movement_type}")

    single_goal.goal.config = move.config

    return single_goal

def call_planning_service(node: Node, goal: MoveManipulatorGoal) -> Optional[RobotTrajectory]:
    """
    Call the planning service to generate a trajectory.
    """
    client = node.create_client(PlanManipulator, 'plan_manipulator')
    if not client.wait_for_service(timeout_sec=3.0):
        node.get_logger().error("Planning service 'plan_manipulator' not available.")
        return None

    request = PlanManipulator.Request()
    request.goal = goal

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None and future.result().success:
        node.get_logger().info("Planning service succeeded.")
        return future.result().trajectory
    else:
        node.get_logger().error("Planning service failed.")
        return None


def send_execute_trajectory(node: Node, trajectory: RobotTrajectory) -> bool:
    """
    Send a trajectory to the execute trajectory action server.
    """
    action_client = ActionClient(node, ExecuteTrajectory, 'execute_manipulator_traj')
    if not action_client.wait_for_server(timeout_sec=3.0):
        node.get_logger().error("Execute Trajectory action server not available.")
        return False

    goal_msg = ExecuteTrajectory.Goal()
    goal_msg.trajectory = trajectory

    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("Execute Trajectory goal rejected.")
        return False

    node.get_logger().info("Execute Trajectory goal accepted.")
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)

    result = get_result_future.result()
    if result.result.success:
        node.get_logger().info("Trajectory execution succeeded.")
        return True
    else:
        node.get_logger().error(f"Trajectory execution failed: {result.result.message}")
        return False