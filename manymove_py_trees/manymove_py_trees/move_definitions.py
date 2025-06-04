#!/usr/bin/env python3

from dataclasses import dataclass
from typing import List, Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, Point, Quaternion

# manymove_msgs definitions
from manymove_msgs.msg import MovementConfig, MoveManipulatorGoal
from manymove_msgs.action import MoveManipulator


@dataclass
class Move:
    """
    Dataclass representing a single move request for the 'move_manipulator' action.
    """
    movement_type: str  # "pose", "joint", "named", "cartesian"
    tcp_frame: str  
    pose_target: Optional[Pose] = None
    named_target: Optional[str] = None
    joint_values: Optional[List[float]] = None
    config: MovementConfig = MovementConfig()

    def __post_init__(self):
        allowed_types = ["pose", "joint", "named", "cartesian"]
        if self.movement_type not in allowed_types:
            raise ValueError(
                f"Unsupported movement_type '{self.movement_type}'. "
                f"Must be one of {allowed_types}."
            )

    def to_move_manipulator_goal(self) -> MoveManipulatorGoal:
        """
        Convert this Move object into a MoveManipulatorGoal message.
        """
        mmg = MoveManipulatorGoal()
        mmg.movement_type = self.movement_type
        mmg.config = self.config

        mmg.config.tcp_frame = self.tcp_frame

        if self.movement_type in ["pose", "cartesian"]:
            if not isinstance(self.pose_target, Pose):
                raise TypeError(
                    f"For '{self.movement_type}' moves, a valid Pose must be provided."
                )
            mmg.pose_target = self.pose_target

        elif self.movement_type == "joint":
            if not isinstance(self.joint_values, list):
                raise TypeError("For 'joint' moves, a list of joint values must be provided.")
            mmg.joint_values = self.joint_values

        elif self.movement_type == "named":
            if not isinstance(self.named_target, str):
                raise TypeError("For 'named' moves, a named_target (string) must be provided.")
            mmg.named_target = self.named_target

        return mmg


def define_movement_configs() -> Dict[str, MovementConfig]:
    """
    Define and return some example movement configurations for your robot.
    """
    max_move_config = MovementConfig()
    max_move_config.velocity_scaling_factor = 1.0
    max_move_config.acceleration_scaling_factor = 1.0
    max_move_config.max_cartesian_speed = 0.5
    max_move_config.linear_precision = 0.001
    max_move_config.rotational_precision = 0.05
    max_move_config.deceleration_time = 0.5
    max_move_config.step_size = 0.01
    max_move_config.jump_threshold = 0.0
    max_move_config.plan_number_target = 8
    max_move_config.plan_number_limit = 32
    max_move_config.smoothing_type = "time_optimal"

    mid_move_config = MovementConfig()
    mid_move_config.velocity_scaling_factor = 0.5
    mid_move_config.acceleration_scaling_factor = 0.5
    mid_move_config.max_cartesian_speed = 0.2
    mid_move_config.linear_precision = 0.001
    mid_move_config.rotational_precision = 0.05
    mid_move_config.deceleration_time = 0.5
    mid_move_config.step_size = 0.01
    mid_move_config.jump_threshold = 0.0
    mid_move_config.plan_number_target = 8
    mid_move_config.plan_number_limit = 32
    mid_move_config.smoothing_type = "time_optimal"
    
    slow_move_config = MovementConfig()
    slow_move_config.velocity_scaling_factor = 0.25
    slow_move_config.acceleration_scaling_factor = 0.25
    slow_move_config.max_cartesian_speed = 0.05
    slow_move_config.linear_precision = 0.001
    slow_move_config.rotational_precision = 0.05
    slow_move_config.deceleration_time = 0.5
    slow_move_config.step_size = 0.01
    slow_move_config.jump_threshold = 0.0
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
    Helper function to create a Pose from Python dictionaries, e.g.:
        position = {"x": 0.2, "y": 0.0, "z": 0.3}
        orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    """
    return Pose(
        position=Point(**position),
        orientation=Quaternion(**orientation)
    )


def create_move(
    movement_type: str,
    tcp_frame: str,
    target: Pose = None,
    named_target: str = None,
    joint_values: List[float] = None,
    config: MovementConfig = None
) -> Move:
    """
    Helper to create a Move object.

    Args:
        movement_type: "pose", "cartesian", "joint", or "named"
        target: a Pose for 'pose'/'cartesian'
        named_target: a string name for 'named'
        joint_values: list of angles for 'joint'
        config: MovementConfig for velocity, acceleration, etc.

    Returns:
        A Move instance.
    """
    if config is None:
        config = MovementConfig()

    return Move(
        movement_type=movement_type,
        tcp_frame=tcp_frame,
        pose_target=target,
        named_target=named_target,
        joint_values=joint_values,
        config=config
    )


def build_move_manipulator_goal(move: Move) -> MoveManipulator.Goal:
    """
    Convert a Move instance into a MoveManipulator action goal.

    The MoveManipulator action expects 'plan_request' to be a MoveManipulatorGoal message,
    which includes movement_type, pose_target, joint_values, etc.

    If your code doesn't call 'to_move_manipulator_goal()' on the Move object,
    you can call this function instead to produce the same data.
    """
    action_goal = MoveManipulator.Goal()

    # Create the sub-message
    mmg = move.to_move_manipulator_goal()  # Reuse the method on Move
    action_goal.plan_request = mmg

    return action_goal


def send_move_manipulator_goal(node: Node, move: Move) -> bool:
    """
    Helper function to send a single MoveManipulator goal to the 'move_manipulator' server
    and block until completion. Returns True if success, False otherwise.
    """
    action_client = ActionClient(node, MoveManipulator, 'move_manipulator')

    node.get_logger().info("Waiting for 'move_manipulator' action server...")
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("'move_manipulator' action server not available.")
        return False

    # Build the final goal
    goal_msg = build_move_manipulator_goal(move)
    node.get_logger().info(f"Sending MoveManipulator goal [type={move.movement_type}] ...")

    # Send the goal
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("MoveManipulator goal was rejected!")
        return False

    node.get_logger().info("MoveManipulator goal accepted; waiting for result...")
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    result = get_result_future.result()

    if not result or not result.result:
        node.get_logger().error("MoveManipulator action returned an invalid result object.")
        return False

    if result.result.success:
        node.get_logger().info(f"MoveManipulator Succeeded: {result.result.message}")
        return True
    else:
        node.get_logger().error(f"MoveManipulator Failed: {result.result.message}")
        return False
