# Copyright 2025 Flexin Group SRL
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Flexin Group SRL nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python3

"""Helpers for composing MoveManipulator requests and convenience moves."""

from dataclasses import dataclass, field
from typing import Dict
from typing import List
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from manymove_msgs.action import MoveManipulator
from manymove_msgs.msg import MoveManipulatorGoal
from manymove_msgs.msg import MovementConfig


@dataclass
class Move:
    """Represent a single request for the MoveManipulator action."""

    movement_type: str  # "pose", "joint", "named", "cartesian"
    tcp_frame: str
    pose_target: Optional[Pose] = None
    named_target: Optional[str] = None
    joint_values: Optional[List[float]] = None
    config: MovementConfig = field(default_factory=MovementConfig)

    def __post_init__(self):
        """Validate that the requested move type is supported."""
        allowed_types = ['pose', 'joint', 'named', 'cartesian']
        if self.movement_type not in allowed_types:
            raise ValueError(
                f"Unsupported movement_type '{self.movement_type}'. "
                f'Must be one of {allowed_types}.'
            )

    def to_move_manipulator_goal(self) -> MoveManipulatorGoal:
        """Convert this Move object into a MoveManipulatorGoal message."""
        mmg = MoveManipulatorGoal()
        mmg.movement_type = self.movement_type
        mmg.config = self.config

        mmg.config.tcp_frame = self.tcp_frame

        if self.movement_type in ['pose', 'cartesian']:
            if not isinstance(self.pose_target, Pose):
                raise TypeError(f"For '{self.movement_type}' moves, a valid Pose must be provided.")
            mmg.pose_target = self.pose_target

        elif self.movement_type == 'joint':
            if not isinstance(self.joint_values, list):
                raise TypeError("For 'joint' moves, a list of joint values must be provided.")
            mmg.joint_values = self.joint_values

        elif self.movement_type == 'named':
            if not isinstance(self.named_target, str):
                raise TypeError("For 'named' moves, a named_target (string) must be provided.")
            mmg.named_target = self.named_target

        return mmg


def define_movement_configs() -> Dict[str, MovementConfig]:
    """Create a handful of example MovementConfig presets used by the demos."""
    max_move_config = MovementConfig()
    max_move_config.velocity_scaling_factor = 1.0
    max_move_config.acceleration_scaling_factor = 1.0
    max_move_config.max_cartesian_speed = 0.5
    max_move_config.linear_precision = 0.001
    max_move_config.rotational_precision = 0.05
    max_move_config.deceleration_time = 0.5
    max_move_config.min_stop_time = 0.5
    max_move_config.step_size = 0.01
    max_move_config.jump_threshold = 0.0
    # Jazzy CartesianPrecision defaults (ignored on Humble)
    max_move_config.cartesian_precision_translational = 0.001
    max_move_config.cartesian_precision_rotational = 0.05
    max_move_config.cartesian_precision_max_resolution = 0.005
    max_move_config.plan_number_target = 8
    max_move_config.plan_number_limit = 32
    max_move_config.smoothing_type = 'time_optimal'

    mid_move_config = MovementConfig()
    mid_move_config.velocity_scaling_factor = 0.5
    mid_move_config.acceleration_scaling_factor = 0.5
    mid_move_config.max_cartesian_speed = 0.2
    mid_move_config.linear_precision = 0.001
    mid_move_config.rotational_precision = 0.05
    mid_move_config.deceleration_time = 0.5
    mid_move_config.min_stop_time = 0.5
    mid_move_config.step_size = 0.01
    mid_move_config.jump_threshold = 0.0
    mid_move_config.cartesian_precision_translational = 0.001
    mid_move_config.cartesian_precision_rotational = 0.05
    mid_move_config.cartesian_precision_max_resolution = 0.005
    mid_move_config.plan_number_target = 8
    mid_move_config.plan_number_limit = 32
    mid_move_config.smoothing_type = 'time_optimal'

    slow_move_config = MovementConfig()
    slow_move_config.velocity_scaling_factor = 0.25
    slow_move_config.acceleration_scaling_factor = 0.25
    slow_move_config.max_cartesian_speed = 0.05
    slow_move_config.linear_precision = 0.001
    slow_move_config.rotational_precision = 0.05
    slow_move_config.deceleration_time = 0.5
    slow_move_config.min_stop_time = 0.5
    slow_move_config.step_size = 0.01
    slow_move_config.jump_threshold = 0.0
    slow_move_config.cartesian_precision_translational = 0.001
    slow_move_config.cartesian_precision_rotational = 0.05
    slow_move_config.cartesian_precision_max_resolution = 0.005
    slow_move_config.plan_number_target = 8
    slow_move_config.plan_number_limit = 32
    slow_move_config.smoothing_type = 'time_optimal'

    return {
        'max_move': max_move_config,
        'mid_move': mid_move_config,
        'slow_move': slow_move_config,
    }


def create_pose(position: dict, orientation: dict) -> Pose:
    """
    Create a Pose from dictionaries describing position and orientation.

    Parameters
    ----------
    position : dict
        Mapping with ``x``, ``y`` and ``z`` entries.
    orientation : dict
        Mapping with quaternion ``x``, ``y``, ``z`` and ``w`` entries.

    Returns
    -------
    Pose
        Populated pose message derived from the supplied dictionaries.

    """
    return Pose(position=Point(**position), orientation=Quaternion(**orientation))


def create_move(
    movement_type: str,
    tcp_frame: str,
    target: Pose = None,
    named_target: str = None,
    joint_values: List[float] = None,
    config: MovementConfig = None,
) -> Move:
    """
    Build a Move dataclass instance from the provided target information.

    Parameters
    ----------
    movement_type : str
        Mode of motion such as ``pose``, ``cartesian``, ``joint`` or ``named``.
    tcp_frame : str
        Frame to treat as the tool centre point.
    target : Pose, optional
        Pose target for ``pose`` or ``cartesian`` moves.
    named_target : str, optional
        Predefined target for ``named`` moves.
    joint_values : List[float], optional
        Joint positions for ``joint`` moves.
    config : MovementConfig, optional
        Movement tuning parameters to attach to the request.

    Returns
    -------
    Move
        Populated Move dataclass ready to be converted into a goal.

    """
    if config is None:
        config = MovementConfig()
    return Move(
        movement_type=movement_type,
        tcp_frame=tcp_frame,
        pose_target=target,
        named_target=named_target,
        joint_values=joint_values,
        config=config,
    )


def build_move_manipulator_goal(move: Move) -> MoveManipulator.Goal:
    """Convert a Move instance into a MoveManipulator action goal."""
    action_goal = MoveManipulator.Goal()

    # Create the sub-message
    mmg = move.to_move_manipulator_goal()  # Reuse the method on Move
    action_goal.plan_request = mmg

    return action_goal


def send_move_manipulator_goal(node: Node, move: Move) -> bool:
    """
    Send a MoveManipulator goal and block until the action completes.

    Parameters
    ----------
    node : Node
        rclpy node used to create the action client and spin callbacks.
    move : Move
        Move description to forward to the action server.

    Returns
    -------
    bool
        ``True`` when the goal succeeds, ``False`` otherwise.

    """
    action_client = ActionClient(node, MoveManipulator, 'move_manipulator')

    node.get_logger().info("Waiting for 'move_manipulator' action server...")
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("'move_manipulator' action server not available.")
        return False

    # Build the final goal
    goal_msg = build_move_manipulator_goal(move)
    node.get_logger().info(f'Sending MoveManipulator goal [type={move.movement_type}] ...')

    # Send the goal
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error('MoveManipulator goal was rejected!')
        return False

    node.get_logger().info('MoveManipulator goal accepted; waiting for result...')
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    result = get_result_future.result()

    if not result or not result.result:
        node.get_logger().error('MoveManipulator action returned an invalid result object.')
        return False

    if result.result.success:
        node.get_logger().info(f'MoveManipulator Succeeded: {result.result.message}')
        return True
    else:
        node.get_logger().error(f'MoveManipulator Failed: {result.result.message}')
        return False
