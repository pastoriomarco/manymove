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

"""Behavior-tree leaf that manages MoveManipulator action goals via py_trees."""

from rclpy.action import ActionClient

import py_trees
from manymove_msgs.action import MoveManipulator
from manymove_msgs.action._move_manipulator import MoveManipulator_Result
from moveit_msgs.msg import RobotTrajectory
from py_trees.common import Access


class MoveManipulatorBehavior(py_trees.behaviour.Behaviour):
    """Wrap the MoveManipulator action into a reusable py_trees behaviour."""

    def __init__(
        self,
        name: str,
        node,
        move,
        prefix: str = '',
        wait_for_server_seconds: float = 5.0,
    ):
        """Store configuration and initialise blackboard handles."""
        super().__init__(name=name)
        self.prefix = prefix
        self._node = node  # store the ROS2 node reference directly

        # Blackboard usage
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key('/node', access=Access.WRITE)
        self.bb.set('/node', node)

        self.bb.register_key(self.prefix + 'existing_trajectory', Access.READ)
        self.bb.register_key(self.prefix + 'existing_trajectory', Access.WRITE)
        self.bb.register_key(self.prefix + 'collision_detected', Access.READ)
        self.bb.register_key(self.prefix + 'collision_detected', Access.WRITE)
        self.bb.register_key(self.prefix + 'stop_execution', Access.READ)
        self.bb.register_key(self.prefix + 'stop_execution', Access.WRITE)

        self.bb.register_key(self.prefix + 'reset', Access.READ)
        self.bb.register_key(self.prefix + 'invalidate_traj_on_exec', Access.READ)

        self._wait_for_server_seconds = wait_for_server_seconds
        self._move_obj = move

        # Internal action-related state
        self._goal_sent = False
        self._result_received = False
        self._goal_handle = None
        self._action_result = None
        self._action_client = None

        # Internal logic states
        self._waiting_to_send = True
        self._paused = False
        self._cancel_in_progress = False

    def setup(self, **kwargs):
        """
        Initialise the action client and cache the ROS node from the blackboard.

        Raises
        ------
        RuntimeError
            Raised when the node is missing or the action server is unavailable.

        """
        self._node = self.bb.get('node')
        if not self._node:
            raise RuntimeError(f"[{self.name}] No rclpy Node found in blackboard under key 'node'")

        server_name = self.prefix + 'move_manipulator'
        self._action_client = ActionClient(
            self._node,
            MoveManipulator,
            server_name,
            callback_group=self._node.default_callback_group,
        )

        self._node.get_logger().info(
            (
                f'[{self.name}] waiting up to {self._wait_for_server_seconds}s '
                f"for action server '{server_name}'"
            )
        )
        if not self._action_client.wait_for_server(timeout_sec=self._wait_for_server_seconds):
            raise RuntimeError(f"[{self.name}] server '{server_name}' not available")

    def initialise(self):
        """Prepare behaviour state each time it transitions to RUNNING."""
        self._node.get_logger().info(f'[{self.name}] => initialise()')
        self._goal_sent = False
        self._result_received = False
        self._goal_handle = None
        self._action_result = None
        self._waiting_to_send = True
        self._paused = False
        self._cancel_in_progress = False

        if self._bb_get_bool('reset'):
            self._node.get_logger().warn(f'[{self.name}] reset => immediate FAIL in initialise()')
            self.stop(py_trees.common.Status.FAILURE)
            return

        if self._bb_get_bool('stop_execution'):
            self._paused = True

    def update(self):
        """Tick the behaviour until the MoveManipulator action finishes."""
        if self.status == py_trees.common.Status.FAILURE:
            return py_trees.common.Status.FAILURE

        # 1) Check 'reset' => immediate FAIL
        if self._bb_get_bool('reset'):
            self._node.get_logger().warn(f'[{self.name}] reset => FAIL mid-run')
            self._cancel_all_goals()
            self._clear_trajectory()
            self.stop(py_trees.common.Status.FAILURE)
            return py_trees.common.Status.FAILURE

        # 2) Check 'stop_execution' => pause
        if self._bb_get_bool('stop_execution'):
            self._paused = True
            if self._goal_sent and not self._result_received and not self._cancel_in_progress:
                self._node.get_logger().error(
                    f'[{self.name}] stop_execution => CANCEL => goal canceled'
                )
                self._cancel_all_goals()
                self._cancel_in_progress = True
        else:
            self._paused = False

        if self._paused:
            self._node.get_logger().debug(f'[{self.name}] => paused => RUNNING')
            return py_trees.common.Status.RUNNING

        # 3) collision_detected => if goal is active, cancel
        if self._bb_get_bool('collision_detected'):
            self.bb.set(self.prefix + 'collision_detected', False)
            self.bb.set(self.prefix + 'stop_execution', True)
            if self._goal_sent and not self._result_received and not self._cancel_in_progress:
                self._node.get_logger().error(
                    f'[{self.name}] collision => CANCEL => will retry next tick'
                )
                self._cancel_all_goals()
                self._cancel_in_progress = True
            return py_trees.common.Status.RUNNING

        # 4) If we haven't sent the goal yet => build & send
        if self._waiting_to_send and not self._goal_sent:
            goal_msg = self._build_goal()
            if not goal_msg:
                self._node.get_logger().error(
                    f'[{self.name}] failed to build goal => immediate FAILURE'
                )
                self.stop(py_trees.common.Status.FAILURE)
                return py_trees.common.Status.FAILURE

            self._node.get_logger().info(
                f"[{self.name}] sending MoveManipulator goal => prefix '{self.prefix}'"
            )
            send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)

            self._goal_sent = True
            self._waiting_to_send = False
            return py_trees.common.Status.RUNNING

        # 5) If we have the final result => interpret success/fail
        if self._result_received:
            if self._action_result.success:
                if self._bb_get_bool('invalidate_traj_on_exec'):
                    self._clear_trajectory()
                else:
                    if self._action_result.final_trajectory is not None:
                        self._store_trajectory(self._action_result.final_trajectory)

                self._node.get_logger().info(f'[{self.name}] => SUCCESS => returning SUCCESS')
                self.stop(py_trees.common.Status.SUCCESS)
                return py_trees.common.Status.SUCCESS
            else:
                self._node.get_logger().warn(
                    f'[{self.name}] => action FAIL => clearing trajectory => next tick => re-send'
                )
                self._clear_trajectory()
                self._goal_sent = False
                self._result_received = False
                self._action_result = None
                self._waiting_to_send = True
                return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Clean up state whenever the behaviour stops running."""
        if new_status == py_trees.common.Status.INVALID:
            self._cancel_all_goals()
            self._clear_trajectory()

    # -------------------------------------------------------
    # Helper Methods
    # -------------------------------------------------------
    def _bb_get_bool(self, suffix: str) -> bool:
        return bool(self.bb.get(self.prefix + suffix))

    def _build_goal(self) -> MoveManipulator.Goal:
        if not self._move_obj:
            self._node.get_logger().error(f'[{self.name}] no Move object found in constructor')
            return None

        mmg = self._move_obj.to_move_manipulator_goal()
        existing_traj = self.bb.get(self.prefix + 'existing_trajectory')
        if not isinstance(existing_traj, RobotTrajectory):
            existing_traj = RobotTrajectory()

        goal_msg = MoveManipulator.Goal()
        goal_msg.plan_request = mmg
        goal_msg.existing_trajectory = existing_traj
        return goal_msg

    def _clear_trajectory(self):
        self.bb.set(self.prefix + 'existing_trajectory', RobotTrajectory())

    def _store_trajectory(self, traj: RobotTrajectory):
        self.bb.set(self.prefix + 'existing_trajectory', traj)

    def _cancel_all_goals(self):
        if self._goal_handle is not None:
            self._node.get_logger().info(f'[{self.name}] Canceling current active goal')
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._cancel_done)
            self._cancel_in_progress = True
        else:
            self._node.get_logger().warn(f'[{self.name}] No active goal to cancel')

    def _cancel_done(self, future):
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            self._node.get_logger().info(f'[{self.name}] Goal successfully canceled.')
        else:
            self._node.get_logger().warn(
                f'[{self.name}] Goal failed to cancel or already completed.'
            )
        self._cancel_in_progress = False
        self._goal_sent = False
        self._result_received = False
        self._action_result = None
        self._waiting_to_send = True

    # -------------------------------------------------------
    # Action Callbacks
    # -------------------------------------------------------
    def goal_response_callback(self, future):
        """Handle responses from the MoveManipulator action server."""
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self._node.get_logger().error(f'[{self.name}] goal REJECTED => will fail in update()')

            self.bb.set(self.prefix + 'stop_execution', True)

            fake_res = MoveManipulator_Result()
            fake_res.success = False
            fake_res.message = 'Goal Rejected'
            self._action_result = fake_res
            self._result_received = True
        else:
            self._node.get_logger().info(f'[{self.name}] goal ACCEPTED => waiting for result')
            self._goal_handle = goal_handle
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle completion of the MoveManipulator goal."""
        wrapped_result = future.result()
        if not wrapped_result:
            self._node.get_logger().error(
                f'[{self.name}] result callback with no wrapped_result => fail'
            )
            fake_res = MoveManipulator_Result()
            fake_res.success = False
            fake_res.message = 'No result'
            self._action_result = fake_res
            self._result_received = True
            return

        # Directly use the returned result (no 'code' attribute)
        self._action_result = wrapped_result.result
        self._result_received = True

        self._result_received = True

    def feedback_callback(self, fb_msg):
        """React to feedback updates from the MoveManipulator action."""
        fb = fb_msg.feedback
        if fb.in_collision:
            self._node.get_logger().warn(
                (
                    f'[{self.name}] feedback => in_collision => setting '
                    f"'{self.prefix}collision_detected' = True"
                )
            )
            self.bb.set(self.prefix + 'collision_detected', True)
