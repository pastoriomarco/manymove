import py_trees
import rclpy
from rclpy.action import ActionClient
from manymove_msgs.action import MoveManipulator
from moveit_msgs.msg import RobotTrajectory

class MoveManipulatorBehavior(py_trees.behaviour.Behaviour):
    """
    A Behavior Tree node that sends a single MoveManipulator goal and waits for completion.
    """

    def __init__(self, name, node, move,
                 robot_prefix="", 
                 blackboard=None,
                 blackboard_key=None):
        """
        :param name: BT node name (for logging)
        :param node: rclpy Node
        :param move: a 'Move' dataclass instance describing the request
        :param robot_prefix: optional robot prefix used in the server name
        :param blackboard: optional py_trees blackboard
        :param blackboard_key: if you want to store/retrieve an existing trajectory in the blackboard
        """
        super().__init__(name)
        self.node = node
        self.move = move
        self.blackboard = blackboard
        self.blackboard_key = blackboard_key

        # Build the final server name from prefix + "move_manipulator"
        server_name = robot_prefix + "move_manipulator"

        # The ActionClient now uses the prefix-based server name
        self._action_client = ActionClient(self.node, MoveManipulator, server_name)

        # Internal state
        self._goal_sent = False
        self._result_received = False
        self._goal_handle = None
        self._result = None

    def setup(self, **kwargs):
        """
        Called once when the tree is setup (before ticking).
        Optionally wait for the server here.
        """
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                f"[{self.name}] MoveManipulator server not available."
            )
        return super().setup(**kwargs)

    def initialise(self):
        """
        Called every time we transition from INVALID/IDLE to RUNNING.
        Reset any per-execution variables here.
        """
        self.node.get_logger().info(f"[{self.name}] initialise() - sending new goal")
        self._goal_sent = False
        self._result_received = False
        self._goal_handle = None
        self._result = None

    def update(self):
        """
        Called every BT tick while RUNNING.
        """
        # (1) If not yet sent a goal, do so now
        if not self._goal_sent:
            goal_msg = MoveManipulator.Goal()
            goal_msg.plan_request = self.move.to_move_manipulator_goal()

            if self.blackboard_key and self.blackboard:
                existing_traj = self.blackboard.get(self.blackboard_key)
                if isinstance(existing_traj, RobotTrajectory):
                    goal_msg.existing_trajectory = existing_traj

            # Send the goal
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            self._goal_sent = True
            return py_trees.common.Status.RUNNING

        # (2) If we have a final result, interpret success/failure
        if self._result_received:
            if self._result.success:
                self.node.get_logger().info(
                    f"[{self.name}] MoveManipulator SUCCEEDED: {self._result.message}"
                )
                # Optionally store final trajectory on blackboard
                if self.blackboard_key and self.blackboard:
                    self.blackboard.set(self.blackboard_key,
                                        self._result.final_trajectory)
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(
                    f"[{self.name}] MoveManipulator FAILED: {self._result.message}"
                )
                return py_trees.common.Status.FAILURE

        # Still waiting => RUNNING
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Called when the behavior transitions to a non-running state (SUCCESS/FAILURE)
        or if it is halted (INVALID).
        """
        if new_status == py_trees.common.Status.INVALID:
            # If the tree is halted mid-run, consider canceling the action
            if self._goal_handle and not self._result_received:
                self.node.get_logger().warn(
                    f"[{self.name}] Halting => canceling MoveManipulator goal."
                )
                self._goal_handle.cancel_goal_async()

    # ----------------------------------------------------------------
    # Callbacks:
    # ----------------------------------------------------------------

    def goal_response_callback(self, future):
        """
        Called once the server accepts/rejects the goal.
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().error(
                f"[{self.name}] Goal REJECTED by MoveManipulator server."
            )
            # Create a fake "result" so we can fail immediately
            from manymove_msgs.action._move_manipulator import MoveManipulator_Result
            fake_result = MoveManipulator_Result()
            fake_result.success = False
            fake_result.message = "Rejected"
            self._result = fake_result
            self._result_received = True
        else:
            self.node.get_logger().info(f"[{self.name}] Goal ACCEPTED.")
            get_result_future = self._goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Called when the final result is ready.
        """
        wrapped_result = future.result()
        self._result = wrapped_result.result
        self._result_received = True

    def feedback_callback(self, feedback_msg):
        """
        Periodic feedback from the MoveManipulator action server.
        """
        fb = feedback_msg.feedback
        self.node.get_logger().info(
            f"[{self.name}] feedback => progress={fb.progress:.2f}, in_collision={fb.in_collision}"
        )
