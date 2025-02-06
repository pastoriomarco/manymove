import py_trees
import rclpy
from manymove_planner.action import MoveManipulatorSequence
from rclpy.action import ActionClient

class MoveSequenceBehaviour(py_trees.behaviour.Behaviour):
    """
    Behavior to execute a sequence of moves once, with optional retries if it fails.
    """

    def __init__(self, name, sequence_goal: MoveManipulatorSequence.Goal, node: rclpy.node.Node, max_retries=3):
        super().__init__(name)
        self.sequence_goal = sequence_goal
        self.node = node
        self.action_client = ActionClient(node, MoveManipulatorSequence, 'move_manipulator_sequence')
        self.goal_sent = False
        self.result_response = None
        self.max_retries = max_retries
        self.retry_count = 0

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Initializing MoveSequenceBehaviour.")
        self.goal_sent = False
        self.result_response = None
        self.retry_count = 0

    def update(self):
        # Ensure the action server is available
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error(f"[{self.name}] Action server 'move_manipulator_sequence' not available!")
            return py_trees.common.Status.FAILURE

        # Send the sequence goal once
        if not self.goal_sent:
            self.node.get_logger().info(f"[{self.name}] Sending MoveManipulatorSequence goal.")
            send_goal_future = self.action_client.send_goal_async(
                self.sequence_goal,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        # If we have a final result, interpret success/failure
        if self.result_response is not None:
            if self.result_response.result.success:
                self.node.get_logger().info(
                    f"[{self.name}] Sequence succeeded: {self.result_response.result.message}"
                )
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(
                    f"[{self.name}] Sequence failed: {self.result_response.result.message}"
                )
                if self.retry_count < self.max_retries:
                    self.retry_count += 1
                    self.node.get_logger().warn(
                        f"[{self.name}] Retrying sequence ({self.retry_count}/{self.max_retries})..."
                    )
                    # Reset to send the goal again
                    self.goal_sent = False
                    self.result_response = None
                    return py_trees.common.Status.RUNNING
                else:
                    self.node.get_logger().error(
                        f"[{self.name}] Sequence failed after {self.retry_count} retries. Aborting."
                    )
                    return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        """
        Called when the sequence goal is accepted or rejected.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"[{self.name}] Sequence goal was rejected by the action server.")
            # Simulate a final result indicating failure
            self.result_response = MoveManipulatorSequence.GetResult.Response()
            self.result_response.result.success = False
            self.result_response.result.message = "Sequence goal rejected by action server."
        else:
            self.node.get_logger().info(f"[{self.name}] Sequence goal accepted by the action server.")
            # Wait for the final result asynchronously
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Called when the final result from get_result_async() is ready.
        """
        self.result_response = future.result()

    def feedback_callback(self, feedback_msg):
        """
        Called whenever feedback is published by the action server for the sequence.
        """
        feedback = feedback_msg.feedback
        self.node.get_logger().info(
            f"[{self.name}] Sequence execution progress: {feedback.progress:.2f}"
        )
