import py_trees
import rclpy
from manymove_planner.action import MoveManipulator
from rclpy.action import ActionClient

class SingleMoveBehaviour(py_trees.behaviour.Behaviour):
    """
    Behavior to execute a single move.
    Sends the goal once and waits for the result.
    """

    def __init__(self, name, single_move_goal: MoveManipulator.Goal, node: rclpy.node.Node):
        super().__init__(name)
        self.single_move_goal = single_move_goal
        self.node = node
        self.action_client = ActionClient(node, MoveManipulator, 'move_manipulator')
        self.goal_sent = False
        self.result_response = None  # This will store the final get_result() response

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Initializing SingleMoveBehaviour.")
        self.goal_sent = False
        self.result_response = None

    def update(self):
        # Ensure the action server is available
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error(f"[{self.name}] Action server 'move_manipulator' not available!")
            return py_trees.common.Status.FAILURE

        # Send the goal once
        if not self.goal_sent:
            self.node.get_logger().info(f"[{self.name}] Sending SingleMove goal.")
            send_goal_future = self.action_client.send_goal_async(
                self.single_move_goal,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        # If we have a final result, interpret success/failure
        if self.result_response is not None:
            # Check our custom "success" field from the action definition
            if self.result_response.result.success:
                self.node.get_logger().info(f"[{self.name}] Single move succeeded: {self.result_response.result.message}")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(
                    f"[{self.name}] Single move failed: {self.result_response.result.message}"
                )
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        """
        Called when the goal is accepted/rejected by the action server.
        If accepted, we add a done callback for the final result.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"[{self.name}] SingleMove goal was rejected by the action server.")
            # We'll simulate a final result with success=False
            self.result_response = MoveManipulator.GetResult.Response()
            self.result_response.result.success = False
            self.result_response.result.message = "Goal rejected by action server."
        else:
            self.node.get_logger().info(f"[{self.name}] SingleMove goal accepted by the action server.")
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Called when the final result from get_result_async() is ready.
        """
        self.result_response = future.result()

    def feedback_callback(self, feedback_msg):
        """
        Called whenever feedback is published from the action server.
        """
        feedback = feedback_msg.feedback
        self.node.get_logger().info(
            f"[{self.name}] Single move progress: {feedback.progress:.2f}"
        )
