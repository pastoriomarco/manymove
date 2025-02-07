import py_trees
import rclpy
from manymove_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient

class ExecuteTrajectoryBehaviour(py_trees.behaviour.Behaviour):
    """
    Behavior to execute a preplanned trajectory.
    """

    def __init__(self, name, blackboard, blackboard_key: str, validity_key: str, node: rclpy.node.Node):
        super().__init__(name)
        self.blackboard = blackboard
        self.blackboard_key = blackboard_key
        self.validity_key = validity_key
        self.node = node
        self.action_client = ActionClient(node, ExecuteTrajectory, 'execute_manipulator_traj')
        self.goal_sent = False
        self.result_response = None

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Initializing ExecuteTrajectoryBehaviour.")
        self.goal_sent = False
        self.result_response = None

    def update(self):
        # Access the blackboard to get the planned trajectory and validity
        trajectory = self.blackboard.get(self.blackboard_key)
        validity = self.blackboard.get(self.validity_key)

        # Check if the trajectory exists
        if trajectory is None:
            self.node.get_logger().error(f"[{self.name}] Trajectory not available in the blackboard.")
            return py_trees.common.Status.FAILURE

        # Check if the trajectory is valid
        if not validity:
            self.node.get_logger().error(f"[{self.name}] Trajectory is marked as invalid.")
            return py_trees.common.Status.FAILURE

        # Ensure the action server is available
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error(f"[{self.name}] Action server 'execute_manipulator_traj' not available!")
            return py_trees.common.Status.FAILURE

        if not self.goal_sent:
            self.node.get_logger().info(f"[{self.name}] Sending ExecuteTrajectory goal.")
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = trajectory
            send_goal_future = self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        # If we have a final result, interpret success or failure
        if self.result_response is not None:
            if self.result_response.result.success:
                self.node.get_logger().info(f"[{self.name}] Trajectory execution succeeded.")
                self.blackboard.set(self.validity_key, False)  # Invalidate trajectory after execution
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(f"[{self.name}] Trajectory execution failed.")
                self.blackboard.set(self.validity_key, False)  # Invalidate trajectory on failure
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        """
        Called when the goal is accepted/rejected by the action server.
        If accepted, we add a done callback for the final result.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"[{self.name}] ExecuteTrajectory goal was rejected by the action server.")
            self.result_response = ExecuteTrajectory.GetResult.Response()
            self.result_response.result.success = False
        else:
            self.node.get_logger().info(f"[{self.name}] Execution goal accepted.")
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Called when the final result from get_result_async() is ready.
        """
        self.result_response = future.result()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Use the updated field names here
        # self.node.get_logger().info(
        #     f"[{self.name}] Execution progress: {feedback.progress:.3f}, in_collision: {feedback.in_collision}"
        # )
