import py_trees
from manymove_planner.action import PlanManipulator  # generated from PlanManipulator.action
from rclpy.action import ActionClient

class PlanningActionBehaviour(py_trees.behaviour.Behaviour):
    """
    Behavior to call the PlanManipulator action and store the result in the blackboard.
    Replaces the old planning service approach with an asynchronous action call.
    """

    def __init__(self, name, goal, blackboard, blackboard_key, validity_key, previous_key=None, node=None):
        super().__init__(name)
        self.goal = goal  # a MoveManipulatorGoal
        self.blackboard = blackboard
        self.blackboard_key = blackboard_key
        self.validity_key = validity_key
        self.previous_key = previous_key
        self.node = node

        # Replace the old service client with an action client
        self.action_client = ActionClient(node, PlanManipulator, 'plan_manipulator')
        self.goal_handle = None
        self.result_response = None
        self.goal_sent = False

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Initializing PlanningActionBehaviour.")
        self.goal_handle = None
        self.result_response = None
        self.goal_sent = False

    def update(self):
        # 1) Ensure the action server is available
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error(f"[{self.name}] Action server 'plan_manipulator' not available.")
            return py_trees.common.Status.FAILURE

        # 2) If we haven't sent the goal yet, do so now
        if not self.goal_sent:
            # If we need to chain from a previous trajectory:
            if self.previous_key:
                prev_traj = self.blackboard.get(self.previous_key)
                if prev_traj and len(prev_traj.joint_trajectory.points) > 0:
                    end_positions = prev_traj.joint_trajectory.points[-1].positions
                    self.goal.start_joint_values = end_positions
                else:
                    self.node.get_logger().error(f"[{self.name}] Previous trajectory invalid.")
                    return py_trees.common.Status.FAILURE

            # Construct the action goal
            action_goal = PlanManipulator.Goal()
            action_goal.goal = self.goal  # MoveManipulatorGoal

            self.node.get_logger().info(f"[{self.name}] Sending PlanManipulator action goal.")
            send_goal_future = self.action_client.send_goal_async(
                action_goal,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        # 3) If we have a final result, decide SUCCESS/FAILURE
        if self.result_response is not None:
            if self.result_response.result.success:
                self.node.get_logger().info(f"[{self.name}] Planning Succeeded.")
                # Store the resulting trajectory
                self.blackboard.set(self.blackboard_key, self.result_response.result.trajectory)
                self.blackboard.set(self.validity_key, True)
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(f"[{self.name}] Planning Failed.")
                self.blackboard.set(self.validity_key, False)
                return py_trees.common.Status.FAILURE

        # 4) Otherwise, still waiting
        return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        """Called once the action server accepts/rejects the goal."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().error(f"[{self.name}] Planning goal was rejected by the action server.")
            # Simulate a final result with success=False
            fake_response = PlanManipulator.GetResult.Response()
            fake_response.result.success = False
            self.result_response = fake_response
        else:
            self.node.get_logger().info(f"[{self.name}] Planning goal accepted.")
            get_result_future = self.goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Called when the final result from get_result_async() is ready."""
        self.result_response = future.result()

    def feedback_callback(self, feedback_msg):
        """Optional feedback from the PlanManipulator action."""
        feedback = feedback_msg.feedback
        progress = feedback.progress
        self.node.get_logger().info(f"[{self.name}] PlanManipulator progress: {progress:.2f}")
