#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
import py_trees
import json
import time  # <-- for sleep

class HMIServiceNode(Node):
    def __init__(self, node_name, blackboard=None, robot_prefix=""):
        super().__init__(node_name)
        self.robot_prefix = robot_prefix
        if blackboard is None:
            blackboard = py_trees.blackboard.Blackboard()
        self.blackboard = blackboard

        self.start_execution_srv = self.create_service(
            Empty, f"{self.robot_prefix}start_execution", self.handle_start_execution
        )
        self.stop_execution_srv = self.create_service(
            Empty, f"{self.robot_prefix}stop_execution", self.handle_stop_execution
        )
        self.reset_program_srv = self.create_service(
            Empty, f"{self.robot_prefix}reset_program", self.handle_reset_program
        )

        self.get_logger().info("HMI Service Node started.")

        self.status_publisher = self.create_publisher(
            String, f"{self.robot_prefix}blackboard_status", 10
        )

        # Publish every 250 ms
        self.create_timer(0.25, self.publish_blackboard_status)

    def handle_start_execution(self, request, response):
        self.blackboard.set(f"{self.robot_prefix}stop_execution", False)
        self.get_logger().info(
            f"start_execution: {self.robot_prefix}stop_execution=False, "
        )
        return response

    def handle_stop_execution(self, request, response):
        self.blackboard.set(f"{self.robot_prefix}stop_execution", True)
        self.get_logger().info(
            f"stop_execution: {self.robot_prefix}stop_execution=True"
        )
        return response

    def handle_reset_program(self, request, response):
        self.blackboard.set(f"{self.robot_prefix}stop_execution", True)
        self.blackboard.set(f"{self.robot_prefix}reset", True)
        self.get_logger().info(
            f"reset_program: {self.robot_prefix}stop_execution=True, "
            f"{self.robot_prefix}reset=True, "
        )
        return response

    def publish_blackboard_status(self):
        # Force a KeyError if not set.
        stop_execution = self.blackboard.get(f"{self.robot_prefix}stop_execution")
        reset = self.blackboard.get(f"{self.robot_prefix}reset")
        collision_detected = self.blackboard.get(f"{self.robot_prefix}collision_detected")

        msg_dict = {
            f"{self.robot_prefix}stop_execution": stop_execution,
            f"{self.robot_prefix}reset": reset,
            f"{self.robot_prefix}collision_detected": collision_detected
        }
        msg = String()
        msg.data = json.dumps(msg_dict)
        self.status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # 1) Create the node
    node = HMIServiceNode("hmi_service_node", robot_prefix="")

    # 2) Sleep for 2 seconds so that your BT or other code can set the blackboard keys
    time.sleep(2.0)
    node.get_logger().info("Waited 2 seconds, hopefully the Blackboard keys are now set.")

    # 3) Spin until shutdown
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
