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

"""Expose services and helpers for manipulating behaviour-tree blackboard state."""

import json
import time

import rclpy
from rclpy.node import Node

import py_trees
from geometry_msgs.msg import Pose
from manymove_msgs.srv import SetBlackboardValues
from std_msgs.msg import String


def create_pose_from_dict(d):
    """Convert a JSON-like dictionary into a Pose message with an identity quaternion."""
    pose = Pose()
    pose.position.x = d.get('x', 0.0)
    pose.position.y = d.get('y', 0.0)
    pose.position.z = d.get('z', 0.0)
    # Here, we simply set an identity quaternion.
    # For proper conversion, you could use tf_transformations.quaternion_from_euler.
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose


class HMIServiceNode(Node):
    """Expose services and status topics used to adjust the behaviour-tree blackboard."""

    def __init__(self, node_name, blackboard=None, robot_prefix=''):
        """Initialize publishers and services for behaviour tree interaction."""
        super().__init__(node_name)
        self.robot_prefix = robot_prefix
        if blackboard is None:
            blackboard = py_trees.blackboard.Blackboard()
        self.blackboard = blackboard

        # Create a single "update_blackboard" service using the new service type.
        self.update_blackboard_srv = self.create_service(
            SetBlackboardValues, 'update_blackboard', self.handle_update_blackboard
        )

        self.get_logger().info('HMI Service Node started (Python version).')

        # Create a publisher for blackboard status on topic "blackboard_status".
        self.status_publisher = self.create_publisher(String, 'blackboard_status', 10)

        # Publish status every 250 ms.
        self.create_timer(0.25, self.publish_blackboard_status)

    def handle_update_blackboard(self, request, response):
        """Apply SetBlackboardValues requests onto the shared py_trees blackboard."""
        n = len(request.key)
        if len(request.value_type) != n or len(request.value_data) != n:
            response.success = False
            response.message = 'Mismatched array lengths among key, value_type, and value_data.'
            self.get_logger().error(response.message)
            return response

        for i in range(n):
            key = request.key[i]
            value_type = request.value_type[i]
            data_str = request.value_data[i]
            self.get_logger().info(f"Updating BB key='{key}' type='{value_type}' data='{data_str}'")
            try:
                if value_type == 'bool':
                    # Expect data_str to be a JSON string "true" or "false"
                    val = json.loads(data_str.lower())
                    self.blackboard.set(key, val)
                elif value_type == 'double':
                    val = float(data_str)
                    self.blackboard.set(key, val)
                elif value_type == 'string':
                    self.blackboard.set(key, data_str)
                elif value_type == 'double_array':
                    # Expect a JSON array like "[0.01,0.01,0.25]"
                    arr = json.loads(data_str)
                    if not isinstance(arr, list):
                        raise ValueError('Expected a list for double_array')
                    # Ensure all values are floats
                    arr = [float(x) for x in arr]
                    self.blackboard.set(key, arr)
                elif value_type == 'pose':
                    # Expect a JSON object like
                    # {"x":0.1,"y":0.2,"z":0.3,"roll":1.57,"pitch":0.0,"yaw":0.0}
                    d = json.loads(data_str)
                    if not isinstance(d, dict):
                        raise ValueError('Expected a dict for pose')
                    pose = create_pose_from_dict(d)
                    self.blackboard.set(key, pose)
                else:
                    raise ValueError(f'Unsupported value_type: {value_type}')
            except Exception as e:
                self.get_logger().error(f"Error updating key='{key}': {str(e)}")
        response.success = True
        response.message = f'Updated {n} blackboard keys'
        return response

    def publish_blackboard_status(self):
        """Publish the current blackboard state so that HMI clients can reflect it."""
        msg = String()
        if self.robot_prefix == '':
            # Global keys: "stop_execution", "reset", "collision_detected"
            stop_execution = self.blackboard.get('stop_execution')
            reset = self.blackboard.get('reset')
            collision_detected = self.blackboard.get('collision_detected')
            msg_dict = {
                'stop_execution': stop_execution,
                'reset': reset,
                'collision_detected': collision_detected,
            }
        else:
            # For a specific robot prefix, expect keys like "L_stop_execution", etc.
            stop_execution = self.blackboard.get(f'{self.robot_prefix}stop_execution')
            reset = self.blackboard.get(f'{self.robot_prefix}reset')
            collision_detected = self.blackboard.get(f'{self.robot_prefix}collision_detected')
            msg_dict = {
                f'{self.robot_prefix}stop_execution': stop_execution,
                f'{self.robot_prefix}reset': reset,
                f'{self.robot_prefix}collision_detected': collision_detected,
            }
        msg.data = json.dumps(msg_dict)
        self.status_publisher.publish(msg)


def main(args=None):
    """Spin the HMI service node as a standalone executable."""
    rclpy.init(args=args)

    # Create the node with an empty prefix (global) or a specific one ("L_", "R_")
    node = HMIServiceNode('hmi_service_node', robot_prefix='')
    time.sleep(2.0)
    node.get_logger().info('Waited 2 seconds, hopefully the Blackboard keys are now set.')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
