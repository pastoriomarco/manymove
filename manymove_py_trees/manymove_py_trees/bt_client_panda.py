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
"""Demo behaviour-tree client for the Franka Panda manipulator."""

import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import py_trees
import py_trees_ros
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from manymove_py_trees.hmi_service_node import HMIServiceNode
from manymove_py_trees.move_definitions import create_move
from manymove_py_trees.move_definitions import define_movement_configs
from manymove_py_trees.tree_helper import create_tree_from_sequences
from moveit_msgs.msg import RobotTrajectory
from py_trees.blackboard import Blackboard


def build_and_run_bt(node: Node):
    """Create and tick the Panda demonstration behaviour tree."""
    node.get_logger().info('BT Client Node started')
    node.declare_parameter('tcp_frame', 'panda_link8')
    tcp_frame = node.get_parameter('tcp_frame').value

    # Create and initialize the Blackboard with the keys used by the behavior
    bb = Blackboard()
    bb.set('reset', False)
    bb.set('stop_execution', False)
    bb.set('collision_detected', False)
    bb.set('invalidate_traj_on_exec', False)
    bb.set('existing_trajectory', RobotTrajectory())

    # 1) Define the movement configurations
    movement_configs = define_movement_configs()

    # 2) Define your Panda-specific move sequences.
    # (These values are examples—adapt joint values, poses, and named targets as needed.)
    joint_rest = [0.0, -0.785, 0.0, -2.355, 0.0, 3.14, 0.785]
    named_home = 'ready'

    pick_target = Pose(
        position=Point(x=0.3, y=0.3, z=0.25),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )
    approach_target = Pose(
        position=Point(
            x=pick_target.position.x,
            y=pick_target.position.y,
            z=pick_target.position.z + 0.02,
        ),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )

    # Define three sequences: rest, pick, and home.
    rest_position = [
        create_move(
            'joint',
            tcp_frame,
            joint_values=joint_rest,
            config=movement_configs['max_move'],
        ),
    ]
    pick_sequence = [
        create_move(
            'pose',
            tcp_frame,
            target=approach_target,
            config=movement_configs['mid_move'],
        ),
        create_move(
            'cartesian',
            tcp_frame,
            target=pick_target,
            config=movement_configs['slow_move'],
        ),
        create_move(
            'cartesian',
            tcp_frame,
            target=approach_target,
            config=movement_configs['max_move'],
        ),
    ]
    home_position = [
        create_move(
            'named',
            tcp_frame,
            named_target=named_home,
            config=movement_configs['max_move'],
        ),
    ]

    # Build a list of sequences and the Behavior Tree (BT)
    list_of_sequences = [rest_position, pick_sequence, home_position]
    chained_branch = create_tree_from_sequences(node, list_of_sequences, root_name='LogicSequence')
    main_seq = py_trees.composites.Sequence('Main_Sequence', memory=True)
    main_seq.add_child(chained_branch.root)
    repeated_root = py_trees.decorators.Repeat(
        child=main_seq, num_success=-1, name='RepeatForever'
    )  # infinite repeat
    bt_tree = py_trees_ros.trees.BehaviourTree(repeated_root)

    # Setup the BT (which will read the rclpy Node from the blackboard)
    try:
        bt_tree.setup(node=node, timeout=10.0)
    except Exception as e:
        node.get_logger().error(f'Failed to setup BT: {e}')
        return

    # Manual tick loop: tick the BT while processing pending ROS events
    try:
        while rclpy.ok():
            bt_tree.tick()
            status = bt_tree.root.status
            if status == py_trees.common.Status.SUCCESS:
                node.get_logger().info('Tree completed successfully.')
                break
            elif status == py_trees.common.Status.FAILURE:
                node.get_logger().error('Tree failed.')
                break
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.001)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, exiting...')

    bt_tree.shutdown()


def main():
    """Entry point that spins the Panda BT client alongside the HMI service node."""
    rclpy.init()

    # Use a MultiThreadedExecutor so that the HMI service node and BT node can run concurrently
    executor = MultiThreadedExecutor(num_threads=2)

    # Create the BT node for the panda client
    bt_node = rclpy.create_node('bt_client_node')

    # Create the HMI service node (with empty prefix in this example)
    hmi_node = HMIServiceNode(node_name='hmi_service_node', robot_prefix='')
    hmi_node.get_logger().info('HMI service node created in the same process')

    # Add both nodes to the executor
    executor.add_node(bt_node)
    executor.add_node(hmi_node)

    # Run the executor in a background thread
    import threading

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Run the BT tick loop in the main thread
    build_and_run_bt(bt_node)

    # Cleanup: destroy nodes and shutdown executor
    bt_node.destroy_node()
    hmi_node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()
