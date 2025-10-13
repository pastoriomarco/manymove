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

"""Fake behaviour-tree client that runs prerecorded moves without robot hardware."""

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
    """Build the demonstration behaviour tree and tick it until completion."""
    node.get_logger().info('BT Client Node started')
    node.declare_parameter('tcp_frame', 'link_tcp')
    tcp_frame = node.get_parameter('tcp_frame').value

    bb = Blackboard()
    # Set defaults for keys used by MoveManipulatorBehavior:
    bb.set('reset', False)
    bb.set('stop_execution', False)
    bb.set('collision_detected', False)
    bb.set('invalidate_traj_on_exec', False)
    bb.set('existing_trajectory', RobotTrajectory())

    # 1) Define move configurations
    movement_configs = define_movement_configs()

    # 2) Define the actual moves or sequences of moves
    joint_rest = [0.0, -0.785, 0.785, 0.0, 1.57, 0.0]
    joint_look_sx = [-0.175, -0.419, 1.378, 0.349, 1.535, -0.977]
    joint_look_dx = [0.733, -0.297, 1.378, -0.576, 1.692, 1.291]
    named_home = 'home'

    pick_target = Pose(
        position=Point(x=0.2, y=-0.1, z=0.15),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )
    approach_target = Pose(
        position=Point(x=0.2, y=-0.1, z=0.17),  # pick_target.z + 0.02
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )

    rest_position = [
        create_move(
            'joint',
            tcp_frame,
            joint_values=joint_rest,
            config=movement_configs['max_move'],
        ),
    ]

    scan_surroundings = [
        create_move(
            'joint',
            tcp_frame,
            joint_values=joint_look_sx,
            config=movement_configs['max_move'],
        ),
        create_move(
            'joint',
            tcp_frame,
            joint_values=joint_look_dx,
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

    # Build a list of sequences
    list_of_sequences = [rest_position, scan_surroundings, pick_sequence, home_position]

    # 3) Create a BT from these sequences
    chained_branch = create_tree_from_sequences(node, list_of_sequences, root_name='LogicSequence')

    main_seq = py_trees.composites.Sequence('Main_Sequence', memory=True)
    main_seq.add_child(chained_branch.root)

    repeated_root = py_trees.decorators.Repeat(
        child=main_seq,
        num_success=-1,
        name='RepeatForever',  # negative => infinite repeats
    )

    bt_tree = py_trees_ros.trees.BehaviourTree(repeated_root)

    # 4) Setup
    try:
        bt_tree.setup(node=node, timeout=10.0)
    except Exception as e:
        node.get_logger().error(f'Failed to setup BT: {e}')
        return  # early return => won't do the main loop

    # 5) Manual tick loop
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

            # This ensures we process any pending ROS events (service calls, etc.)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.001)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, exiting...')

    # 6) Shutdown
    bt_tree.shutdown()


def main():
    """Entry point that spins the fake BT client alongside the HMI service node."""
    # Initialize rclpy
    rclpy.init()

    # We'll use a MultiThreadedExecutor so that the HMI service node
    # can handle incoming service calls while we do manual ticking.
    executor = MultiThreadedExecutor(num_threads=2)

    # Create a Node for the BT logic
    bt_node = rclpy.create_node('bt_client_node')

    # Create the HMI service node in the same process
    hmi_node = HMIServiceNode(node_name='hmi_service_node', robot_prefix='')
    hmi_node.get_logger().info('HMI service node created in the same process')

    # Add both nodes to the executor
    executor.add_node(bt_node)
    executor.add_node(hmi_node)

    # We'll run the BT logic in "build_and_run_bt" but we also want
    # the HMI node to be active. The easiest approach:
    #  1) Start the executor in a background thread
    #  2) Then do the BT loop in this main thread
    import threading

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Now do the manual BT tick in the main thread
    build_and_run_bt(bt_node)

    # Done => shut down
    bt_node.destroy_node()
    hmi_node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()
