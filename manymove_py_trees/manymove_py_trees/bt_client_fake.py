#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import py_trees
import py_trees_ros

import time
from geometry_msgs.msg import Pose, Point, Quaternion

# (1) Import the updated movement configs + create_move
from manymove_py_trees.move_definitions import (
    define_movement_configs,
    create_move,
)

# (2) Import the updated tree helper that uses MoveManipulatorBehavior nodes
from manymove_py_trees.tree_helper import create_tree_from_sequences

def main():
    rclpy.init()
    node = rclpy.create_node("bt_client_node")
    node.get_logger().info("BT Client Node started")

    # 1) Define move configurations
    movement_configs = define_movement_configs()

    # 2) Define the actual moves or sequences of moves
    joint_rest = [0.0, -0.785, 0.785, 0.0, 1.57, 0.0]
    joint_look_sx = [-0.175, -0.419, 1.378, 0.349, 1.535, -0.977]
    joint_look_dx = [0.733, -0.297, 1.378, -0.576, 1.692, 1.291]
    named_home = "home"

    pick_target = Pose(
        position=Point(x=0.2, y=-0.1, z=0.15),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )
    approach_target = Pose(
        position=Point(x=0.2, y=-0.1, z=0.17),  # pick_target.z + 0.02
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )

    rest_position = [
        create_move("joint", joint_values=joint_rest, config=movement_configs["max_move"]),
    ]

    scan_surroundings = [
        create_move("joint", joint_values=joint_look_sx, config=movement_configs["max_move"]),
        create_move("joint", joint_values=joint_look_dx, config=movement_configs["max_move"]),
    ]

    pick_sequence = [
        create_move("pose", target=approach_target, config=movement_configs["mid_move"]),
        create_move("cartesian", target=pick_target, config=movement_configs["slow_move"]),
        create_move("cartesian", target=approach_target, config=movement_configs["max_move"]),
    ]

    home_position = [
        create_move("named", named_target=named_home, config=movement_configs["max_move"]),
    ]

    # Build a list of sequences (each sequence is a list of Moves)
    list_of_sequences = [rest_position, scan_surroundings, pick_sequence, home_position]

    # 3) Create a Behavior Tree from these sequences
    #    This new version does NOT do concurrency or separate plan/execute:
    #    it simply runs each Move in a row by sending one move_manipulator goal.
    chained_branch = create_tree_from_sequences(
        node,
        list_of_sequences,
        root_name="LogicSequence"
    )

    # If desired, you can combine multiple subtrees or do more advanced composition:
    main_seq = py_trees.composites.Sequence("Main_Sequence", memory=True)
    main_seq.add_child(chained_branch.root)

    # Optionally, decorate with a "Repeat forever" for testing
    repeated_root = py_trees.decorators.Repeat(
        child=main_seq,
        num_success=-1,  # negative => infinite repeats
        name="RepeatForever"
    )

    bt_tree = py_trees_ros.trees.BehaviourTree(repeated_root)

    # 4) Setup
    try:
        bt_tree.setup(node=node, timeout=10.0)
    except Exception as e:
        node.get_logger().error(f"Failed to setup BT: {e}")
        rclpy.shutdown()
        return

    # 5) Tick until complete or failure
    try:
        while rclpy.ok():
            bt_tree.tick()
            status = bt_tree.root.status
            if status == py_trees.common.Status.SUCCESS:
                node.get_logger().info("Tree completed successfully.")
                break
            elif status == py_trees.common.Status.FAILURE:
                node.get_logger().error("Tree failed.")
                break

            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.001)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, exiting...")

    # 6) Shutdown
    bt_tree.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
