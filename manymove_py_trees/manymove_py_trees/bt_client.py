#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import py_trees
import py_trees_ros

from geometry_msgs.msg import Pose, Point, Quaternion

from manymove_py_trees.move_definitions import (
    define_movement_configs,
    create_move,
)
from manymove_py_trees.tree_helper import create_tree_from_sequences, create_tree_from_sequence, create_tree_from_sequence_v2 

import time

def main():
    rclpy.init()
    node = rclpy.create_node("bt_client_node")
    node.get_logger().info("BT Client Node started")

    # 1) Define configs
    movement_configs = define_movement_configs()

    # 2) Define move sets
    joint_rest = [0.0, -0.785, 0.785, 0.0, 1.57, 0.0]
    joint_look_sx = [-0.175, -0.419, 1.378, 0.349, 1.535, -0.977]
    joint_look_dx = [0.733, -0.297, 1.378, -0.576, 1.692, 1.291]

    named_home = "home"

    pick_target = Pose(
        position=Point(x=0.2, y=-0.1, z=0.15),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    )

    approach_target = Pose(
        position=Point(x=0.2, y=-0.1, z=pick_target.position.z + 0.02),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    )

    # Define move sequences:

    # First, a single move: in the create_tree_from_sequence function, it will be planned alone and will take less time to process
    # The next sequence of moves will be then be planned while the first is executing
    rest_position = [
        create_move("joint", joint_values=joint_rest, config=movement_configs["max_move"]),
    ]

    # This sequence scans the surroundings to build the octomap
    scan_surroundings = [
        #create_move("joint", joint_values=joint_rest, config=movement_configs["mid_move"]),
        create_move("joint", joint_values=joint_look_sx, config=movement_configs["max_move"]),
        create_move("joint", joint_values=joint_look_dx, config=movement_configs["max_move"]),
    ]

    # This sequence simulates a series of moves typical when picking up an object, minus the commands to action the gripper that are under construction
    pick_sequence = [
        create_move("pose", target=approach_target, config=movement_configs["mid_move"]),
        create_move("cartesian", target=pick_target, config=movement_configs["slow_move"]),
        create_move("cartesian", target=approach_target, config=movement_configs["max_move"]),
    ]

    home_position = [
        create_move("named", named_target=named_home, config=movement_configs["max_move"]),
    ]

    whole_sequence = [
        create_move("joint", joint_values=joint_rest, config=movement_configs["max_move"]),
        create_move("joint", joint_values=joint_look_sx, config=movement_configs["max_move"]),
        create_move("joint", joint_values=joint_look_dx, config=movement_configs["max_move"]),
        create_move("pose", target=approach_target, config=movement_configs["mid_move"]),
        create_move("cartesian", target=pick_target, config=movement_configs["slow_move"]),
        create_move("cartesian", target=approach_target, config=movement_configs["max_move"]),
        create_move("named", named_target=named_home, config=movement_configs["max_move"]),
    ]

    # 3) Build the tree with our parallel plan/exec logic
    list_of_sequences = [rest_position, scan_surroundings, pick_sequence, home_position]
    # this chained branch executes faster: it leverages a more parallel computation of the following moves since the parallelization in python is not perfect
    chained_branch = create_tree_from_sequences(node, list_of_sequences, root_name="LogicSequence")

    # # Here I define sequence per sequence to test other ways of building the sequence. This results slower than create_tree_from_sequences
    # rest_branch= create_tree_from_sequence(node, rest_position, root_name="Rest_Sequence")
    # scan_branch= create_tree_from_sequence(node, scan_surroundings, root_name="Scan_Sequence")
    # pick_branch= create_tree_from_sequence(node, pick_sequence, root_name="Pick_Sequence")
    # home_branch= create_tree_from_sequence(node, home_position, root_name="Home_Sequence")

    # # This also results slower than create_tree_from_sequences:
    # whole_seq_branch= create_tree_from_sequence_v2(node, whole_sequence, root_name="Whole_Sequence")

    main_seq = py_trees.composites.Sequence("Main_Sequence", True)
    main_seq.add_child(chained_branch.root)

    # main_seq.add_child(rest_branch.root)
    # main_seq.add_child(scan_branch.root)
    # main_seq.add_child(pick_branch.root)
    # main_seq.add_child(home_branch.root)
    # main_seq.add_child(whole_seq_branch.root)

    # This decorator will repeat the sequence indefinitely, for test purposes
    repeated_root = py_trees.decorators.Repeat(
    child=main_seq,
    num_success=-1,   # means repeat indefinitely
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

    # 5) Tick until done
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
