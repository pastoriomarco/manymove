#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import py_trees
import py_trees_ros

import time
from geometry_msgs.msg import Pose, Point, Quaternion

from py_trees.blackboard import Blackboard
from moveit_msgs.msg import RobotTrajectory

from manymove_py_trees.move_definitions import (
    define_movement_configs,
    create_move,
)
from manymove_py_trees.tree_helper import create_tree_from_sequences

def main():
    rclpy.init()
    node = rclpy.create_node("bt_client_node")
    node.get_logger().info("BT Client Node started")

    bb = Blackboard()
    # Set defaults for keys used by MoveManipulatorBehavior:
    bb.set("reset", False)
    bb.set("stop_execution", False)
    bb.set("collision_detected", False)
    bb.set("invalidate_traj_on_exec", False)
    bb.set("existing_trajectory", RobotTrajectory())

    # 1) Define configs
    movement_configs = define_movement_configs()

    # 2) Define move sets
    joint_rest = [0.0, -0.785, 0.0, -2.355, 0.0, 3.14, 0.785]
    # (If you want more scanning moves, define them here)
    named_home = "ready"

    pick_target = Pose(
        position=Point(x=0.3, y=0.3, z=0.25),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    )
    approach_target = Pose(
        position=Point(x=pick_target.position.x, y=pick_target.position.y, z=pick_target.position.z + 0.02),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    )

    # Define your sequences of moves
    rest_position = [
        create_move("joint", joint_values=joint_rest, config=movement_configs["max_move"]),
    ]
    pick_sequence = [
        create_move("pose", target=approach_target, config=movement_configs["mid_move"]),
        create_move("cartesian", target=pick_target, config=movement_configs["slow_move"]),
        create_move("cartesian", target=approach_target, config=movement_configs["max_move"]),
    ]
    home_position = [
        create_move("named", named_target=named_home, config=movement_configs["max_move"]),
    ]

    # We'll chain these sequences in order
    list_of_sequences = [rest_position, pick_sequence, home_position]

    # 3) Build the Behavior Tree from these sequences
    chained_branch = create_tree_from_sequences(
        node,
        list_of_sequences,
        root_name="LogicSequence"
    )

    main_seq = py_trees.composites.Sequence("Main_Sequence", memory=True)
    main_seq.add_child(chained_branch.root)

    # Decorate with a "Repeat forever" for keeping it alive
    repeated_root = py_trees.decorators.Repeat(
        child=main_seq,
        num_success=-1,  # infinite repeat
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
