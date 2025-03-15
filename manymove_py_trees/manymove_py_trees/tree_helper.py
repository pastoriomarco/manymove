#!/usr/bin/env python3

import py_trees
import py_trees_ros
from typing import List

# Assuming you have these in your package:
from manymove_py_trees.move_definitions import Move
from manymove_py_trees.move_manipulator_behavior import MoveManipulatorBehavior

def create_tree_from_moves(
    node,
    moves: List[Move],
    root_name: str = "MoveSequence"
) -> py_trees_ros.trees.BehaviourTree:
    """
    Build a simple BT that executes each Move in sequence using the 'move_manipulator' action.

    Each Move is handled by a single MoveManipulatorBehavior node, which:
      - Sends the MoveManipulator action goal
      - Waits for success/failure
      - Cancels on halt, etc.

    Args:
        node:       ROS2 node (rclpy.node.Node)
        moves:      list of Move objects
        root_name:  name for the top-level node

    Returns:
        A py_trees_ros.trees.BehaviourTree that executes each move in a row.
    """
    # If no moves, just return an empty Sequence
    if not moves:
        empty_root = py_trees.composites.Sequence(name=root_name, memory=True)
        return py_trees_ros.trees.BehaviourTree(empty_root)

    # Build a top-level Sequence
    root = py_trees.composites.Sequence(name=root_name, memory=True)

    # For each Move, add a child node that calls MoveManipulator
    for i, move in enumerate(moves):
        move_node = MoveManipulatorBehavior(
            name=f"Move_{i}",
            node=node,
            move=move
        )
        root.add_child(move_node)

    # Wrap in a ROS BehaviorTree
    bt_tree = py_trees_ros.trees.BehaviourTree(root)
    return bt_tree


def create_tree_from_sequences(
    node,
    list_of_move_sequences: List[List[Move]],
    root_name: str = "RootOfSequences"
) -> py_trees_ros.trees.BehaviourTree:
    """
    Create a BT that executes multiple sequences of moves in order.

    For example, if you have:
       seq1 = [Move(...), Move(...)]
       seq2 = [Move(...), Move(...)]
       seq3 = [Move(...)]
    Then we first do seq1 (in a sub-sequence), then seq2, then seq3.

    Each move is handled by a MoveManipulatorBehavior node.

    Args:
        node:                  ROS2 node
        list_of_move_sequences A list of sequences, each a list of Move objects
        root_name:             name for the top-level node

    Returns:
        A py_trees_ros.trees.BehaviourTree that runs each sub-sequence in series.
    """
    # Build a top-level Sequence
    root = py_trees.composites.Sequence(name=root_name, memory=True)

    # For each sub-sequence, create another Sequence
    for seq_idx, moves in enumerate(list_of_move_sequences):
        sub_seq_node = py_trees.composites.Sequence(name=f"SubSequence_{seq_idx}", memory=True)
        for i, move in enumerate(moves):
            move_node = MoveManipulatorBehavior(
                name=f"Move_{seq_idx}_{i}",
                node=node,
                move=move
            )
            sub_seq_node.add_child(move_node)
        # Add this sub-sequence to the root
        root.add_child(sub_seq_node)

    return py_trees_ros.trees.BehaviourTree(root)
