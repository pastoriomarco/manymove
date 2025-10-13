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

"""Utilities for building behaviour trees from MoveManipulator requests."""

from typing import List

import py_trees
import py_trees_ros
from manymove_py_trees.move_definitions import Move
from manymove_py_trees.move_manipulator_behavior import MoveManipulatorBehavior


def create_tree_from_moves(
    node, moves: List[Move], root_name: str = 'MoveSequence'
) -> py_trees_ros.trees.BehaviourTree:
    """
    Build a behaviour tree that executes each move sequentially.

    The resulting tree adds a :class:`MoveManipulatorBehavior` node per move and
    ticks them in sequence while propagating success or failure back to the caller.

    Parameters
    ----------
    node
        rclpy node used to run the behaviour tree.
    moves : List[Move]
        Ordered list of Move objects to execute.
    root_name : str, optional
        Name to apply to the top-level sequence.

    Returns
    -------
    py_trees_ros.trees.BehaviourTree
        Behaviour tree that executes the supplied moves.

    """
    # If no moves, just return an empty Sequence
    if not moves:
        empty_root = py_trees.composites.Sequence(name=root_name, memory=True)
        return py_trees_ros.trees.BehaviourTree(empty_root)

    # Build a top-level Sequence
    root = py_trees.composites.Sequence(name=root_name, memory=True)

    # For each Move, add a child node that calls MoveManipulator
    for i, move in enumerate(moves):
        move_node = MoveManipulatorBehavior(name=f'Move_{i}', node=node, move=move)
        root.add_child(move_node)

    # Wrap in a ROS BehaviorTree
    bt_tree = py_trees_ros.trees.BehaviourTree(root)
    return bt_tree


def create_tree_from_sequences(
    node, list_of_move_sequences: List[List[Move]], root_name: str = 'RootOfSequences'
) -> py_trees_ros.trees.BehaviourTree:
    """
    Create a behaviour tree that runs each sub-sequence of moves in series.

    Parameters
    ----------
    node
        rclpy node used to run the behaviour tree.
    list_of_move_sequences : List[List[Move]]
        List of move sequences to stitch together.
    root_name : str, optional
        Name to apply to the root composite node.

    Returns
    -------
    py_trees_ros.trees.BehaviourTree
        Behaviour tree that runs each sub-sequence in order.

    """
    # Build a top-level Sequence
    root = py_trees.composites.Sequence(name=root_name, memory=True)

    # For each sub-sequence, create another Sequence
    for seq_idx, moves in enumerate(list_of_move_sequences):
        sub_seq_node = py_trees.composites.Sequence(name=f'SubSequence_{seq_idx}', memory=True)
        for i, move in enumerate(moves):
            move_node = MoveManipulatorBehavior(name=f'Move_{seq_idx}_{i}', node=node, move=move)
            sub_seq_node.add_child(move_node)
        # Add this sub-sequence to the root
        root.add_child(sub_seq_node)

    return py_trees_ros.trees.BehaviourTree(root)
