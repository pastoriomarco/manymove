import py_trees
import py_trees_ros
from typing import List

from manymove_py_trees.move_definitions import define_single_move_goal, Move
from manymove_py_trees.planning_behavior import PlanningActionBehaviour
from manymove_py_trees.execution_behavior import ExecuteTrajectoryBehaviour

def create_tree_from_sequence_v2(
    node,
    moves: List[Move],
    root_name: str = "SingleSequenceTreeV2",
    max_plan_next_retries: int = 2,
    max_exec_retries: int = 2
) -> py_trees_ros.trees.BehaviourTree:
    """
    A variant of create_tree_from_sequence where each move i is structured as:

      (1) "Plan i" (which quickly succeeds if the blackboard says it's already valid)
      (2) "Parallel i" => Fallback( Execute i + re-plan i on fail ), in parallel with Plan i+1

    This way:
      - When we are executing move i, we plan i+1 in parallel.
      - When the tree steps to the next move i+1, if the plan is already valid, the "Plan i+1" node is a quick success (skipped).
      - If any move fails to execute, the fallback re-plans that same move i.

    Note on partial re-planning:
      If 'execute i' fails, we can re-run Plan i in that fallback. If that fallback also fails,
      the parallel node fails, which means we re-run the entire step for i.

    Args:
      node: ROS2 node
      moves: list of Move objects
      root_name: name of the root node
      max_plan_next_retries: how many times to attempt planning the "next move" if it fails
      max_exec_retries: how many times to attempt re-executing a move if it fails

    Returns:
      A py_trees_ros.trees.BehaviourTree that tries to skip re-planning if it's already valid,
      planning the next move while the current move executes.
    """

    blackboard = py_trees.blackboard.Blackboard()
    n = len(moves)

    # If no moves, return an empty sequence
    if n == 0:
        empty_root = py_trees.composites.Sequence(name=root_name, memory=True)
        return py_trees_ros.trees.BehaviourTree(empty_root)

    root = py_trees.composites.Sequence(name=root_name, memory=True)

    for i in range(n):
        # --- (A) "Plan i" node ---
        # This plan node might quickly succeed if 'valid_traj_move{i}' is already True
        # or if there's a trajectory from planning i in a prior parallel step.

        planned_key_i = f"planned_traj_move{i}"
        validity_key_i = f"valid_traj_move{i}"
        previous_key = f"planned_traj_move{i-1}" if i > 0 else None

        # Build a plan_i node
        plan_i = PlanningActionBehaviour(
            name=f"PlanMove_{i}",
            goal=define_single_move_goal(moves[i]).goal,
            blackboard=blackboard,
            blackboard_key=planned_key_i,
            validity_key=validity_key_i,
            previous_key=previous_key,
            node=node
        )

        # Insert plan_i as the next child in the Sequence
        root.add_child(plan_i)

        # --- (B) Build the Parallel C_i node ---
        # which executes move i (with fallback that re-plans i if failure),
        # and in parallel plans move i+1 (if i < n-1).

        parallel_name = f"C_{i}"
        parallel_node = py_trees.composites.Parallel(
            name=parallel_name,
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )

        # 1) The left side is a Fallback for executing move i:
        #    Fallback( direct exec i, re-plan i + exec i ) with up to max_exec_retries
        fallback_execute = py_trees.composites.Selector(
            name=f"FallbackExec_{i}",
            memory=False
        )

        # Attempt #1: direct exec
        exec_i = ExecuteTrajectoryBehaviour(
            name=f"ExecuteMove_{i}",
            blackboard=blackboard,
            blackboard_key=planned_key_i,
            validity_key=validity_key_i,
            node=node
        )

        # Attempt #2: re-plan i, then re-execute i
        replan_seq_i = py_trees.composites.Sequence(name=f"RePlanAndExec_{i}", memory=True)

        # re-plan i if we had a failure
        replan_i = PlanningActionBehaviour(
            name=f"RePlanMove_{i}",
            goal=define_single_move_goal(moves[i]).goal,
            blackboard=blackboard,
            blackboard_key=planned_key_i,
            validity_key=validity_key_i,
            previous_key=previous_key,
            node=node
        )
        replan_seq_i.add_child(replan_i)

        re_exec_i = ExecuteTrajectoryBehaviour(
            name=f"ExecuteMove_{i}_retry",
            blackboard=blackboard,
            blackboard_key=planned_key_i,
            validity_key=validity_key_i,
            node=node
        )
        replan_seq_i.add_child(re_exec_i)

        fallback_execute.add_children([exec_i, replan_seq_i])

        # Optionally wrap the entire fallback in a Retry for N times if you'd like
        if max_exec_retries > 1:
            fallback_execute = py_trees.decorators.Retry(
                name=f"RetryExec_{i}",
                child=fallback_execute,
                num_failures=max_exec_retries
            )

        parallel_node.add_child(fallback_execute)

        # 2) The right side is "Plan i+1" if i < n-1, else a dummy success leaf
        if i < n - 1:
            # Plan the next move i+1
            planned_key_next = f"planned_traj_move{i+1}"
            validity_key_next = f"valid_traj_move{i+1}"
            goal_move_next = define_single_move_goal(moves[i+1]).goal

            plan_next = PlanningActionBehaviour(
                name=f"PlanMove_{i+1}",
                goal=goal_move_next,
                blackboard=blackboard,
                blackboard_key=planned_key_next,
                validity_key=validity_key_next,
                previous_key=planned_key_i,
                node=node
            )

            # Optionally wrap in a Retry
            if max_plan_next_retries > 1:
                plan_next = py_trees.decorators.Retry(
                    name=f"RetryPlan_{i+1}",
                    child=plan_next,
                    num_failures=max_plan_next_retries
                )

            parallel_node.add_child(plan_next)
        else:
            # if i == n-1, there's no next move to plan, so put a dummy success child
            # that won't block parallel_node from succeeding
            idle_child = py_trees.behaviours.Success(name=f"NoNextMove_{i}")
            parallel_node.add_child(idle_child)

        # Insert the parallel node in the sequence
        root.add_child(parallel_node)

    # Finally, wrap in a ROS BehaviorTree
    return py_trees_ros.trees.BehaviourTree(root)

def create_tree_from_sequence(
    node,
    moves: List[Move],
    root_name: str = "SingleSequenceTree",
    max_exec_retries: int = 5,
    max_plan_next_retries: int = 3
) -> py_trees_ros.trees.BehaviourTree:
    """
    Build a simpler Behavior Tree for a *single* sequence of moves, all in one function
    without a separate helper.

    -----------------------------
    DESIGN OVERVIEW (Move-by-Move)
    -----------------------------
    1) We have N moves, indexed [0..N-1].
    2) The top-level is a Sequence with N children (S_0, S_1, ..., S_{N-1}).
    3) Each child S_i does:
         * PlanMove_i
         * Then either:
             - Parallel(ExecuteMove_i, PlanMove_{i+1}) if i < N-1
             - ExecuteMove_i alone if i == N-1 (last move)
       and we optionally wrap [Plan_i + Parallel(...)] in a Retry decorator
       so that if ExecuteMove_i fails, we can re-run planning & execution.

    ---------------------------------------
    PER-MOVE RETRY + NEXT-MOVE PLANNING RETRY
    ---------------------------------------
    - Up to 'max_exec_retries' times if ExecuteMove_i fails.
    - Up to 'max_plan_next_retries' times if PlanMove_{i+1} fails.
    - This ensures partial re-planning if the robot or planner experiences a temporary issue.

    ---------------------------------------
    EXAMPLE (N=3: moves 0,1,2)
    ---------------------------------------
    The tree is:
       RootSequence [S_0, S_1, S_2]
         where S_0 is:
           RetryExec_0 -> Sequence(name="S_0")
             [PlanMove_0,
              Parallel( ExecuteMove_0, RetryPlan_1( PlanMove_1 ) ) ]
         S_1 is:
           RetryExec_1 -> Sequence(name="S_1")
             [PlanMove_1,
              Parallel( ExecuteMove_1, RetryPlan_2( PlanMove_2 ) ) ]
         S_2 is:
           RetryExec_2 -> Sequence(name="S_2")
             [PlanMove_2,
              ExecuteMove_2 ]  # last move, no next plan

    Args:
      node:                the ROS2 node (rclpy.node.Node)
      moves:               a list of Move objects for the single sequence
      root_name:           name for the top-level node
      max_exec_retries:    how many times to retry execution failures
      max_plan_next_retries: how many times to retry planning the *next* move if it fails

    Returns:
      A py_trees_ros.trees.BehaviourTree implementing concurrency
      (plan move i+1 while executing move i) plus per-move failure handling.
    """
    # Edge case: no moves => return an empty Sequence
    if not moves:
        empty_root = py_trees.composites.Sequence(name=root_name, memory=True)
        return py_trees_ros.trees.BehaviourTree(empty_root)

    # Build a top-level sequence for the entire chain of moves
    root_sequence = py_trees.composites.Sequence(name=root_name, memory=True)

    n = len(moves)

    for i in range(n):
        # -------------------------------
        # 1) PlanMove_i
        # -------------------------------
        blackboard = py_trees.blackboard.Blackboard()
        planned_key_i = f"planned_traj_move{i}"
        validity_key_i = f"valid_traj_move{i}"
        previous_key = f"planned_traj_move{i-1}" if i > 0 else None

        goal_move_i = define_single_move_goal(moves[i]).goal
        plan_i = PlanningActionBehaviour(
            name=f"PlanMove_{i}",
            goal=goal_move_i,
            blackboard=blackboard,
            blackboard_key=planned_key_i,
            validity_key=validity_key_i,
            previous_key=previous_key,
            node=node
        )

        # -------------------------------
        # 2) Build the child composite
        #    Either:
        #    - Parallel( ExecuteMove_i, PlanMove_{i+1} ) if i < n-1
        #    - ExecuteMove_i if i == n-1
        # -------------------------------
        if i < n - 1:
            # Next move's plan
            planned_key_next = f"planned_traj_move{i+1}"
            validity_key_next = f"valid_traj_move{i+1}"

            goal_move_next = define_single_move_goal(moves[i+1]).goal
            plan_next = PlanningActionBehaviour(
                name=f"PlanMove_{i+1}",
                goal=goal_move_next,
                blackboard=blackboard,
                blackboard_key=planned_key_next,
                validity_key=validity_key_next,
                previous_key=planned_key_i,  # chain from i's final positions
                node=node
            )

            # If needed, wrap next move's planning in a Retry
            if max_plan_next_retries > 1:
                plan_next = py_trees.decorators.Retry(
                    name=f"RetryPlan_{i+1}",
                    child=plan_next,
                    num_failures=max_plan_next_retries
                )

            # Build execute node for move i
            exec_i = ExecuteTrajectoryBehaviour(
                name=f"ExecuteMove_{i}",
                blackboard=blackboard,
                blackboard_key=planned_key_i,
                validity_key=validity_key_i,
                node=node
            )

            # Concurrency: both must succeed
            parallel_child = py_trees.composites.Parallel(
                name=f"Parallel_Exec{i}_Plan{i+1}",
                policy=py_trees.common.ParallelPolicy.SuccessOnAll()
            )
            parallel_child.add_child(exec_i)
            parallel_child.add_child(plan_next)

            child_i = parallel_child

        else:
            # Last move => just ExecuteMove_i with no concurrency
            exec_i = ExecuteTrajectoryBehaviour(
                name=f"ExecuteMove_{i}",
                blackboard=blackboard,
                blackboard_key=planned_key_i,
                validity_key=validity_key_i,
                node=node
            )
            child_i = exec_i

        # -------------------------------
        # 3) Combine [Plan_i, child_i] into a sequence "S_i"
        # -------------------------------
        seq_i = py_trees.composites.Sequence(name=f"S_{i}", memory=True)
        seq_i.add_child(plan_i)
        seq_i.add_child(child_i)

        # -------------------------------
        # 4) Optionally wrap S_i in a Retry
        #    => if exec fails, re-run plan i + exec i
        # -------------------------------
        if max_exec_retries > 1:
            seq_i = py_trees.decorators.Retry(
                name=f"RetryExec_{i}",
                child=seq_i,
                num_failures=max_exec_retries
            )

        # Add sub-sequence S_i to the root Sequence
        root_sequence.add_child(seq_i)

    # Wrap in a ROS BehaviorTree
    bt_tree = py_trees_ros.trees.BehaviourTree(root_sequence)
    return bt_tree


# ---------------------------------------------------------------------------------------------
# ORIGINAL FUNCTION create_tree_from_sequences
# ---------------------------------------------------------------------------------------------

def create_tree_from_sequences(
    node,
    list_of_move_sequences: List[List[Move]],
    root_name: str = "RootSequence"
) -> py_trees_ros.trees.BehaviourTree:
    """
    Creates a Behavior Tree that orchestrates multiple *sequences*, each treated as a single unit of plan+execute.
    
    ------------------------------------
    WHAT THIS FUNCTION DOES (HIGH LEVEL)
    ------------------------------------
    1) Each *sequence* is first fully planned (planning all moves in that sequence).
    2) We then do a Parallel:
         - Execute the just-planned sequence,
         - Plan the next sequence in parallel.
    3) We repeat this pattern for all sequences, so that while one sequence executes,
       the next one is being planned.

    This replicates the older "Parallel" pattern:
        RootSequence
        ├─ PlanSequence_0
        ├─ Parallel( ExecSequence_0, PlanSequence_1 )
        ├─ Parallel( ExecSequence_1, PlanSequence_2 )
        ├─ ...
        └─ ExecSequence_(n-1)

    -------------------------------------------
    REASONS FOR PLANNING MULTIPLE MOVES AT ONCE
    -------------------------------------------
    - By treating each sequence as a single plan, we can handle an entire chunk of moves together.
    - If the sequence fails at execution, we can replan that entire block if needed.
    - It's simpler if the user lumps moves into coherent sequences (e.g., a pick or a place phase).

    Args:
      node:            the ROS2 node used by actions.
      list_of_move_sequences: a list of sequences, each a list of Move objects.
      root_name:       name for the top-level Behavior node.

    Returns:
      A py_trees_ros.trees.BehaviourTree that first plans the 0th sequence,
      then in parallel executes it while planning the 1st, etc., finishing
      by executing the last sequence alone.
    """
    blackboard = py_trees.blackboard.Blackboard()

    plan_branches = []
    exec_branches = []
    last_planned_keys_per_seq = []  # for chaining final positions between sequences

    # ---------------------------------------------
    # 1) Build plan and exec branches for each seq
    # ---------------------------------------------
    for seq_index, moves in enumerate(list_of_move_sequences):
        # Create unique blackboard keys for each move
        planned_keys = [f"planned_traj_seq{seq_index}_move{i}" for i in range(len(moves))]
        validity_keys = [f"valid_traj_seq{seq_index}_move{i}" for i in range(len(moves))]

        # Create a Sequence that plans the entire sequence: PlanSequence_seqIndex
        plan_branch = py_trees.composites.Sequence(
            name=f"PlanSequence_{seq_index}", memory=True
        )
        for i, move in enumerate(moves):
            single_goal = define_single_move_goal(move).goal

            # For the first move in this sequence, chain from the end of the previous sequence if needed
            if i == 0 and seq_index > 0:
                previous_key = last_planned_keys_per_seq[seq_index - 1]
            elif i > 0:
                previous_key = planned_keys[i - 1]
            else:
                previous_key = None

            plan_behavior = PlanningActionBehaviour(
                name=f"PlanMove_{seq_index}_{i}",
                goal=single_goal,
                blackboard=blackboard,
                blackboard_key=planned_keys[i],
                validity_key=validity_keys[i],
                previous_key=previous_key,
                node=node
            )
            plan_branch.add_child(plan_behavior)

        # Create a Sequence that executes all moves in that sequence: ExecSequence_seqIndex
        exec_branch = py_trees.composites.Sequence(
            name=f"ExecSequence_{seq_index}", memory=True
        )
        for i in range(len(moves)):
            exec_behavior = ExecuteTrajectoryBehaviour(
                name=f"ExecuteMove_{seq_index}_{i}",
                blackboard=blackboard,
                blackboard_key=planned_keys[i],
                validity_key=validity_keys[i],
                node=node
            )
            exec_branch.add_child(exec_behavior)

        plan_branches.append(plan_branch)
        exec_branches.append(exec_branch)

        # The final planned key of this sequence is the last in planned_keys
        if len(planned_keys) > 0:
            last_planned_keys_per_seq.append(planned_keys[-1])
        else:
            last_planned_keys_per_seq.append(None)

    # ---------------------------------------
    # 2) Construct the root logic
    # ---------------------------------------
    root = py_trees.composites.Sequence(name=root_name, memory=True)
    n = len(list_of_move_sequences)
    if n == 0:
        return py_trees_ros.trees.BehaviourTree(root)

    # a) First, plan the 0th sequence fully
    root.add_child(plan_branches[0])

    # b) For each subsequent sequence in [1..n-1], we do a Parallel:
    #      - Execute the previous sequence
    #      - Plan the current sequence
    for seq_index in range(1, n):
        parallel_node = py_trees.composites.Parallel(
            name=f"Parallel_Exec{seq_index-1}_Plan{seq_index}",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        # left child: ExecSequence_{seq_index-1}
        parallel_node.add_child(exec_branches[seq_index - 1])
        # right child: PlanSequence_{seq_index}
        parallel_node.add_child(plan_branches[seq_index])
        root.add_child(parallel_node)

    # c) Finally, execute the *last* sequence
    root.add_child(exec_branches[n - 1])

    # Wrap the entire structure in a ROS BehaviorTree
    return py_trees_ros.trees.BehaviourTree(root)
