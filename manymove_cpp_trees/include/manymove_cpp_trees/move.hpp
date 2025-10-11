// Copyright 2025 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MANYMOVE_CPP_TREES_MOVE_HPP
#define MANYMOVE_CPP_TREES_MOVE_HPP

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "manymove_msgs/msg/movement_config.hpp"
#include "manymove_msgs/msg/move_manipulator_goal.hpp"
#include <unordered_map>

namespace manymove_cpp_trees
{
// ----------------------------------------------------------------------------
// Move setup functions
// ----------------------------------------------------------------------------

/**
 * @brief Return some standard MovementConfig presets (max_move, mid_move, slow_move).
 */
  inline std::unordered_map<std::string, manymove_msgs::msg::MovementConfig>
  defineMovementConfigs()
  {
    using manymove_msgs::msg::MovementConfig;

    /**
     * REFERENCE MovementConfig STRUCTURE
     *
     * # manymove parameters
     * float64 max_cartesian_speed             # max cartesian speed for the move, referred to TCP
     * int32 plan_number_target                # number of valid plans to find before declaring
     *success
     * int32 plan_number_limit                 # max planning tries before fail
     * string smoothing_type                   # "time-optimal" / "ruckig" (## under construction
     *##)
     * string tcp_frame                        # End-effector (TCP) frame for this request
     * float64 linear_precision                # Linear precision for end-point trajectory position
     *check
     * float64 rotational_precision            # Rotational precision for end-point trajectory
     *position check
     * float64 deceleration_time               # Time to decelerate the robot to arrive at a full
     *stop
     * float64 min_stop_time                   # Minimum time to perform a controlled stop, if less
     *time remains the traj will just end
     *
     * # moveit planner parameters
     * float64 velocity_scaling_factor         # 0.0 to 1.0
     * float64 acceleration_scaling_factor     # 0.0 to 1.0
     * string planning_pipeline                # e.g. "ompl", "pilz",
     *"pilz_industrial_motion_planner", ...
     * string planner_id                       # e.g. "RRTConnect", "PTP", "LIN", ...
     * float64 planning_time                   # overall time budget in seconds
     * int32  planning_attempts                # e.g. 1, 5, etc.
     *
     * # CartesianInterpolator parameters
     * float64 step_size
     * float64 jump_threshold
     */

    /**
     * Note about plan_number_target:
     * Increasing the plan_number_target increase time to plan but produce trajs closer to optimal.
     * With 4 plans I noticed a quite good result, 8 to 12 plans reach almost always a very good
     *result but with planning time increases.
     * Some robots may need more, expecially if they have a wider range of movement on each joint.
     */

    /**
     * Note about tcp_frame: it's to be set on move basis, so we give it as an input to the Move
     *constructor
     */

    MovementConfig max_move_config;
    max_move_config.velocity_scaling_factor = 1.0;
    max_move_config.acceleration_scaling_factor = 0.75;
    max_move_config.max_cartesian_speed = 0.45;
    max_move_config.linear_precision = 0.001;
    max_move_config.deceleration_time = 0.5;
    max_move_config.min_stop_time = 0.5;
    max_move_config.rotational_precision = 0.05;
    max_move_config.step_size = 0.005;
    max_move_config.jump_threshold = 0.0;
    // Defaults for Jazzy CartesianPrecision (ignored on Humble)
    max_move_config.cartesian_precision_translational = 0.001;       // 1 mm
    max_move_config.cartesian_precision_rotational = 0.05;           // ~2.9 deg
    max_move_config.cartesian_precision_max_resolution = 0.005;      // 5 mm
    max_move_config.plan_number_target = 8;
    max_move_config.plan_number_limit = 16;
    max_move_config.smoothing_type = "time_optimal";
    max_move_config.planning_pipeline = "ompl";
    max_move_config.planner_id = "RRTConnect";
    max_move_config.planning_time = 5;
    max_move_config.planning_attempts = 5;

    MovementConfig mid_move_config = max_move_config;
    mid_move_config.velocity_scaling_factor /= 2.0;
    mid_move_config.acceleration_scaling_factor /= 2.0;
    mid_move_config.max_cartesian_speed = 0.25;

    MovementConfig slow_move_config = max_move_config;
    slow_move_config.step_size = 0.002;
    slow_move_config.velocity_scaling_factor /= 4.0;
    slow_move_config.acceleration_scaling_factor /= 4.0;
    slow_move_config.max_cartesian_speed = 0.05;
    slow_move_config.deceleration_time = 0.25;
    slow_move_config.min_stop_time = 0.2;

    // Cartesian path shouldn't need more than one plan to reach optimal traj, since it's a straight
    // line.
    MovementConfig cartesian_max_move_config = max_move_config;
    cartesian_max_move_config.plan_number_target = 1;

    MovementConfig cartesian_mid_move_config = mid_move_config;
    cartesian_mid_move_config.plan_number_target = 1;

    MovementConfig cartesian_slow_move_config = slow_move_config;
    cartesian_slow_move_config.plan_number_target = 1;

    // Moves to perform search/interrupt: using cartesian as base since seaching on a probabilistic
    // path would't make much sense...
    MovementConfig search_mid_move_config = cartesian_max_move_config;
    search_mid_move_config.velocity_scaling_factor /= 2.0;
    search_mid_move_config.acceleration_scaling_factor /= 2.0;
    search_mid_move_config.max_cartesian_speed = 0.15;
    search_mid_move_config.deceleration_time = 0.25;
    search_mid_move_config.min_stop_time = 0.05;     // need more stop precision on slow moves for
                                                     // input searches

    MovementConfig search_slow_move_config = cartesian_max_move_config;
    search_slow_move_config.step_size = 0.001;
    search_slow_move_config.velocity_scaling_factor /= 4.0;
    search_slow_move_config.acceleration_scaling_factor /= 4.0;
    search_slow_move_config.max_cartesian_speed = 0.025;
    search_slow_move_config.deceleration_time = 0.1;
    search_slow_move_config.min_stop_time = 0.025;     // need more stop precision on slow moves for
                                                       // input searches

    // isaac_ros_cumotion test move
    MovementConfig cumotion_max_move_config = max_move_config;
    cumotion_max_move_config.planning_pipeline = "isaac_ros_cumotion";
    cumotion_max_move_config.planner_id = "cuMotion";
    cumotion_max_move_config.planning_time = 5;
    cumotion_max_move_config.planning_attempts = 1;
    cumotion_max_move_config.plan_number_target = 1;

    // Pilz PTP move
    MovementConfig PTP_max_move_config = max_move_config;
    PTP_max_move_config.planning_pipeline = "pilz_industrial_motion_planner";
    PTP_max_move_config.planner_id = "PTP";
    PTP_max_move_config.planning_time = 5;
    PTP_max_move_config.planning_attempts = 1;
    PTP_max_move_config.plan_number_target = 1;
    PTP_max_move_config.velocity_scaling_factor = 1.0;     ///< This scales the max_rot_vel in
                                                           // pilz_cartesian_limits.yaml

    // Pilz LIN moves
    MovementConfig LIN_max_move_config = max_move_config;
    LIN_max_move_config.planning_pipeline = "pilz_industrial_motion_planner";
    LIN_max_move_config.planner_id = "LIN";
    LIN_max_move_config.planning_time = 5;
    LIN_max_move_config.planning_attempts = 1;
    LIN_max_move_config.plan_number_target = 1;
    LIN_max_move_config.velocity_scaling_factor = 0.5;         ///< This scales the max_trans_vel in
                                                               // pilz_cartesian_limits.yaml
    LIN_max_move_config.acceleration_scaling_factor = 0.5;     ///< This scales the max_trans_acc in
                                                               // pilz_cartesian_limits.yaml

    MovementConfig LIN_mid_move_config = LIN_max_move_config;
    LIN_mid_move_config.velocity_scaling_factor = 0.2;
    LIN_mid_move_config.acceleration_scaling_factor = 0.2;

    MovementConfig LIN_slow_move_config = LIN_max_move_config;
    LIN_slow_move_config.velocity_scaling_factor = 0.1;
    LIN_slow_move_config.acceleration_scaling_factor = 0.1;
    LIN_slow_move_config.min_stop_time = 0.05;     // need more stop precision on slow moves for
                                                   // input searches

    // CHOMP move
    MovementConfig CHOMP_max_move_config = max_move_config;
    CHOMP_max_move_config.planning_pipeline = "chomp";
    CHOMP_max_move_config.planner_id = "CHOMP";
    CHOMP_max_move_config.planning_time = 5;
    CHOMP_max_move_config.planning_attempts = 1;
    CHOMP_max_move_config.plan_number_target = 1;

    // // STOMP move (uncomment here and modify src/manymove/manymove_planner/config/moveit_cpp.yaml
    // to add STOMP in Jazzy)
    // MovementConfig CHOMP_max_move_config = max_move_config;
    // CHOMP_max_move_config.planning_pipeline = "stomp";
    // CHOMP_max_move_config.planner_id = "STOMP";
    // CHOMP_max_move_config.planning_time = 5;
    // CHOMP_max_move_config.planning_attempts = 1;
    // CHOMP_max_move_config.plan_number_target = 1;

    return {
      // Standard moves for joint and pose for OMPL planning library
      {"max_move", max_move_config},
      {"mid_move", mid_move_config},
      {"slow_move", slow_move_config},

      // Params for cartesian moves
      {"cartesian_max_move", cartesian_max_move_config},
      {"cartesian_mid_move", cartesian_mid_move_config},
      {"cartesian_slow_move", cartesian_slow_move_config},

      // Moves for search/interrupt movements
      {"search_mid_move", search_mid_move_config},
      {"search_slow_move", search_slow_move_config},

      // Params for pilz_industrial_planner planning library
      {"PTP_max_move", PTP_max_move_config},
      {"LIN_max_move", LIN_max_move_config},
      {"LIN_mid_move", LIN_mid_move_config},
      {"LIN_slow_move", LIN_slow_move_config},

      // Params for CHOMP planning library
      {"CHOMP_max_move", CHOMP_max_move_config},

      // // Params for STOMP planning library (uncomment here and modify
      // src/manymove/manymove_planner/config/moveit_cpp.yaml to add STOMP in Jazzy)
      // {"STOMP_max_move", STOMP_max_move_config},

      // Test for moves with cuMotion planning library
      {"cumotion_max_move", cumotion_max_move_config},
    };
  }

/**
 * @struct Move
 * @brief Represents a single move command with a specified type and configuration.
 *
 * This struct is used to store details of a single move, including its type
 * (e.g., "pose", "joint", "named", or "cartesian"), any relevant pose or joint data,
 * and the associated MovementConfig.
 */
  struct Move
  {
    std::string type;                              ///< The movement type
    std::string pose_key;                          ///< Blackboard key for dynamic pose
    std::vector<double> joint_values;              ///< Joint values for "joint" type.
    std::string named_target;                      ///< Named target for "named" type.
    manymove_msgs::msg::MovementConfig config;     ///< Movement configuration parameters.
    std::vector<double> start_joint_values;        ///< Starting joint values for planning.
    std::string robot_prefix;                      ///< Prefix to correctly reference previous
                                                   // moves.
    std::string tcp_frame;                         ///< TCP for cartesian speed calculations

    Move(
      const std::string& robot_prefix,
      const std::string& tcp_frame,
      const std::string& type,
      const manymove_msgs::msg::MovementConfig& config,
      const std::string& pose_key = "",
      const std::vector<double>& joint_values = {},
      const std::string& named_target = "",
      const std::vector<double>& start_joint_values = {})
      : type(type),
      pose_key(pose_key),
      joint_values(joint_values),
      named_target(named_target),
      config(config),
      start_joint_values(start_joint_values),
      robot_prefix(robot_prefix),
      tcp_frame(tcp_frame)
    {
    }

    manymove_msgs::msg::MoveManipulatorGoal to_move_manipulator_goal() const
    {
      manymove_msgs::msg::MoveManipulatorGoal goal;
      goal.movement_type = type;

      if (type == "pose" || type == "cartesian") {
        // Retrieve pose from blackboard using pose_key
        // This will be handled in the PlanningAction node
        // goal.pose_target = pose_target;
      }
      else if (type == "joint") {
        goal.joint_values = joint_values;
      }
      else if (type == "named") {
        goal.named_target = named_target;
      }

      goal.start_joint_values = start_joint_values;

      goal.config = config;
      goal.config.tcp_frame = tcp_frame;

      return goal;
    }
  };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_MOVE_HPP
