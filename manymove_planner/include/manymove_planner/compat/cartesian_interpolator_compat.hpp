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


// Compatibility wrapper for moveit::core::CartesianInterpolator API changes
// between Humble (uses JumpThreshold) and Jazzy (introduces CartesianPrecision).

#pragma once

#include <type_traits>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include "manymove_planner/compat/moveit_includes_compat.hpp"
#include "manymove_msgs/msg/movement_config.hpp"

namespace manymove_planner_compat
{

// Prefer Jazzy .hpp presence as proxy for CartesianPrecision availability
#if __has_include(<moveit/robot_state/cartesian_interpolator.hpp>)
#  define MANYMOVE_HAS_MOVEIT_CARTESIAN_PRECISION 1
#else
#  define MANYMOVE_HAS_MOVEIT_CARTESIAN_PRECISION 0
#endif

// Wrapper that selects the correct overload at compile time.
template<typename RefStatePtr>
inline double computeCartesianPathCompat(
  moveit::core::RobotState * start_state,
  const moveit::core::JointModelGroup * jmg,
  std::vector<moveit::core::RobotStatePtr> & trajectory_states,
  const moveit::core::LinkModel * ee_link,
  const EigenSTL::vector_Isometry3d & eigen_waypoints,
  bool global_reference_frame,
  const moveit::core::MaxEEFStep & max_eef_step,
  const manymove_msgs::msg::MovementConfig & cfg,
  const moveit::core::GroupStateValidityCallbackFn & validity_callback,
  const kinematics::KinematicsQueryOptions & options,
  RefStatePtr ref_state)
{
#if MANYMOVE_HAS_MOVEIT_CARTESIAN_PRECISION
  moveit::core::CartesianPrecision cp{cfg.cartesian_precision_translational,
    cfg.cartesian_precision_rotational,
    cfg.cartesian_precision_max_resolution};

  return moveit::core::CartesianInterpolator::computeCartesianPath(
    start_state, jmg, trajectory_states, ee_link, eigen_waypoints, global_reference_frame,
    max_eef_step, cp, validity_callback, options, ref_state);
#else
  return moveit::core::CartesianInterpolator::computeCartesianPath(
    start_state, jmg, trajectory_states, ee_link, eigen_waypoints, global_reference_frame,
    max_eef_step, moveit::core::JumpThreshold(cfg.jump_threshold),
    validity_callback, options, ref_state);
#endif
}

} // namespace manymove_planner_compat
