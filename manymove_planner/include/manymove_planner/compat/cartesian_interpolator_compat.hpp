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
