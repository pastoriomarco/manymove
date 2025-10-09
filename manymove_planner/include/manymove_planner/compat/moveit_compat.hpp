// Thin MoveIt compatibility layer for API/header drift between ROS 2 Humble and Jazzy.
// - Includes the correct MoveIt headers (.hpp preferred, fallback to .h)
// - Provides helpers to access MoveGroupInterface::Plan trajectory member
// - Provides a wrapper for MoveGroupInterface::computeCartesianPath signature changes

#pragma once

#include <type_traits>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "manymove_planner/compat/moveit_includes_compat.hpp"

namespace manymove_planner_compat
{

// Accessor for MoveGroupInterface::Plan trajectory field, supporting both
// Jazzy (plan.trajectory) and Humble (plan.trajectory_).
template <typename PlanT>
inline auto mgiPlanTrajectory(PlanT &plan) -> decltype((plan.trajectory))
{
  return plan.trajectory;
}

template <typename PlanT>
inline auto mgiPlanTrajectory(PlanT &plan) -> decltype((plan.trajectory_))
{
  return plan.trajectory_;
}

template <typename PlanT>
inline auto mgiPlanTrajectory(const PlanT &plan) -> decltype((plan.trajectory))
{
  return plan.trajectory;
}

template <typename PlanT>
inline auto mgiPlanTrajectory(const PlanT &plan) -> decltype((plan.trajectory_))
{
  return plan.trajectory_;
}

// Detection for old (Humble) MGI computeCartesianPath signature with jump_threshold
template <typename MGI>
struct has_mgi_cartesian_with_jump
{
private:
  template <typename T>
  static auto test(int) -> decltype(
      std::declval<T &>().computeCartesianPath(
          std::declval<const std::vector<geometry_msgs::msg::Pose> &>(),
          std::declval<double>(),
          std::declval<double>(),
          std::declval<moveit_msgs::msg::RobotTrajectory &>()),
      std::true_type{});

  template <typename>
  static std::false_type test(...);

public:
  static constexpr bool value = decltype(test<MGI>(0))::value;
};

// Detection for new (Jazzy) MGI computeCartesianPath signature without jump_threshold
template <typename MGI>
struct has_mgi_cartesian_no_jump
{
private:
  template <typename T>
  static auto test(int) -> decltype(
      std::declval<T &>().computeCartesianPath(
          std::declval<const std::vector<geometry_msgs::msg::Pose> &>(),
          std::declval<double>(),
          std::declval<moveit_msgs::msg::RobotTrajectory &>()),
      std::true_type{});

  template <typename>
  static std::false_type test(...);

public:
  static constexpr bool value = decltype(test<MGI>(0))::value;
};

// Wrapper that calls the appropriate MGI API depending on which signature exists.
template <typename MGI, typename PlanT>
inline double computeCartesianPathCompat(
    MGI &mgi,
    const std::vector<geometry_msgs::msg::Pose> &waypoints,
    double eef_step,
    double jump_threshold,
    PlanT &plan)
{
  (void)jump_threshold; // unused in newer API

  if constexpr (has_mgi_cartesian_with_jump<MGI>::value)
  {
    return mgi.computeCartesianPath(waypoints, eef_step, jump_threshold, mgiPlanTrajectory(plan));
  }
  else if constexpr (has_mgi_cartesian_no_jump<MGI>::value)
  {
    return mgi.computeCartesianPath(waypoints, eef_step, mgiPlanTrajectory(plan));
  }
  else
  {
    static_assert(has_mgi_cartesian_with_jump<MGI>::value || has_mgi_cartesian_no_jump<MGI>::value,
                  "Unsupported MoveGroupInterface::computeCartesianPath signature");
    return 0.0;
  }
}

} // namespace manymove_planner_compat

