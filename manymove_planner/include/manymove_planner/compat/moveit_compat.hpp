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


// Thin MoveIt compatibility layer for API/header drift between ROS 2 Humble and Jazzy.
// - Includes the correct MoveIt headers (.hpp preferred, fallback to .h)
// - Provides helpers to access MoveGroupInterface::Plan trajectory member
// - Provides a wrapper for MoveGroupInterface::computeCartesianPath signature changes

#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "manymove_planner/compat/moveit_includes_compat.hpp"

namespace manymove_planner_compat
{

// Accessor for MoveGroupInterface::Plan trajectory field, supporting both
// Jazzy (plan.trajectory) and Humble (plan.trajectory_).
template<typename PlanT>
inline auto mgiPlanTrajectory(PlanT & plan) -> decltype((plan.trajectory))
{
  return plan.trajectory;
}

template<typename PlanT>
inline auto mgiPlanTrajectory(PlanT & plan) -> decltype((plan.trajectory_))
{
  return plan.trajectory_;
}

template<typename PlanT>
inline auto mgiPlanTrajectory(const PlanT & plan) -> decltype((plan.trajectory))
{
  return plan.trajectory;
}

template<typename PlanT>
inline auto mgiPlanTrajectory(const PlanT & plan) -> decltype((plan.trajectory_))
{
  return plan.trajectory_;
}

// Detection for old (Humble) MGI computeCartesianPath signature with jump_threshold
template<typename MGI>
struct has_mgi_cartesian_with_jump
{
private:
  template<typename T>
  static auto test(int) -> decltype(
    std::declval<T &>().computeCartesianPath(
      std::declval<const std::vector<geometry_msgs::msg::Pose> &>(),
      std::declval<double>(),
      std::declval<double>(),
      std::declval<moveit_msgs::msg::RobotTrajectory &>()),
    std::true_type{});

  template<typename>
  static std::false_type test(...);

public:
  static constexpr bool value = decltype(test<MGI>(0))::value;
};

// Detection for new (Jazzy) MGI computeCartesianPath signature without jump_threshold
template<typename MGI>
struct has_mgi_cartesian_no_jump
{
private:
  template<typename T>
  static auto test(int) -> decltype(
    std::declval<T &>().computeCartesianPath(
      std::declval<const std::vector<geometry_msgs::msg::Pose> &>(),
      std::declval<double>(),
      std::declval<moveit_msgs::msg::RobotTrajectory &>()),
    std::true_type{});

  template<typename>
  static std::false_type test(...);

public:
  static constexpr bool value = decltype(test<MGI>(0))::value;
};

// Wrapper that calls the appropriate MGI API depending on which signature exists.
template<typename MGI, typename PlanT>
inline double computeCartesianPathCompat(
  MGI & mgi,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  double eef_step,
  double jump_threshold,
  PlanT & plan)
{
  (void)jump_threshold; // unused in newer API

  if constexpr (has_mgi_cartesian_with_jump<MGI>::value) {
    return mgi.computeCartesianPath(waypoints, eef_step, jump_threshold, mgiPlanTrajectory(plan));
  } else if constexpr (has_mgi_cartesian_no_jump<MGI>::value) {
    return mgi.computeCartesianPath(waypoints, eef_step, mgiPlanTrajectory(plan));
  } else {
    static_assert(
      has_mgi_cartesian_with_jump<MGI>::value || has_mgi_cartesian_no_jump<MGI>::value,
      "Unsupported MoveGroupInterface::computeCartesianPath signature");
    return 0.0;
  }
}

namespace detail
{

template<typename MoveItCppT>
auto getPlanningSceneMonitorRwImpl(MoveItCppT & micpp, int)
-> decltype(micpp.getPlanningSceneMonitorNonConst())
{
  return micpp.getPlanningSceneMonitorNonConst();
}

template<typename MoveItCppT>
auto getPlanningSceneMonitorRwImpl(MoveItCppT & micpp, long)
-> decltype(micpp.getPlanningSceneMonitor())
{
  return micpp.getPlanningSceneMonitor();
}

template<typename PlanningComponentT>
auto executePlanningComponentImpl(PlanningComponentT & pc, bool blocking, int)
-> decltype(pc.execute(blocking), bool ())
{
  return pc.execute(blocking);
}

template<typename PlanningComponentT>
auto executePlanningComponentImpl(PlanningComponentT & pc, bool /*blocking*/, long)
-> decltype(pc.execute(), bool ())
{
  return pc.execute();
}

template<typename MoveItCppT>
auto detect_execute_with_controllers(int)
-> decltype(
  std::declval<MoveItCppT &>().execute(
    std::declval<robot_trajectory::RobotTrajectoryPtr>(),
    std::declval<const std::vector<std::string> &>()),
  std::true_type {})
{
  return {};
}

template<typename MoveItCppT>
std::false_type detect_execute_with_controllers(...);

template<typename MoveItCppT>
struct has_execute_with_controllers
  : decltype(detect_execute_with_controllers<MoveItCppT>(0))
{
};

template<typename MoveItCppT>
auto detect_execute_with_blocking(int)
-> decltype(
  std::declval<MoveItCppT &>().execute(
    std::declval<robot_trajectory::RobotTrajectoryPtr>(),
    bool {},
    std::declval<const std::vector<std::string> &>()),
  std::true_type{})
{
  return {};
}

template<typename MoveItCppT>
std::false_type detect_execute_with_blocking(...);

template<typename MoveItCppT>
struct has_execute_with_blocking
  : decltype(detect_execute_with_blocking<MoveItCppT>(0))
{
};

template<typename MoveItCppT>
auto detect_execute_with_group(int)
-> decltype(
  std::declval<MoveItCppT &>().execute(
    std::declval<const std::string &>(),
    std::declval<robot_trajectory::RobotTrajectoryPtr>()),
  std::true_type {})
{
  return {};
}

template<typename MoveItCppT>
std::false_type detect_execute_with_group(...);

template<typename MoveItCppT>
struct has_execute_with_group
  : decltype(detect_execute_with_group<MoveItCppT>(0))
{
};

template<typename MoveItCppT>
auto getPlanningPipelineNamesImpl(MoveItCppT & micpp, const std::string & planning_group, int)
-> decltype(micpp.getPlanningPipelineNames(planning_group))
{
  return micpp.getPlanningPipelineNames(planning_group);
}

template<typename MoveItCppT>
auto getPlanningPipelineNamesImpl(MoveItCppT & micpp, const std::string & /*planning_group*/, long)
-> std::vector<std::string>
{
  std::vector<std::string> names;
  const auto & pipelines = micpp.getPlanningPipelines();
  names.reserve(pipelines.size());
  for (const auto & kv : pipelines) {
    names.push_back(kv.first);
  }
  return names;
}

} // namespace detail

// Unified accessor for a non-const PlanningSceneMonitor pointer across MoveIt versions.
template<typename MoveItCppT>
auto getPlanningSceneMonitorRw(MoveItCppT & micpp)
-> decltype(detail::getPlanningSceneMonitorRwImpl(micpp, 0))
{
  return detail::getPlanningSceneMonitorRwImpl(micpp, 0);
}

template<typename MoveItCppT>
auto getPlanningSceneMonitorRw(const MoveItCppT & micpp)
-> decltype(detail::getPlanningSceneMonitorRwImpl(const_cast<MoveItCppT &>(micpp), 0))
{
  return detail::getPlanningSceneMonitorRwImpl(const_cast<MoveItCppT &>(micpp), 0);
}

template<typename MoveItCppT>
auto getPlanningSceneMonitorRw(const std::shared_ptr<MoveItCppT> & micpp)
-> decltype(getPlanningSceneMonitorRw(*micpp))
{
  return getPlanningSceneMonitorRw(*micpp);
}

// Execute PlanningComponent plans with graceful handling of API drift.
template<typename PlanningComponentT>
bool executePlanningComponent(PlanningComponentT & pc, bool blocking = true)
{
  return detail::executePlanningComponentImpl(pc, blocking, 0);
}

// Execute trajectories via MoveItCpp regardless of signature differences.
template<typename MoveItCppT>
auto executeTrajectory(
  MoveItCppT & micpp,
  const std::string & planning_group,
  const robot_trajectory::RobotTrajectoryPtr & trajectory,
  const std::vector<std::string> & controllers = {},
  bool blocking = true)
-> decltype(auto)
{
  if constexpr (detail::has_execute_with_controllers<MoveItCppT>::value) {
    (void)planning_group;
    (void)blocking;
    return micpp.execute(trajectory, controllers);
  } else if constexpr (detail::has_execute_with_blocking<MoveItCppT>::value) {
    (void)planning_group;
    return micpp.execute(trajectory, blocking, controllers);
  } else {
    static_assert(
      detail::has_execute_with_group<MoveItCppT>::value,
      "Unsupported MoveItCpp::execute signature for the provided MoveIt version");
    (void)controllers;
    (void)blocking;
    return micpp.execute(planning_group, trajectory);
  }
}

template<typename MoveItCppT>
auto getPlanningPipelineNames(MoveItCppT & micpp, const std::string & planning_group)
-> decltype(detail::getPlanningPipelineNamesImpl(micpp, planning_group, 0))
{
  return detail::getPlanningPipelineNamesImpl(micpp, planning_group, 0);
}

} // namespace manymove_planner_compat
