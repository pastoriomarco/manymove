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

#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "manymove_planner/compat/moveit_includes_compat.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace manymove_planner::trajectory_utils
{
inline constexpr double kDefaultRotationSoftThresholdRad = 2.356194490192345;  // 135 degrees

inline geometry_msgs::msg::Pose poseFromState(
  const moveit::core::RobotState & robot_state, const std::string & link_frame)
{
  moveit::core::RobotState state_copy(robot_state);
  state_copy.updateLinkTransforms();

  geometry_msgs::msg::Pose pose;
  const Eigen::Isometry3d & transform = state_copy.getGlobalLinkTransform(link_frame);
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  const Eigen::Quaterniond quat(transform.rotation());
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  return pose;
}

inline double cartesianDistance(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  const double dx = b.position.x - a.position.x;
  const double dy = b.position.y - a.position.y;
  const double dz = b.position.z - a.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline geometry_msgs::msg::Pose poseFromTrajectory(
  const moveit_msgs::msg::RobotTrajectory & traj,
  const moveit::core::RobotState & reference_state, const std::string & link_frame,
  bool use_last_point)
{
  if (traj.joint_trajectory.points.empty()) {
    throw std::runtime_error("Trajectory is empty, cannot extract pose.");
  }

  const auto & point = use_last_point ? traj.joint_trajectory.points.back() :
    traj.joint_trajectory.points.front();

  moveit::core::RobotState state(reference_state);
  state.updateLinkTransforms();

  if (point.positions.size() != traj.joint_trajectory.joint_names.size()) {
    throw std::runtime_error("Trajectory point does not match joint name count.");
  }
  state.setVariablePositions(traj.joint_trajectory.joint_names, point.positions);
  return poseFromState(state, link_frame);
}

inline double rotationPenalty(
  const moveit_msgs::msg::RobotTrajectory & traj, double soft_threshold, double weight)
{
  if (traj.joint_trajectory.points.size() < 2) {
    return 0.0;
  }

  std::vector<double> accumulated(traj.joint_trajectory.joint_names.size(), 0.0);
  for (size_t i = 1; i < traj.joint_trajectory.points.size(); ++i) {
    const auto & prev = traj.joint_trajectory.points[i - 1];
    const auto & curr = traj.joint_trajectory.points[i];
    for (size_t j = 0; j < accumulated.size(); ++j) {
      if (j < prev.positions.size() && j < curr.positions.size()) {
        accumulated[j] += std::abs(curr.positions[j] - prev.positions[j]);
      }
    }
  }

  double penalty = 0.0;
  for (double travel : accumulated) {
    const double over = std::max(0.0, travel - soft_threshold);
    penalty += weight * over * over;
  }
  return penalty;
}

inline double maxCartesianSpeed(
  const robot_trajectory::RobotTrajectory & trajectory, const std::string & tcp_frame)
{
  if (trajectory.getWayPointCount() < 2) {
    return 0.0;
  }

  double max_speed = 0.0;
  for (size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
    const Eigen::Isometry3d prev_pose =
      trajectory.getWayPoint(i - 1).getGlobalLinkTransform(tcp_frame);
    const Eigen::Isometry3d curr_pose =
      trajectory.getWayPoint(i).getGlobalLinkTransform(tcp_frame);

    const double dist = (curr_pose.translation() - prev_pose.translation()).norm();
    const double dt = trajectory.getWayPointDurationFromPrevious(i);
    if (dt > 0.0) {
      max_speed = std::max(max_speed, dist / dt);
    }
  }
  return max_speed;
}

inline bool jointTargetsEqual(
  const std::vector<double> & first, const std::vector<double> & second, double tolerance)
{
  if (first.size() != second.size()) {
    return false;
  }
  for (size_t i = 0; i < first.size(); ++i) {
    if (std::abs(first[i] - second[i]) > tolerance) {
      return false;
    }
  }
  return true;
}

inline void updateJointStateMaps(
  const sensor_msgs::msg::JointState & msg, std::map<std::string, double> & positions,
  std::map<std::string, double> & velocities)
{
  const auto & names = msg.name;
  const auto & position_values = msg.position;
  const auto & velocity_values = msg.velocity;

  for (size_t i = 0; i < names.size(); ++i) {
    const std::string & joint = names[i];
    const double pos = (i < position_values.size()) ? position_values[i] : 0.0;
    const double vel = (i < velocity_values.size()) ? velocity_values[i] : 0.0;
    positions[joint] = pos;
    velocities[joint] = vel;
  }
}
}  // namespace manymove_planner::trajectory_utils
