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

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <manymove_msgs/action/plan_manipulator.hpp>
#include <manymove_msgs/msg/movement_config.hpp>
#include <memory>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <utility>
#include <vector>

#include "manymove_planner/compat/moveit_includes_compat.hpp"

/**
 * @class PlannerInterface
 * @brief Abstract interface for motion planners used in the manymove_planner package.
 *
 * This interface defines the essential methods that a planner implementation must provide:
 * - planning a trajectory for a given goal,
 * - executing a given trajectory,
 * - applying time parameterization (smoothing) while enforcing a maximum Cartesian speed,
 * - and sending a controlled stop command.
 */
class PlannerInterface
{
public:
  /**
 * @brief Virtual destructor for PlannerInterface.
 */
  virtual ~PlannerInterface() = default;

  /**
 * @brief Plan a trajectory to achieve a specified goal.
 * @param goal The target goal for the manipulator.
 * @return A pair containing a success flag (true if planning succeeded) and the planned robot
 * trajectory.
 */
  virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(
    const manymove_msgs::action::PlanManipulator::Goal & goal) = 0;

  /**
 * @brief Apply time parameterization to a trajectory.
 * @param input_traj The raw robot trajectory message (without time stamps) produced by the
 * planner.
 * @param config The movement configuration that specifies velocity and acceleration scaling
 * factors,
 * and the maximum allowed Cartesian speed.
 * @return A pair where the first element is true if time parameterization succeeded and the
 * second element is the
 * resulting trajectory with computed time stamps.
 *
 * @details Most of the industrial and collaborative robots have a maximum cartesian speed over
 * which the robot will perform
 * and emergency stop. Moreover, safety regulations in collaborative applications require the
 * enforcement of maximum cartesian
 * speed limits. While this package is not meant to provide functionalities compliant with safety
 * regulations, most robots
 * will come with such functionalities from factory, and they can't (or shouldn't) be overruled or
 * removed.
 * This function not only applies the time parametrization required for the trajectory to be
 * executed with a smooth motion,
 * but also reduces the velocity scaling if the calculated cartesian speed at any segment of the
 * trajectory exceeds the
 * cartesian limit set on the @p config parameter. Currently this function only limits the
 * velocity scaling factor, not the
 * acceleration scaling factor: this allows for faster movements as the acceleration is not
 * reduced together with the
 * velocity, but try to keep velocities and accelerations coherent with the cartesian speed you
 * want to obtain. Having really
 * slow moves with high accelerations may cause jerky and instable moves, so when you set the @p
 * config param always try to
 * keep the velocity and acceleration scaling factors coherent with the maximum cartesian speed
 * you set.
 */
  virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParameterization(
    const moveit_msgs::msg::RobotTrajectory & input_traj,
    const manymove_msgs::msg::MovementConfig & config) = 0;

  /**
 * @brief Send a controlled stop command to the robot.
 * @param decel_time_s The duration (in seconds) over which the robot’s velocities should be
 * ramped down to zero.
 * @param running_traj Current traj to stop
 * @param elapsed_s Elapsed time from the start of the current traj
 * @return True if the stop command was sent and executed successfully, false otherwise.
 *
 * @details If the running_traj is not set, this function sends a single-point trajectory to the
 * robot’s trajectory controller that holds the current
 * joint positions (with zero velocities) and gives the controller a deceleration window. The
 * effect is a “spring-back”
 * stop where the robot decelerates smoothly.
 * If running_traj is valid the end point will be the point of the traj where the robot will be at
 * decel_time_s from now.
 * Increasing the deceleration_time leads to a smoother stop, but also increases
 * the movement required to decelerate.
 */
  virtual bool sendControlledStop(
    const manymove_msgs::msg::MovementConfig & move_cfg,
    const moveit_msgs::msg::RobotTrajectory & running_traj = moveit_msgs::msg::RobotTrajectory(),
    double elapsed_s = 0.0) = 0;

  /**
 * @brief Retrieve the action client for FollowJointTrajectory.
 * @return A shared pointer to the action client used to send joint trajectory execution goals.
 */
  virtual rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
  getFollowJointTrajClient() const = 0;

  /**
 * @brief Check whether a given set of joint positions is valid (i.e. collision free).
 * @param joint_positions A vector of joint positions.
 * @return True if the joint state is valid (no collisions), false otherwise.
 */
  virtual bool isJointStateValid(const std::vector<double> & joint_positions) const = 0;

  /**
 * @brief Checks if the start of the trajectory (the first waypoint)
 *        is within a specified tolerance of the given current joint state.
 * @param traj The planned trajectory.
 * @param current_joint_state A vector of doubles representing the current joint positions.
 * @return true if each joint position in the first waypoint is within tolerance, false otherwise.
 */
  virtual bool isTrajectoryStartValid(
    const moveit_msgs::msg::RobotTrajectory & traj,
    const manymove_msgs::msg::MoveManipulatorGoal & move_request,
    const std::vector<double> & current_joint_state) const = 0;

  virtual bool isTrajectoryEndValid(
    const moveit_msgs::msg::RobotTrajectory & traj,
    const manymove_msgs::msg::MoveManipulatorGoal & move_request) const = 0;

  virtual bool isTrajectoryValid(
    const trajectory_msgs::msg::JointTrajectory & joint_traj_msg,
    const moveit_msgs::msg::Constraints & path_constraints,
    const double time_from_start = 0) const = 0;

  virtual bool isTrajectoryValid(
    const robot_trajectory::RobotTrajectory & trajectory,
    const moveit_msgs::msg::Constraints & path_constraints,
    const double time_from_start = 0) const = 0;

protected:
  /**
 * @brief Protected constructor to prevent direct instantiation of the interface.
 */
  PlannerInterface() = default;
};
