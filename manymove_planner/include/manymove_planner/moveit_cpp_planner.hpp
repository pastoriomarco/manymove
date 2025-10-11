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

#include "planner_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <future>
#include <map>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
 # include <tf2/LinearMath/Quaternion.hpp>
#else
 # include <tf2/LinearMath/Quaternion.h>
#endif
#include <tf2_eigen/tf2_eigen.hpp>

#include "manymove_planner/compat/moveit_includes_compat.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
// Included via compat: robot_state/cartesian_interpolator, robot_trajectory,
// and trajectory processing headers.

#include "manymove_msgs/msg/move_manipulator_goal.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>

// Included via compat: robot_model, planning_scene, planning_component

/**
 * @class MoveItCppPlanner
 * @brief A planner class integrating with MoveItCpp for motion planning in ROS2.
 *
 * This class implements the PlannerInterface methods using MoveItCpp as the
 * underlying planning framework. It provides functionalities for trajectory
 * generation, time parameterization, and optionally execution through a
 * FollowJointTrajectory action server.It provides utilities to handle multiple
 * movement types (joint, pose, named, cartesian).
 */
class MoveItCppPlanner : public PlannerInterface
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

/**
 * @brief Constructor for MoveItCppPlanner.
 * @param node Shared pointer to the ROS2 node.
 * @param planning_group Name of the planning group (defined in MoveIt).
 * @param base_frame The base frame of the robot.
 * @param traj_controller Name of the trajectory controller to use for execution.
 */
  MoveItCppPlanner(const rclcpp::Node::SharedPtr & node,
    const std::string & planning_group,
    const std::string & base_frame,
    const std::string & traj_controller,
    const std::shared_ptr<moveit_cpp::MoveItCpp> & moveit_cpp_ptr = nullptr);

/**
 * @brief Default destructor.
 */
  ~MoveItCppPlanner() override = default;

/**
 * @brief Plan a trajectory for a single goal.
 * @param goal The PlanManipulator goal containing movement type and parameters.
 * @return A pair containing a success flag and the planned robot trajectory.
 */
  std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(
    const manymove_msgs::action::PlanManipulator::Goal & goal) override;

/**
 * @brief Apply time parameterization to a single trajectory using the given movement config.
 * @param trajectory Pointer to a RobotTrajectory to parameterize.
 * @param config The movement configuration specifying velocity/acceleration factors, etc.
 * @return True if the time parameterization succeeded, false otherwise, plus the resulting
 * RobotTrajectory.
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
  std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParameterization(
    const moveit_msgs::msg::RobotTrajectory & input_traj,
    const manymove_msgs::msg::MovementConfig & config);

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
  bool sendControlledStop(const manymove_msgs::msg::MovementConfig & move_cfg,
    const moveit_msgs::msg::RobotTrajectory & running_traj = moveit_msgs::msg::RobotTrajectory
      (),
    double elapsed_s = 0.0);

/**
 * @brief Retrieve the action client for FollowJointTrajectory.
 * @return A shared pointer to the action client used to send joint trajectory execution goals.
 */
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
  getFollowJointTrajClient() const;

/**
 * @brief Check whether a given set of joint positions is valid (i.e. collision free).
 * @param joint_positions A vector of joint positions.
 * @return True if the joint state is valid (no collisions), false otherwise.
 */
  bool isJointStateValid(const std::vector<double> & joint_positions) const;

/**
 * @brief Checks if the start of the trajectory (the first waypoint)
 *        is within a specified tolerance of the given current joint state.
 * @param traj The planned trajectory.
 * @param current_joint_state A vector of doubles representing the current joint positions.
 * @return true if each joint position in the first waypoint is within tolerance, false otherwise.
 */
  bool isTrajectoryStartValid(const moveit_msgs::msg::RobotTrajectory & traj,
    const manymove_msgs::msg::MoveManipulatorGoal & move_request,
    const std::vector<double> & current_joint_state) const;

  bool isTrajectoryEndValid(const moveit_msgs::msg::RobotTrajectory & traj,
    const manymove_msgs::msg::MoveManipulatorGoal & move_request) const;

  bool isTrajectoryValid(const trajectory_msgs::msg::JointTrajectory & joint_traj_msg,
    const moveit_msgs::msg::Constraints & path_constraints,
    const double time_from_start) const;

  bool isTrajectoryValid(const robot_trajectory::RobotTrajectory & trajectory,
    const moveit_msgs::msg::Constraints & path_constraints,
    const double time_from_start) const;

private:
/**
 * @brief Compute the total path length of a given trajectory.
 * @param trajectory The MoveIt robot trajectory message.
 * @return The computed path length in joint/Cartesian space.
 */
  double computePathLength(const moveit_msgs::msg::RobotTrajectory & trajectory,
    const manymove_msgs::msg::MovementConfig & config) const;

/**
 * @brief Calculate the pose relative to a frame from a robot state.
 * @param robot_state The robot state to get the joint positions from.
 * @param link_frame The reference frame to calculate the pose from the joint positions of the
 * robot state.
 * @return The computed distance between the two poses.
 */
  geometry_msgs::msg::Pose getPoseFromRobotState(const moveit::core::RobotState & robot_state,
    const std::string & link_frame) const;

/**
 * @brief Compute the euclidean distance between two poses.
 * @param start_pose The start pose to calculate the distance from.
 * @param target_pose The target pose to calculate the distance to.
 * @return The computed distance between the two poses.
 */
  double computeCartesianDistance(const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & target_pose) const;

/**
 * @brief Calculate the pose relative to a frame from the first or the last point of a trajectory.
 * @param traj_msg The trajectory from which to get the point to calculate from.
 * @param robot_state Contains the info about the robot to calculate the pose.
 * @param link_frame The reference frame to calculate the pose.
 * @param use_last_point If true it uses the last point of the trajectory, if false the first
 * point.
 * @return The computed distance between the two poses.
 */
  geometry_msgs::msg::Pose getPoseFromTrajectory(const moveit_msgs::msg::RobotTrajectory & traj_msg,
    const moveit::core::RobotState & robot_state,
    const std::string & link_frame,
    bool use_last_point = true) const;

/**
 * @brief Compute the maximum Cartesian speed found in a trajectory.
 * @param trajectory A pointer to the robot trajectory object.
 * @return The maximum Cartesian speed (m/s) found in the trajectory.
 */
  double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr & trajectory,
    const manymove_msgs::msg::MovementConfig & config) const;

/**
 * @brief Check if two joint targets (vectors of joint values) are equal within a specified
 * tolerance.
 * @param j1 First joint target.
 * @param j2 Second joint target.
 * @param tolerance The acceptable tolerance for each joint's difference.
 * @return True if all corresponding joints match within the tolerance, false otherwise.
 */
  bool areSameJointTargets(const std::vector<double> & j1, const std::vector<double> & j2,
    double tolerance) const;

/**
 * @brief Convert a RobotTrajectory object to a corresponding message.
 * @param trajectory The input RobotTrajectory reference.
 * @return A moveit_msgs::msg::RobotTrajectory representation of the same trajectory.
 */
  moveit_msgs::msg::RobotTrajectory convertToMsg(
    const robot_trajectory::RobotTrajectory & trajectory) const;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

/**
 * @brief Collision-check callback used by Cartesian path computations.
 * @param state Pointer to the RobotState being tested for collisions.
 * @param group Pointer to the JointModelGroup for which collision checks should be performed.
 * @return True if the state is free of collisions, false otherwise.
 *
 * @details This function is provided as a custom validity checker for
 * moveit::core::CartesianInterpolator::computeCartesianPath(), ensuring that
 * intermediate states do not collide with any known obstacles or the robot
 * itself. Without isStateValid() used as a callback function, computeCartesianPath()
 * wouldn't check for collisions just for mesh objects, while it did for primitive objects.
 * I couldn't figure if I missed some setting to avoid this problem, if I found some edge
 * cases that resulted in this abnormal behavior, or if it is in fact the intended behavior
 * of computeCartesianPath(). Regardless, this seems to solve the issue.
 * The function performs a collision check on the given @p state for the
 * specified joint model group. It uses a read only locked version of the current
 * Planning Scene to verify whether the @p state is in collision. If a collision is
 * detected, the function logs a warning and returns false. Otherwise, it returns true.
 */
  bool isStateValid(const moveit::core::RobotState * state,
    const moveit::core::JointModelGroup * group) const;

  rclcpp::Node::SharedPtr node_;   ///< Shared pointer to the ROS2 node.
  rclcpp::Logger logger_;          ///< Logger instance for logging messages.
  std::string planning_group_;     ///< Name of the planning group.
  std::string base_frame_;         ///< The base frame of the robot.
  std::string traj_controller_;    ///< Name of the trajectory controller.

  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;                  ///< Shared pointer to
// the MoveItCpp
// instance.
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;     ///< Shared pointer to
// the PlanningComponent
// instance.
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_;   ///< Planning parameters
// loaded at startup.

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    follow_joint_traj_client_;                                                                               ///<
// Action
// client
// for
// FollowJointTrajectory.
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;                            ///<
// /joint_states
// subscriber.

  mutable std::mutex js_mutex_;
  std::map<std::string, double> current_positions_;
  std::map<std::string, double> current_velocities_;
};
