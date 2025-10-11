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

#include "manymove_cpp_trees/action_nodes_gripper.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <chrono>

using namespace std::chrono_literals;

namespace manymove_cpp_trees
{

// -------------------------------------------------
// GripperCommandAction
// -------------------------------------------------
GripperCommandAction::GripperCommandAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config), goal_sent_(false), result_received_(false)
{
  // Obtain the ROS node from the blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("GripperCommandAction: no blackboard provided.");
  }
  if (!config.blackboard->get("node", node_)) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperCommandAction"),
      "Shared node not found in blackboard, cannot initialize GripperCommandAction.");
    throw BT::RuntimeError("GripperCommandAction: 'node' not found in blackboard.");
  }

  // Retrieve the action server name from the input port
  std::string action_server_name;
  if (!getInput<std::string>("action_server", action_server_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing input [action_server]");
    throw BT::RuntimeError("GripperCommandAction: Missing input [action_server]");
  }

  action_client_ = rclcpp_action::create_client<GripperCommand>(node_, action_server_name);
}

BT::NodeStatus GripperCommandAction::onStart()
{
  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "GripperCommandAction: action server not available!");
    return BT::NodeStatus::FAILURE;
  }

  double position, max_effort;
  if (!getInput("position", position)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing input [position]");
    return BT::NodeStatus::FAILURE;
  }
  getInput("max_effort", max_effort);

  // Build the goal
  auto goal_msg = GripperCommand::Goal();
  goal_msg.command.position = position;
  goal_msg.command.max_effort = max_effort;

  goal_sent_ = true;
  result_received_ = false;

  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&GripperCommandAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&GripperCommandAction::resultCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&GripperCommandAction::feedbackCallback, this,
      std::placeholders::_1, std::placeholders::_2);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperCommandAction::onRunning()
{
  // If the goal wasn't even sent, fail
  if (!goal_sent_) {
    return BT::NodeStatus::FAILURE;
  }

  // If we're still waiting on the result, keep running
  if (!result_received_) {
    return BT::NodeStatus::RUNNING;
  }

  // Evaluate success from standard fields
  bool success = (action_result_.reached_goal || action_result_.stalled);

  // store the final position
  double pos = action_result_.position;
  setOutput("current_position", pos);

  // Return SUCCESS if truly reached the goal, else FAILURE
  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void GripperCommandAction::onHalted()
{
  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
  }
}

void GripperCommandAction::goalResponseCallback(
  std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "GripperCommandAction: goal was rejected!");
    result_received_ = true;
  }
  else {
    RCLCPP_INFO(node_->get_logger(),
      "GripperCommandAction: goal accepted, waiting for result...");
  }
}

void GripperCommandAction::resultCallback(
  const GoalHandleGripperCommand::WrappedResult & wrapped_result)
{
  action_result_ = *wrapped_result.result;
  result_received_ = true;
  RCLCPP_INFO(node_->get_logger(),
    "GripperCommandAction: result received. reached_goal=%s, stalled=%s",
    action_result_.reached_goal ? "true" : "false",
    action_result_.stalled ? "true" : "false");
}

void GripperCommandAction::feedbackCallback(
  std::shared_ptr<GoalHandleGripperCommand>,
  const std::shared_ptr<const GripperCommand::Feedback> feedback)
{
  // store the current feedback position
  setOutput("current_position", feedback->position);
  // e.g. we could log or track feedback->effort, feedback->stalled, etc.
}

// -------------------------------------------------
// GripperTrajAction
// -------------------------------------------------

GripperTrajAction::GripperTrajAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config),
  goal_sent_(false),
  result_received_(false),
  success_(false)
{
  // Grab the node handle from blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("GripperTrajAction: no blackboard provided.");
  }
  if (!config.blackboard->get("node", node_)) {
    RCLCPP_ERROR(rclcpp::get_logger("GripperTrajAction"),
      "Shared node not found in blackboard.");
    throw BT::RuntimeError("GripperTrajAction: 'node' not found in blackboard.");
  }

  // Read action_server from the input port
  std::string action_server_name;
  if (!getInput<std::string>("action_server", action_server_name)) {
    throw BT::RuntimeError("GripperTrajAction: missing [action_server] input port.");
  }

  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(node_, action_server_name);
}

BT::NodeStatus GripperTrajAction::onStart()
{
  // Wait a few seconds for the action server
  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "GripperTrajAction: server not available!");
    return BT::NodeStatus::FAILURE;
  }

  // Required inputs: joint_names, positions
  std::vector<std::string> joint_names;
  if (!getInput("joint_names", joint_names) || joint_names.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Missing or empty [joint_names]");
    return BT::NodeStatus::FAILURE;
  }
  std::vector<double> positions;
  if (!getInput("positions", positions) || positions.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Missing or empty [positions]");
    return BT::NodeStatus::FAILURE;
  }
  double time_from_start = 1.0;
  getInput<double>("time_from_start", time_from_start);

  // Build single-point trajectory
  control_msgs::action::FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
  goal.trajectory.points.push_back(point);

  // Reset flags, send the goal
  goal_sent_ = true;
  result_received_ = false;
  success_ = false;

  auto send_opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_opts.goal_response_callback =
    std::bind(&GripperTrajAction::goalResponseCallback, this, std::placeholders::_1);
  send_opts.result_callback =
    std::bind(&GripperTrajAction::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal, send_opts);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperTrajAction::onRunning()
{
  if (!goal_sent_) {
    return BT::NodeStatus::FAILURE;
  }

  if (!result_received_) {
    return BT::NodeStatus::RUNNING;
  }

  // If result was received, return SUCCESS if success_ is true, else FAILURE
  return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void GripperTrajAction::onHalted()
{
  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
  }
  goal_sent_ = false;
  result_received_ = false;
}

void GripperTrajAction::goalResponseCallback(
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "GripperTrajAction: goal rejected.");
    result_received_ = true;
    success_ = false;
  }
  else {
    RCLCPP_INFO(node_->get_logger(), "GripperTrajAction: goal accepted.");
  }
}

void GripperTrajAction::resultCallback(
  const GoalHandleFollowJointTrajectory::WrappedResult & result)
{
  result_received_ = true;
  success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
  RCLCPP_INFO(node_->get_logger(),
    "GripperTrajAction: action finished. success=%s",
    success_ ? "TRUE" : "FALSE");
}

// -------------------------------------------------
// PublishJointStateAction
// -------------------------------------------------

PublishJointStateAction::PublishJointStateAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("PublishJointStateAction: 'node' not found in blackboard.");
  }
}

BT::NodeStatus PublishJointStateAction::tick()
{
  std::string topic;
  if (!getInput("topic", topic) || topic.empty()) {
    throw BT::RuntimeError("PublishJointStateAction: missing 'topic'");
  }

  std::vector<std::string> names;
  if (!getInput("joint_names", names) || names.empty()) {
    throw BT::RuntimeError(
      "PublishJointStateAction: 'joint_names' must contain at least one joint");
  }

  std::vector<double> pos, vel, eff;
  (void)getInput("joint_positions", pos);
  (void)getInput("joint_velocities", vel);
  (void)getInput("joint_efforts", eff);

  // recreate publisher if topic changed
  if (!pub_ || topic != current_topic_) {
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic, 10);
    current_topic_ = topic;
  }

  const auto N = names.size();
  auto broadcast_or_check = [&](std::vector<double> & v, const char * label)
                            {
                              if (v.empty()) {
                                return;
                              }
                              if (v.size() == 1 && N > 1) {
                                v.assign(N, v[0]);                   // broadcast single value
                              }
                              if (v.size() != N) {
                                throw BT::RuntimeError(std::string("PublishJointStateAction: '") +
                                  label +
                                  "' length must be 1 or equal to 'names' length");
                              }
                            };

  broadcast_or_check(pos, "positions");
  broadcast_or_check(vel, "velocities");
  broadcast_or_check(eff, "efforts");

  sensor_msgs::msg::JointState msg;
  bool stamp_now = true;
  (void)getInput("stamp_now", stamp_now);
  if (stamp_now) {
    msg.header.stamp = node_->now();
  }

  msg.name = names;
  msg.position = pos;
  msg.velocity = vel;
  msg.effort = eff;

  pub_->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
    "[%s] Published JointState to '%s' (names=%zu, pos=%zu, vel=%zu, eff=%zu)",
    name().c_str(), topic.c_str(),
    msg.name.size(), msg.position.size(), msg.velocity.size(), msg.effort.size());

  return BT::NodeStatus::SUCCESS;
}

} // namespace manymove_cpp_trees
