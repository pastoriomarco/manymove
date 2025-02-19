#include "manymove_cpp_trees/action_nodes_gripper.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <chrono>

using namespace std::chrono_literals;

namespace manymove_cpp_trees {

GripperCommandAction::GripperCommandAction(const std::string &name, const BT::NodeConfiguration &config)
  : BT::StatefulActionNode(name, config),
    goal_sent_(false),
    result_received_(false)
{
  node_ = rclcpp::Node::make_shared("gripper_command_action_node");
  action_client_ = rclcpp_action::create_client<GripperCmd>(node_, "/panda_hand_controller/gripper_cmd");
}

BT::NodeStatus GripperCommandAction::onStart() {
  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available!");
    return BT::NodeStatus::FAILURE;
  }

  double position, max_effort;
  if (!getInput("position", position)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing input [position]");
    return BT::NodeStatus::FAILURE;
  }
  getInput("max_effort", max_effort); // Optional input

  auto goal_msg = GripperCmd::Goal();
  goal_msg.position = position;
  goal_msg.max_effort = max_effort;

  goal_sent_ = false;
  result_received_ = false;

  auto send_goal_options = rclcpp_action::Client<GripperCmd>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&GripperCommandAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&GripperCommandAction::resultCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&GripperCommandAction::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperCommandAction::onRunning() {
  if (!goal_sent_) return BT::NodeStatus::FAILURE;
  if (!result_received_) return BT::NodeStatus::RUNNING;

  setOutput("current_position", action_result_.current_position);
  setOutput("success", action_result_.success);
  return action_result_.success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void GripperCommandAction::onHalted() {
  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
  }
}

void GripperCommandAction::goalResponseCallback(std::shared_ptr<GoalHandleGripperCmd> goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper goal was rejected!");
    result_received_ = true;
  }
}

void GripperCommandAction::resultCallback(const GoalHandleGripperCmd::WrappedResult &result) {
  action_result_ = *result.result;
  result_received_ = true;
}

void GripperCommandAction::feedbackCallback(std::shared_ptr<GoalHandleGripperCmd> /*goal_handle*/,
                                            const std::shared_ptr<const GripperCmd::Feedback> feedback) {
  setOutput("current_position", feedback->current_position);
}

}  // namespace manymove_cpp_trees
