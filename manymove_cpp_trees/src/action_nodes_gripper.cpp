#include "manymove_cpp_trees/action_nodes_gripper.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <chrono>

using namespace std::chrono_literals;

namespace manymove_cpp_trees
{

GripperCommandAction::GripperCommandAction(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config), goal_sent_(false), result_received_(false)
{
  // Obtain the ROS node from the blackboard
  if (!config.blackboard)
  {
    throw BT::RuntimeError("GripperCommandAction: no blackboard provided.");
  }
  if (!config.blackboard->get("node", node_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("GripperCommandAction"),
                 "Shared node not found in blackboard, cannot initialize GripperCommandAction.");
    throw BT::RuntimeError("GripperCommandAction: 'node' not found in blackboard.");
  }

  // Retrieve the action server name from the input port
  std::string action_server_name;
  if (!getInput<std::string>("action_server", action_server_name))
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing input [action_server]");
    throw BT::RuntimeError("GripperCommandAction: Missing input [action_server]");
  }

  action_client_ = rclcpp_action::create_client<GripperCommand>(node_, action_server_name);
}

BT::NodeStatus GripperCommandAction::onStart()
{
  if (!action_client_->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available!");
    return BT::NodeStatus::FAILURE;
  }

  double position, max_effort;
  if (!getInput("position", position))
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing input [position]");
    return BT::NodeStatus::FAILURE;
  }
  getInput("max_effort", max_effort);  // Optional input

  auto goal_msg = GripperCommand::Goal();
  goal_msg.command.position = position;
  goal_msg.command.max_effort = max_effort;

  goal_sent_ = false;
  result_received_ = false;

  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&GripperCommandAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback = std::bind(&GripperCommandAction::resultCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&GripperCommandAction::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperCommandAction::onRunning()
{
  if (!goal_sent_)
    return BT::NodeStatus::FAILURE;
  if (!result_received_)
    return BT::NodeStatus::RUNNING;

  // If we got a result, interpret success from standard fields:
  bool success = action_result_.reached_goal && !action_result_.stalled;
  double pos = action_result_.position;

  setOutput("current_position", pos);
  setOutput("success", success);

  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void GripperCommandAction::onHalted()
{
  if (goal_sent_ && !result_received_)
  {
    action_client_->async_cancel_all_goals();
  }
}

void GripperCommandAction::goalResponseCallback(std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(node_->get_logger(), "Gripper goal was rejected!");
    result_received_ = true;
  }
}

void GripperCommandAction::resultCallback(const GoalHandleGripperCommand::WrappedResult& wrapped_result)
{
  action_result_ = *wrapped_result.result;
  result_received_ = true;
}

void GripperCommandAction::feedbackCallback(std::shared_ptr<GoalHandleGripperCommand>,
                                            const std::shared_ptr<const GripperCommand::Feedback> feedback)
{
  setOutput("current_position", feedback->position);
  /// TODO: evaluate if also read feedback->effort, feedback->stalled, feedback->reached_goal
}

}  // namespace manymove_cpp_trees
