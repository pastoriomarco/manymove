#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_GRIPPER_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_GRIPPER_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "control_msgs/action/gripper_command.hpp"

namespace manymove_cpp_trees
{

class GripperCommandAction : public BT::StatefulActionNode
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

  GripperCommandAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("position", 0.0, "Desired gripper position"),
             BT::InputPort<double>("max_effort", 0.0, "Maximum effort"),
             BT::OutputPort<double>("current_position", "Current gripper position"),
             BT::OutputPort<bool>("success", "Command execution success") };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void goalResponseCallback(std::shared_ptr<GoalHandleGripperCommand> goal_handle);
  void resultCallback(const GoalHandleGripperCommand::WrappedResult& result);
  void feedbackCallback(std::shared_ptr<GoalHandleGripperCommand> /*goal_handle*/,
                        const std::shared_ptr<const GripperCommand::Feedback> feedback);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<GripperCommand>::SharedPtr action_client_;
  bool goal_sent_;
  bool result_received_;
  GripperCommand::Result action_result_;
};

}  // namespace manymove_cpp_trees

#endif  // MANYMOVE_CPP_TREES_ACTION_NODES_GRIPPER_HPP
