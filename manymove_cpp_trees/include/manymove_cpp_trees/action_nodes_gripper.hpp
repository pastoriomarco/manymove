#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_GRIPPER_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_GRIPPER_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "control_msgs/action/gripper_command.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace manymove_cpp_trees
{
  // =======================================================
  // GripperCommandAction
  // =======================================================

  class GripperCommandAction : public BT::StatefulActionNode
  {
  public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    GripperCommandAction(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<double>("position", 0.0, "Desired gripper position"),
          BT::InputPort<double>("max_effort", 0.0, "Maximum effort"),
          BT::InputPort<std::string>("action_server", "Action server name"),
          BT::OutputPort<double>("current_position", "Current gripper position")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    void goalResponseCallback(std::shared_ptr<GoalHandleGripperCommand> goal_handle);
    void resultCallback(const GoalHandleGripperCommand::WrappedResult &result);
    void feedbackCallback(std::shared_ptr<GoalHandleGripperCommand>,
                          const std::shared_ptr<const GripperCommand::Feedback> feedback);

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<GripperCommand>::SharedPtr action_client_;

    bool goal_sent_;
    bool result_received_;

    // Store the final result from the action server
    GripperCommand::Result action_result_;
  };

  // =======================================================
  // GripperTrajAction
  // =======================================================
  class GripperTrajAction : public BT::StatefulActionNode
  {
  public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    GripperTrajAction(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>(
              "action_server", "FollowJointTrajectory server name"),
          BT::InputPort<std::vector<std::string>>("joint_names", "List of gripper joint names"),
          BT::InputPort<std::vector<double>>("positions", "Target joint positions for each joint_name"),
          BT::InputPort<double>("time_from_start", 1.0, "Trajectory duration in seconds")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    void goalResponseCallback(std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    void resultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

    bool goal_sent_;
    bool result_received_;
    bool success_;
  };

  // =======================================================
  // PublishJointStateAction
  // =======================================================

  class PublishJointStateAction : public BT::SyncActionNode
  {
  public:
    PublishJointStateAction(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("topic", "/isaac_joint_commands_gripper", "Topic to publish"),
          BT::InputPort<std::vector<std::string>>("joint_names", "Joint names"),
          BT::InputPort<std::vector<double>>("joint_positions", "Positions (optional)"),
          BT::InputPort<std::vector<double>>("joint_velocities", "Velocities (optional)"),
          BT::InputPort<std::vector<double>>("joint_efforts", "Efforts (optional)"),
          BT::InputPort<bool>("stamp_now", true, "Stamp header with node->now()")};
    }

    BT::NodeStatus tick() override;

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    std::string current_topic_;
  };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_ACTION_NODES_GRIPPER_HPP
