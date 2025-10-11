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

#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_PLANNER_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_PLANNER_HPP

#include "manymove_cpp_trees/move.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <manymove_msgs/action/move_manipulator.hpp>

#include <manymove_msgs/action/add_collision_object.hpp>
#include <manymove_msgs/action/remove_collision_object.hpp>
#include <manymove_msgs/action/attach_detach_object.hpp>
#include <manymove_msgs/action/check_object_exists.hpp>
#include <manymove_msgs/action/get_object_pose.hpp>

#include "manymove_msgs/action/set_output.hpp"
#include "manymove_msgs/action/get_input.hpp"
#include "manymove_msgs/action/check_robot_state.hpp"
#include "manymove_msgs/action/reset_robot_state.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>

namespace manymove_cpp_trees
{

  class MoveManipulatorAction : public BT::StatefulActionNode
  {
public:
    using MoveManipulator = manymove_msgs::action::MoveManipulator;
    using GoalHandleMoveManipulator = rclcpp_action::ClientGoalHandle<MoveManipulator>;

    MoveManipulatorAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("move_id"),
        BT::InputPort<std::string>("robot_prefix"),
        BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory"),
        BT::InputPort<std::string>("pose_key", "Optional key to retrieve the dynamic target pose"),
        BT::InputPort<bool>("collision_detected",
                            "If a collision is detected, the execution fails"),
        BT::InputPort<bool>("invalidate_traj_on_exec",
                            "Flag to indicate if the trajectory should be invalidated on exec even if successful"),
        BT::InputPort<bool>("stop_execution", "Flag to indicate that the execution is stopped"),
        BT::InputPort<int>("max_tries", "Number of times to try the execution"),
      };
    }

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void goalResponseCallback(std::shared_ptr<GoalHandleMoveManipulator> goal_handle);
    void feedbackCallback(std::shared_ptr<GoalHandleMoveManipulator>,
                          const std::shared_ptr<const MoveManipulator::Feedback> feedback);
    void resultCallback(const GoalHandleMoveManipulator::WrappedResult& result);

    int max_tries_;
    int current_try_;

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<MoveManipulator>::SharedPtr action_client_;

    bool goal_sent_;
    bool result_received_;

    std::string move_id_;
    std::string robot_prefix_;
    MoveManipulator::Result action_result_;
  };

/**
 * @class ResetTrajectories
 * @brief A synchronous BT node that resets trajectories and their validity in the blackboard.
 *
 * It takes a comma-separated list of move_ids and for each, sets 'trajectory_{id}' to empty
 * and 'validity_{id}' to false in the blackboard.
 */
  class ResetTrajectories : public BT::SyncActionNode
  {
public:
    /**
     * @brief Constructor for the ResetTrajectories node.
     * @param name The name of the BT node.
     * @param config The BT NodeConfiguration (ports, blackboard, etc.).
     */
    ResetTrajectories(const std::string& name, const BT::NodeConfiguration& config);

    /**
     * @brief Define the required/optional ports for this node.
     */
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("move_ids", "Comma-separated list of move IDs to reset")};
    }

    /**
     * @brief Tick function that performs the reset actions.
     */
    BT::NodeStatus tick() override;

private:
    // ROS2 node
    rclcpp::Node::SharedPtr node_;
  };

} // namespace manymove_cpp_trees

#endif
