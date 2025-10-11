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

#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_OBJECTS_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_OBJECTS_HPP

#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <manymove_msgs/action/plan_manipulator.hpp>

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
/**
 * @class AddCollisionObjectAction
 * @brief A Behavior Tree node that adds a collision object to the planning scene using the
 *AddCollisionObject action server.
 */
  class AddCollisionObjectAction : public BT::StatefulActionNode
  {
public:
    using AddCollisionObject = manymove_msgs::action::AddCollisionObject;
    using GoalHandleAddCollisionObject = rclcpp_action::ClientGoalHandle<AddCollisionObject>;

    /**
     * @brief Constructor for the AddCollisionObjectAction node.
     * @param name The name of this BT node.
     * @param config The BT NodeConfiguration (ports, blackboard, etc.).
     */
    AddCollisionObjectAction(const std::string& name, const BT::NodeConfiguration& config);

    /**
     * @brief Define the required ports for this node.
     * @return A list of input ports.
     */
    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
        BT::InputPort<std::string>("shape", "Shape type (e.g., box, mesh)"),
        BT::InputPort<std::vector<double> >("dimensions", "Dimensions for primitive shapes"),
        BT::InputPort<geometry_msgs::msg::Pose>("pose", "Pose of the object"),
        BT::InputPort<std::string>("mesh_file", "", "Mesh file path (for mesh objects)"),
        BT::InputPort<std::vector<double> >("scale_mesh", "Scale factor along X-axis (for mesh)")};
    }

protected:
    /**
     * @brief Called once when transitioning from IDLE to RUNNING.
     * @return The initial state of the node after starting.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Called every tick while in RUNNING state.
     * @return The current status of the node.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called if this node is halted by force.
     */
    void onHalted() override;

private:
    // Callbacks for action client
    void goalResponseCallback(std::shared_ptr<GoalHandleAddCollisionObject> goal_handle);
    void resultCallback(const GoalHandleAddCollisionObject::WrappedResult& result);

    // ROS2 members
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<AddCollisionObject>::SharedPtr action_client_;

    // Internal state
    bool goal_sent_;
    bool result_received_;

    AddCollisionObject::Result action_result_;
    std::string object_id_;     ///< Unique object identifier
  };

/**
 * @class RemoveCollisionObjectAction
 * @brief A Behavior Tree node that removes a collision object from the planning scene using the
 *RemoveCollisionObject action server.
 */
  class RemoveCollisionObjectAction : public BT::StatefulActionNode
  {
public:
    using RemoveCollisionObject = manymove_msgs::action::RemoveCollisionObject;
    using GoalHandleRemoveCollisionObject = rclcpp_action::ClientGoalHandle<RemoveCollisionObject>;

    /**
     * @brief Constructor for the RemoveCollisionObjectAction node.
     * @param name The name of this BT node.
     * @param config The BT NodeConfiguration (ports, blackboard, etc.).
     */
    RemoveCollisionObjectAction(const std::string& name, const BT::NodeConfiguration& config);

    /**
     * @brief Define the required ports for this node.
     * @return A list of input ports.
     */
    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("object_id", "Unique identifier for the object to remove")};
    }

protected:
    /**
     * @brief Called once when transitioning from IDLE to RUNNING.
     * @return The initial state of the node after starting.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Called every tick while in RUNNING state.
     * @return The current status of the node.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called if this node is halted by force.
     */
    void onHalted() override;

private:
    // Callbacks for action client
    void goalResponseCallback(std::shared_ptr<GoalHandleRemoveCollisionObject> goal_handle);
    void resultCallback(const GoalHandleRemoveCollisionObject::WrappedResult& result);

    // ROS2 members
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<RemoveCollisionObject>::SharedPtr action_client_;

    // Internal state
    bool goal_sent_;
    bool result_received_;

    RemoveCollisionObject::Result action_result_;
    std::string object_id_;     ///< Unique object identifier
  };

/**
 * @class AttachDetachObjectAction
 * @brief A Behavior Tree node that attaches or detaches a collision object to/from a robot link
 *using the AttachDetachObject action server.
 */
  class AttachDetachObjectAction : public BT::StatefulActionNode
  {
public:
    using AttachDetachObject = manymove_msgs::action::AttachDetachObject;
    using GoalHandleAttachDetachObject = rclcpp_action::ClientGoalHandle<AttachDetachObject>;

    /**
     * @brief Constructor for the AttachDetachObjectAction node.
     * @param name The name of this BT node.
     * @param config The BT NodeConfiguration (ports, blackboard, etc.).
     */
    AttachDetachObjectAction(const std::string& name, const BT::NodeConfiguration& config);

    /**
     * @brief Define the required ports for this node.
     * @return A list of input ports.
     */
    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
        BT::InputPort<std::string>("link_name", "Name of the link to attach/detach the object"),
        BT::InputPort<bool>("attach", true, "True to attach, False to detach"),
        BT::InputPort<std::vector<std::string> >("touch_links",
                                                 std::vector<std::string>{},
                                                 "List of robot links to exclude from collision checking")};
    }

protected:
    /**
     * @brief Called once when transitioning from IDLE to RUNNING.
     * @return The initial state of the node after starting.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Called every tick while in RUNNING state.
     * @return The current status of the node.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called if this node is halted by force.
     */
    void onHalted() override;

private:
    // Callbacks for action client
    void goalResponseCallback(std::shared_ptr<GoalHandleAttachDetachObject> goal_handle);
    void resultCallback(const GoalHandleAttachDetachObject::WrappedResult& result);

    // ROS2 members
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<AttachDetachObject>::SharedPtr action_client_;

    // Internal state
    bool goal_sent_;
    bool result_received_;

    AttachDetachObject::Result action_result_;
    std::string object_id_;     ///< Unique object identifier
    std::string link_name_;     ///< Link name to attach/detach
    bool attach_;               ///< True to attach, False to detach
  };

/**
 * @class CheckObjectExistsAction
 * @brief A Behavior Tree node that checks if a collision object exists and whether it's attached
 *using the CheckObjectExists action server.
 */
  class CheckObjectExistsAction : public BT::StatefulActionNode
  {
public:
    using CheckObjectExists = manymove_msgs::action::CheckObjectExists;
    using GoalHandleCheckObjectExists = rclcpp_action::ClientGoalHandle<CheckObjectExists>;

    /**
     * @brief Constructor for the CheckObjectExistsAction node.
     * @param name The name of this BT node.
     * @param config The BT NodeConfiguration (ports, blackboard, etc.).
     */
    CheckObjectExistsAction(const std::string& name, const BT::NodeConfiguration& config);

    /**
     * @brief Define the required ports for this node.
     * @return A list of input and output ports.
     */
    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
        BT::OutputPort<bool>("exists", "Indicates if the object exists"),
        BT::OutputPort<bool>("is_attached", "Indicates if the object is attached to a link"),
        BT::OutputPort<std::string>("link_name",
                                    "Name of the link the object is attached to, if any")};
    }

protected:
    /**
     * @brief Called once when transitioning from IDLE to RUNNING.
     * @return The initial state of the node after starting.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Called every tick while in RUNNING state.
     * @return The current status of the node.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called if this node is halted by force.
     */
    void onHalted() override;

private:
    // Callbacks for action client
    void goalResponseCallback(std::shared_ptr<GoalHandleCheckObjectExists> goal_handle);
    void resultCallback(const GoalHandleCheckObjectExists::WrappedResult& result);

    // ROS2 members
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<CheckObjectExists>::SharedPtr action_client_;

    // Internal state
    bool goal_sent_;
    bool result_received_;

    CheckObjectExists::Result action_result_;
    std::string object_id_;     ///< Unique object identifier
  };

/**
 * @class GetObjectPoseAction
 * @brief A Behavior Tree node that retrieves and modifies the pose of a collision object using the
 *GetObjectPose action server.
 */
  class GetObjectPoseAction : public BT::StatefulActionNode
  {
public:
    using GetObjectPose = manymove_msgs::action::GetObjectPose;
    using GoalHandleGetObjectPose = rclcpp_action::ClientGoalHandle<GetObjectPose>;

    /**
     * @brief Constructor for the GetObjectPoseAction node.
     * @param name The name of this BT node.
     * @param config The BT NodeConfiguration (ports, blackboard, etc.).
     */
    GetObjectPoseAction(const std::string& name, const BT::NodeConfiguration& config);

    /**
     * @brief Define the required ports for this node.
     * @return A list of input and output ports.
     */
    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("object_id", "Identifier of the object"),
        BT::InputPort<std::vector<double> >("pre_transform_xyz_rpy",
                                            "First offset and rotation {x, y, z, roll, pitch, yaw}"),
        BT::InputPort<std::vector<double> >("post_transform_xyz_rpy",
                                            "Second offset and orientation {x, y, z, roll, pitch, yaw}"),
        BT::InputPort<std::string>("pose_key", "Blackboard key to store the retrieved pose"),
        BT::InputPort<std::string>("link_name", ""),
        BT::OutputPort<geometry_msgs::msg::Pose>("pose", "Pose after transformations")};
    }

protected:
    /**
     * @brief Called once when transitioning from IDLE to RUNNING.
     * @return The initial state of the node after starting.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Called every tick while in RUNNING state.
     * @return The current status of the node.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called if this node is halted by force.
     */
    void onHalted() override;

private:
    // Callbacks for action client
    void goalResponseCallback(std::shared_ptr<GoalHandleGetObjectPose> goal_handle);
    void resultCallback(const GoalHandleGetObjectPose::WrappedResult& result);

    // ROS2 members
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<GetObjectPose>::SharedPtr action_client_;

    // Internal state
    bool goal_sent_;
    bool result_received_;

    GetObjectPose::Result action_result_;
    std::string object_id_;                          ///< Unique object identifier
    std::vector<double> pre_transform_xyz_rpy_;      ///< Transformation offset and rotation
    std::vector<double> post_transform_xyz_rpy_;     ///< Reference orientation
    std::string pose_key_;                           ///< Blackboard key to store the pose
  };

/**
 * @brief A custom BT node that checks (in a loop) whether an object exists or not,
 *        until the desired condition is met or the timeout expires.
 *
 *  - "object_id" (string) : Name/ID of the object to check.
 *  - "exists"    (bool)   : If true => succeed when the object is found.
 *                           If false => succeed when the object is NOT found.
 *  - "timeout"   (double) : Seconds before giving up (0 => infinite wait).
 *  - "poll_rate" (double) : How often to re-check (seconds).
 *
 *  - "exists" (bool, output)       : Whether the object was found on the final check.
 *  - "is_attached" (bool, output)  : Whether the object was attached.
 *  - "link_name" (string, output)  : If attached, which link.
 */
  class WaitForObjectAction : public BT::StatefulActionNode
  {
public:
    using CheckObjectExists = manymove_msgs::action::CheckObjectExists;
    using GoalHandleCheckObjectExists = rclcpp_action::ClientGoalHandle<CheckObjectExists>;

    WaitForObjectAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("object_id", "ID of the object to check"),
        BT::InputPort<bool>("exists", true, "Wait for object to exist (true) or not exist (false)"),
        BT::InputPort<double>("timeout", 10.0, "Time (seconds) before giving up (0 => infinite)"),
        BT::InputPort<double>("poll_rate", 0.25, "Check frequency (seconds)"),
        BT::InputPort<std::string>("prefix", "Prefix for HMI messages, optional"),
        BT::OutputPort<bool>("exists", "Final check: was the object found?"),
        BT::OutputPort<bool>("is_attached", "Is the object attached?"),
        BT::OutputPort<std::string>("link_name", "Link name if attached")};
    }

protected:
    // Called once when transitioning from IDLE to RUNNING
    BT::NodeStatus onStart() override;

    // Called every tick while in RUNNING
    BT::NodeStatus onRunning() override;

    // Called if this node is halted by force
    void onHalted() override;

private:
    // Helper: send the action goal
    void sendCheckRequest();

    // Action client callbacks
    void goalResponseCallback(std::shared_ptr<GoalHandleCheckObjectExists> goal_handle);
    void resultCallback(const GoalHandleCheckObjectExists::WrappedResult& result);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<CheckObjectExists>::SharedPtr action_client_;

    std::string object_id_;     ///< The object ID to check
    bool desired_exists_;       ///< If true => succeed when object found, false => succeed when
                                // object not found
    double timeout_;            ///< Seconds to wait before giving up (0 => infinite)
    double poll_rate_;          ///< How often to re-check (seconds)
    std::string prefix_;        ///< Prefix for HMI messages

    // Internal timestamps
    rclcpp::Time start_time_;
    rclcpp::Time next_check_time_;

    // Internal flags
    bool goal_sent_;
    bool result_received_;

    // Last result from the action server
    bool last_exists_;
    bool last_is_attached_;
    std::string last_link_name_;
  };

} // namespace manymove_cpp_trees

#endif
