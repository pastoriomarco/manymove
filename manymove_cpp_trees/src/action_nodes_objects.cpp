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

#include "manymove_cpp_trees/action_nodes_objects.hpp"

#include <behaviortree_cpp_v3/blackboard.h>

#include <memory>
#include <stdexcept>

#include "manymove_cpp_trees/hmi_utils.hpp"

namespace manymove_cpp_trees
{
// ---------------------- AddCollisionObjectAction ----------------------

AddCollisionObjectAction::AddCollisionObjectAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), goal_sent_(false), result_received_(false)
{
  static constexpr const char * kName = "AddCollisionObjectAction";

  // Obtain the ROS node from the blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("%s: no blackboard provided.", kName);
  }
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("%s: 'node' not found in blackboard.", kName);
  }

  // Initialize the action client
  action_client_ = rclcpp_action::create_client<AddCollisionObject>(node_, "add_collision_object");
  RCLCPP_INFO(
    node_->get_logger(), "%s: Waiting for 'add_collision_object' server...", kName);
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    throw BT::RuntimeError(
            "%s: 'add_collision_object' server not available after waiting.", kName);
  }
  RCLCPP_INFO(
    node_->get_logger(), "%s: Connected to 'add_collision_object' server.", kName);
}

BT::NodeStatus AddCollisionObjectAction::onStart()
{
  static constexpr const char * kName = "AddCollisionObjectAction";

  RCLCPP_DEBUG(node_->get_logger(), "%s: onStart() called.", kName);

  goal_sent_ = false;
  result_received_ = false;
  action_result_ = AddCollisionObject::Result();

  // Retrieve input ports
  if (!getInput<std::string>("object_id", object_id_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: Missing required input 'object_id'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  std::string shape;
  if (!getInput<std::string>("shape", shape)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Missing required input 'shape'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  std::vector<double> dimensions;
  if (shape != "mesh") {
    if (!getInput<std::vector<double>>("dimensions", dimensions)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "%s: Missing required input 'dimensions' for shape '%s'.",
        kName, shape.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  geometry_msgs::msg::Pose pose;
  if (!getInput<geometry_msgs::msg::Pose>("pose", pose)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Missing required input 'pose'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  std::string mesh_file;
  if (shape == "mesh") {
    if (!getInput<std::string>("mesh_file", mesh_file)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "%s: Missing required input 'mesh_file' for mesh shape.", kName);
      return BT::NodeStatus::FAILURE;
    }
  }

  std::vector<double> scale;
  getInput<std::vector<double>>("scale_mesh", scale);

  // Create and send the goal
  auto goal_msg = AddCollisionObject::Goal();
  goal_msg.id = object_id_;
  goal_msg.shape = shape;
  goal_msg.pose = pose;

  if (shape == "mesh") {
    goal_msg.mesh_file = mesh_file;
    goal_msg.scale_mesh = scale;
  } else {
    goal_msg.dimensions = dimensions;
  }

  RCLCPP_INFO(
    node_->get_logger(), "%s: Sending goal for object '%s'.",
    kName, object_id_.c_str());

  auto send_goal_options = rclcpp_action::Client<AddCollisionObject>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&AddCollisionObjectAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&AddCollisionObjectAction::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AddCollisionObjectAction::onRunning()
{
  static constexpr const char * kName = "AddCollisionObjectAction";

  if (result_received_) {
    if (action_result_.success) {
      RCLCPP_INFO(
        node_->get_logger(), "%s: Successfully added object '%s'.",
        kName, object_id_.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "%s: Failed to add object '%s'. Message: %s",
        kName, object_id_.c_str(), action_result_.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void AddCollisionObjectAction::onHalted()
{
  static constexpr const char * kName = "AddCollisionObjectAction";

  RCLCPP_WARN(
    node_->get_logger(), "%s: onHalted() called. Cancelling goal if sent.", kName);

  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
    RCLCPP_INFO(node_->get_logger(), "%s: Goal canceled.", kName);
  }

  goal_sent_ = false;
  result_received_ = false;
}

void AddCollisionObjectAction::goalResponseCallback(
  std::shared_ptr<GoalHandleAddCollisionObject> goal_handle)
{
  static constexpr const char * kName = "AddCollisionObjectAction";

  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Goal was rejected by the server.", kName);
    // You can set action_result_ here if needed
    result_received_ = true;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s: Goal accepted by the server, waiting for result.", kName);
  }
}

void AddCollisionObjectAction::resultCallback(
  const GoalHandleAddCollisionObject::WrappedResult & wrapped_result)
{
  static constexpr const char * kName = "AddCollisionObjectAction";

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "%s: Goal succeeded.", kName);
      action_result_ = *(wrapped_result.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "%s: Goal was aborted.", kName);
      action_result_.success = false;
      action_result_.message = "Action aborted.";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "%s: Goal was canceled.", kName);
      action_result_.success = false;
      action_result_.message = "Action canceled.";
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "%s: Unknown result code.", kName);
      action_result_.success = false;
      action_result_.message = "Unknown result code.";
      break;
  }

  result_received_ = true;
}

// ---------------------- RemoveCollisionObjectAction ----------------------

RemoveCollisionObjectAction::RemoveCollisionObjectAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), goal_sent_(false), result_received_(false)
{
  static constexpr const char * kName = "RemoveCollisionObjectAction";

  // Obtain the ROS node from the blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("%s: no blackboard provided.", kName);
  }
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("%s: 'node' not found in blackboard.", kName);
  }

  // Initialize the action client
  action_client_ =
    rclcpp_action::create_client<RemoveCollisionObject>(node_, "remove_collision_object");
  RCLCPP_INFO(
    node_->get_logger(),
    "%s: Waiting for 'remove_collision_object' server...", kName);
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    throw BT::RuntimeError(
            "%s: 'remove_collision_object' server not available after waiting.", kName);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "%s: Connected to 'remove_collision_object' server.", kName);
}

BT::NodeStatus RemoveCollisionObjectAction::onStart()
{
  static constexpr const char * kName = "RemoveCollisionObjectAction";

  RCLCPP_DEBUG(node_->get_logger(), "%s: onStart() called.", kName);

  goal_sent_ = false;
  result_received_ = false;
  action_result_ = RemoveCollisionObject::Result();

  // Retrieve input port
  if (!getInput<std::string>("object_id", object_id_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: Missing required input 'object_id'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  // Create and send the goal
  auto goal_msg = RemoveCollisionObject::Goal();
  goal_msg.id = object_id_;

  RCLCPP_INFO(
    node_->get_logger(), "%s: Sending goal to remove object '%s'.", kName,
    object_id_.c_str());

  auto send_goal_options = rclcpp_action::Client<RemoveCollisionObject>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&RemoveCollisionObjectAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&RemoveCollisionObjectAction::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RemoveCollisionObjectAction::onRunning()
{
  static constexpr const char * kName = "RemoveCollisionObjectAction";

  if (result_received_) {
    if (action_result_.success) {
      RCLCPP_INFO(
        node_->get_logger(), "%s: Successfully removed object '%s'.",
        kName, object_id_.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "%s: Failed to remove object '%s'. Message: %s",
        kName, object_id_.c_str(), action_result_.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void RemoveCollisionObjectAction::onHalted()
{
  static constexpr const char * kName = "RemoveCollisionObjectAction";

  RCLCPP_WARN(
    node_->get_logger(),
    "%s: onHalted() called. Cancelling goal if sent.", kName);

  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
    RCLCPP_INFO(node_->get_logger(), "%s: Goal canceled.", kName);
  }

  goal_sent_ = false;
  result_received_ = false;
}

void RemoveCollisionObjectAction::goalResponseCallback(
  std::shared_ptr<GoalHandleRemoveCollisionObject> goal_handle)
{
  static constexpr const char * kName = "RemoveCollisionObjectAction";

  if (!goal_handle) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: Goal was rejected by the server.", kName);
    // You can set action_result_ here if needed
    result_received_ = true;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s: Goal accepted by the server, waiting for result.", kName);
  }
}

void RemoveCollisionObjectAction::resultCallback(
  const GoalHandleRemoveCollisionObject::WrappedResult & wrapped_result)
{
  static constexpr const char * kName = "RemoveCollisionObjectAction";

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "%s: Goal succeeded.", kName);
      action_result_ = *(wrapped_result.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "%s: Goal was aborted.", kName);
      action_result_.success = false;
      action_result_.message = "Action aborted.";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "%s: Goal was canceled.", kName);
      action_result_.success = false;
      action_result_.message = "Action canceled.";
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "%s: Unknown result code.", kName);
      action_result_.success = false;
      action_result_.message = "Unknown result code.";
      break;
  }

  result_received_ = true;
}

// ---------------------- AttachDetachObjectAction ----------------------

AttachDetachObjectAction::AttachDetachObjectAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  goal_sent_(false),
  result_received_(false),
  attach_(true)  // Default to attach
{
  static constexpr const char * kName = "AttachDetachObjectAction";

  // Obtain the ROS node from the blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("%s: no blackboard provided.", kName);
  }
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("%s: 'node' not found in blackboard.", kName);
  }

  // Initialize the action client
  action_client_ = rclcpp_action::create_client<AttachDetachObject>(node_, "attach_detach_object");
  RCLCPP_INFO(
    node_->get_logger(), "%s: Waiting for 'attach_detach_object' server...", kName);
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    throw BT::RuntimeError(
            "%s: 'attach_detach_object' server not available after waiting.", kName);
  }
  RCLCPP_INFO(
    node_->get_logger(), "%s: Connected to 'attach_detach_object' server.", kName);
}

BT::NodeStatus AttachDetachObjectAction::onStart()
{
  static constexpr const char * kName = "AttachDetachObjectAction";

  RCLCPP_DEBUG(node_->get_logger(), "%s: onStart() called.", kName);

  goal_sent_ = false;
  result_received_ = false;
  action_result_ = AttachDetachObject::Result();

  if (!getInput<std::string>("object_id", object_id_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: Missing required input 'object_id'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput<std::string>("link_name", link_name_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: Missing required input 'link_name'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput<bool>("attach", attach_)) {
    RCLCPP_WARN(
      node_->get_logger(), "%s: Missing input 'attach'. Defaulting to true.", kName);
    attach_ = true;
  }

  // Retrieve the optional "touch_links" input
  std::vector<std::string> touch_links;
  if (!getInput("touch_links", touch_links)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "%s: No 'touch_links' provided, defaulting to empty.", kName);
    touch_links = std::vector<std::string>{};
  }

  auto goal_msg = AttachDetachObject::Goal();
  goal_msg.object_id = object_id_;
  goal_msg.link_name = link_name_;
  goal_msg.attach = attach_;
  goal_msg.touch_links = touch_links;

  std::string action = attach_ ? "attaching" : "detaching";
  RCLCPP_INFO(
    node_->get_logger(),
    "%s: Sending goal for %s object '%s' to link '%s'. Touch links size: "
    "'%li'",
    kName, action.c_str(), object_id_.c_str(), link_name_.c_str(), touch_links.size());

  auto send_goal_options = rclcpp_action::Client<AttachDetachObject>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&AttachDetachObjectAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&AttachDetachObjectAction::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AttachDetachObjectAction::onRunning()
{
  static constexpr const char * kName = "AttachDetachObjectAction";

  if (result_received_) {
    if (action_result_.success) {
      std::string action = attach_ ? "attached" : "detached";
      RCLCPP_INFO(
        node_->get_logger(), "%s: Successfully %s object '%s' to link '%s'.",
        kName, action.c_str(), object_id_.c_str(), link_name_.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      std::string action = attach_ ? "attach" : "detach";
      RCLCPP_ERROR(
        node_->get_logger(),
        "%s: Failed to %s object '%s' to link '%s'. Message: %s",
        kName, action.c_str(), object_id_.c_str(), link_name_.c_str(),
        action_result_.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void AttachDetachObjectAction::onHalted()
{
  static constexpr const char * kName = "AttachDetachObjectAction";

  RCLCPP_WARN(
    node_->get_logger(), "%s: onHalted() called. Cancelling goal if sent.", kName);

  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
    RCLCPP_INFO(node_->get_logger(), "%s: Goal canceled.", kName);
  }

  goal_sent_ = false;
  result_received_ = false;
}

void AttachDetachObjectAction::goalResponseCallback(
  std::shared_ptr<GoalHandleAttachDetachObject> goal_handle)
{
  static constexpr const char * kName = "AttachDetachObjectAction";

  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Goal was rejected by the server.", kName);
    // You can set action_result_ here if needed
    result_received_ = true;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s: Goal accepted by the server, waiting for result.", kName);
  }
}

void AttachDetachObjectAction::resultCallback(
  const GoalHandleAttachDetachObject::WrappedResult & wrapped_result)
{
  static constexpr const char * kName = "AttachDetachObjectAction";

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "%s: Goal succeeded.", kName);
      action_result_ = *(wrapped_result.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "%s: Goal was aborted.", kName);
      action_result_ = *(wrapped_result.result);
      // action_result_.success = false;
      // action_result_.message = "Action aborted.";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "%s: Goal was canceled.", kName);
      action_result_.success = false;
      action_result_.message = "Action canceled.";
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "%s: Unknown result code.", kName);
      action_result_.success = false;
      action_result_.message = "Unknown result code.";
      break;
  }

  result_received_ = true;
}

// ---------------------- CheckObjectExistsAction ----------------------

CheckObjectExistsAction::CheckObjectExistsAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), goal_sent_(false), result_received_(false)
{
  static constexpr const char * kName = "CheckObjectExistsAction";

  // Obtain the ROS node from the blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("%s: no blackboard provided.", kName);
  }
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("%s: 'node' not found in blackboard.", kName);
  }

  // Initialize the action client
  action_client_ = rclcpp_action::create_client<CheckObjectExists>(node_, "check_object_exists");
  RCLCPP_INFO(
    node_->get_logger(), "%s: Waiting for 'check_object_exists' server...", kName);
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    throw BT::RuntimeError(
            "%s: 'check_object_exists' server not available after waiting.", kName);
  }
  RCLCPP_INFO(
    node_->get_logger(), "%s: Connected to 'check_object_exists' server.", kName);
}

BT::NodeStatus CheckObjectExistsAction::onStart()
{
  static constexpr const char * kName = "CheckObjectExistsAction";

  RCLCPP_DEBUG(node_->get_logger(), "%s: onStart() called.", kName);

  goal_sent_ = false;
  result_received_ = false;
  action_result_ = CheckObjectExists::Result();

  // Retrieve input port
  if (!getInput<std::string>("object_id", object_id_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: Missing required input 'object_id'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  // Create and send the goal
  auto goal_msg = CheckObjectExists::Goal();
  goal_msg.object_id = object_id_;

  RCLCPP_INFO(
    node_->get_logger(), "%s: Sending goal to check existence of object '%s'.",
    kName, object_id_.c_str());

  auto send_goal_options = rclcpp_action::Client<CheckObjectExists>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&CheckObjectExistsAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&CheckObjectExistsAction::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckObjectExistsAction::onRunning()
{
  static constexpr const char * kName = "CheckObjectExistsAction";

  if (result_received_) {
    // Set outputs
    setOutput("exists", action_result_.exists);
    setOutput("is_attached", action_result_.is_attached);
    setOutput("link_name", action_result_.link_name);

    if (action_result_.exists) {
      RCLCPP_INFO(
        node_->get_logger(), "%s: Object '%s' exists.", object_id_.c_str(), kName);
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_INFO(
        node_->get_logger(), "%s: Object '%s' does not exist.",
        kName, object_id_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void CheckObjectExistsAction::onHalted()
{
  static constexpr const char * kName = "CheckObjectExistsAction";

  RCLCPP_WARN(
    node_->get_logger(), "%s: onHalted() called. Cancelling goal if sent.", kName);

  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
    RCLCPP_INFO(node_->get_logger(), "%s: Goal canceled.", kName);
  }

  goal_sent_ = false;
  result_received_ = false;
}

void CheckObjectExistsAction::goalResponseCallback(
  std::shared_ptr<GoalHandleCheckObjectExists> goal_handle)
{
  static constexpr const char * kName = "CheckObjectExistsAction";

  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Goal was rejected by the server.", kName);
    result_received_ = true;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s: Goal accepted by the server, waiting for result.", kName);
  }
}

void CheckObjectExistsAction::resultCallback(
  const GoalHandleCheckObjectExists::WrappedResult & wrapped_result)
{
  static constexpr const char * kName = "CheckObjectExistsAction";

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "%s: Goal succeeded.", kName);
      action_result_ = *(wrapped_result.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "%s: Goal was aborted.", kName);
      action_result_.exists = false;
      action_result_.is_attached = false;
      action_result_.link_name = "";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "%s: Goal was canceled.", kName);
      action_result_.exists = false;
      action_result_.is_attached = false;
      action_result_.link_name = "";
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "%s: Unknown result code.", kName);
      action_result_.exists = false;
      action_result_.is_attached = false;
      action_result_.link_name = "";
      break;
  }

  result_received_ = true;
}

// ---------------------- GetObjectPoseAction ----------------------

GetObjectPoseAction::GetObjectPoseAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), goal_sent_(false), result_received_(false)
{
  static constexpr const char * kName = "GetObjectPoseAction";

  // Obtain the ROS node from the blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("%s: no blackboard provided.", kName);
  }
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("%s: 'node' not found in blackboard.", kName);
  }

  // Initialize the action client
  action_client_ = rclcpp_action::create_client<GetObjectPose>(node_, "get_object_pose");
  RCLCPP_INFO(node_->get_logger(), "%s: Waiting for 'get_object_pose' server...", kName);
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    throw BT::RuntimeError(
            "%s: 'get_object_pose' server not available after waiting.", kName);
  }
  RCLCPP_INFO(node_->get_logger(), "%s: Connected to 'get_object_pose' server.", kName);
}

BT::NodeStatus GetObjectPoseAction::onStart()
{
  static constexpr const char * kName = "GetObjectPoseAction";

  RCLCPP_DEBUG(node_->get_logger(), "%s: onStart() called.", kName);

  goal_sent_ = false;
  result_received_ = false;
  action_result_ = GetObjectPose::Result();
  std::string reference_link;
  if (!getInput<std::string>("link_name", reference_link)) {
    // default: empty => means "world" or "no specific link"
    reference_link = "";
  }

  // Retrieve input ports
  if (!getInput<std::string>("object_id", object_id_)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Missing required input 'object_id'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput<std::vector<double>>("pre_transform_xyz_rpy", pre_transform_xyz_rpy_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: Missing required input 'pre_transform_xyz_rpy'.", kName);
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s: 'pre_transform_xyz_rpy' = {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}", kName,
      pre_transform_xyz_rpy_[0], pre_transform_xyz_rpy_[1], pre_transform_xyz_rpy_[2],
      pre_transform_xyz_rpy_[3], pre_transform_xyz_rpy_[4], pre_transform_xyz_rpy_[5]);
  }

  if (!getInput<std::vector<double>>("post_transform_xyz_rpy", post_transform_xyz_rpy_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: Missing required input 'post_transform_xyz_rpy'.", kName);
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s: 'post_transform_xyz_rpy' = {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}", kName,
      post_transform_xyz_rpy_[0], post_transform_xyz_rpy_[1], post_transform_xyz_rpy_[2],
      post_transform_xyz_rpy_[3], post_transform_xyz_rpy_[4], post_transform_xyz_rpy_[5]);
  }

  if (!getInput<std::string>("pose_key", pose_key_)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Missing required input 'pose_key'.", kName);
    return BT::NodeStatus::FAILURE;
  }

  // Validate input sizes
  if (pre_transform_xyz_rpy_.size() != 6) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "%s: 'pre_transform_xyz_rpy' must have exactly 6 elements.", kName);
    return BT::NodeStatus::FAILURE;
  }

  if (post_transform_xyz_rpy_.size() != 6) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "%s: 'post_transform_xyz_rpy' must have exactly 6 elements.", kName);
    return BT::NodeStatus::FAILURE;
  }

  // Create and send the goal
  GetObjectPose::Goal goal_msg;
  goal_msg.object_id = object_id_;
  goal_msg.pre_transform_xyz_rpy = pre_transform_xyz_rpy_;
  goal_msg.post_transform_xyz_rpy = post_transform_xyz_rpy_;
  goal_msg.link_name = reference_link;

  RCLCPP_INFO(
    node_->get_logger(), "%s: Sending goal for object '%s'.", object_id_.c_str(), kName);

  auto send_goal_options = rclcpp_action::Client<GetObjectPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&GetObjectPoseAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&GetObjectPoseAction::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetObjectPoseAction::onRunning()
{
  static constexpr const char * kName = "GetObjectPoseAction";

  if (result_received_) {
    if (action_result_.success) {
      // Set the output port "pose"
      setOutput("pose", action_result_.pose);

      // **Important**: Also set the pose in the blackboard under pose_key_
      if (!pose_key_.empty()) {
        // Attempt to set the pose on the blackboard
        auto blackboard = config().blackboard;
        blackboard->set(pose_key_, action_result_.pose);
        RCLCPP_INFO(
          node_->get_logger(), "%s: Successfully set pose to blackboard key '%s'.",
          kName, pose_key_.c_str());
      } else {
        RCLCPP_ERROR(
          node_->get_logger(),
          "%s: pose_key_ is empty. Cannot set pose on blackboard.", kName);
        return BT::NodeStatus::FAILURE;
      }

      // Log the new pose
      RCLCPP_INFO(
        node_->get_logger(),
        "%s: New pose for '%s': Position (%.3f, %.3f, %.3f), Orientation (%.3f, "
        "%.3f, %.3f, %.3f)", kName,
        pose_key_.c_str(), action_result_.pose.position.x, action_result_.pose.position.y,
        action_result_.pose.position.z, action_result_.pose.orientation.x,
        action_result_.pose.orientation.y, action_result_.pose.orientation.z,
        action_result_.pose.orientation.w);

      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "%s: Failed to retrieve pose for object '%s'. Message: %s",
        kName, object_id_.c_str(), action_result_.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void GetObjectPoseAction::onHalted()
{
  static constexpr const char * kName = "GetObjectPoseAction";

  RCLCPP_WARN(
    node_->get_logger(), "%s: onHalted() called. Cancelling goal if sent.", kName);

  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
    RCLCPP_INFO(node_->get_logger(), "%s: Goal canceled.", kName);
  }

  goal_sent_ = false;
  result_received_ = false;
}

void GetObjectPoseAction::goalResponseCallback(std::shared_ptr<GoalHandleGetObjectPose> goal_handle)
{
  static constexpr const char * kName = "GetObjectPoseAction";

  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Goal was rejected by the server.", kName);
    result_received_ = true;
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "%s: Goal accepted by the server, waiting for result.", kName);
  }
}

void GetObjectPoseAction::resultCallback(
  const GoalHandleGetObjectPose::WrappedResult & wrapped_result)
{
  static constexpr const char * kName = "GetObjectPoseAction";

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "%s: Goal succeeded.", kName);
      action_result_ = *(wrapped_result.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "%s: Goal was aborted.", kName);
      action_result_.success = false;
      action_result_.message = "Action aborted.";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "%s: Goal was canceled.", kName);
      action_result_.success = false;
      action_result_.message = "Action canceled.";
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "%s: Unknown result code.", kName);
      action_result_.success = false;
      action_result_.message = "Unknown result code.";
      break;
  }

  result_received_ = true;
}

WaitForObjectAction::WaitForObjectAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  goal_sent_(false),
  result_received_(false),
  last_exists_(false),
  last_is_attached_(false)
{
  static constexpr const char * kName = "WaitForObjectAction";

  // Obtain the ROS node from the blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError("%s: no blackboard provided.", kName);
  }
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("%s: 'node' not found in blackboard.", kName);
  }

  // Initialize the action client
  action_client_ = rclcpp_action::create_client<CheckObjectExists>(node_, "check_object_exists");

  RCLCPP_INFO(
    node_->get_logger(), "%s: Waiting for 'check_object_exists' server...", kName);
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    throw BT::RuntimeError(
            "%s: 'check_object_exists' server not available after waiting.", kName);
  }
  RCLCPP_INFO(
    node_->get_logger(), "%s: Connected to 'check_object_exists' server.", kName);
}

BT::NodeStatus WaitForObjectAction::onStart()
{
  static constexpr const char * kName = "WaitForObjectAction";

  RCLCPP_DEBUG(node_->get_logger(), "%s: onStart() called.", kName);

  // Reset internal state
  goal_sent_ = false;
  result_received_ = false;
  last_exists_ = false;
  last_is_attached_ = false;
  last_link_name_.clear();

  // Read input ports
  if (!getInput<std::string>("object_id", object_id_)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Missing required input [object_id].", kName);
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput<bool>("exists", desired_exists_)) {
    RCLCPP_WARN(
      node_->get_logger(), "%s: Missing 'exists' => defaulting to 'true'", kName);
    desired_exists_ = true;
  }
  getInput<double>("timeout", timeout_);
  getInput<double>("poll_rate", poll_rate_);

  if (!getInput<std::string>("prefix", prefix_) || (prefix_ == "")) {
    prefix_ = "hmi_";
  }

  // Record the start time
  start_time_ = node_->now();
  next_check_time_ = start_time_;  // first check immediately

  RCLCPP_INFO(
    node_->get_logger(),
    "%s: Checking object [%s], desired_exists=%s, timeout=%.2f, poll_rate=%.2f",
    kName, object_id_.c_str(), (desired_exists_ ? "true" : "false"),
    timeout_, poll_rate_);

  // Return RUNNING so the BT engine will call onRunning()
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForObjectAction::onRunning()
{
  static constexpr const char * kName = "WaitForObjectAction";

  auto now = node_->now();

  // If we got a result from the last check
  if (result_received_) {
    // If the server says the object "exists" or "not exists"
    // matches what we want, we succeed
    bool condition_met = (last_exists_ == desired_exists_);
    if (condition_met) {
      // Fill output ports
      setOutput("exists", last_exists_);
      setOutput("is_attached", last_is_attached_);
      setOutput("link_name", last_link_name_);

      // HMI message
      setHMIMessage(config().blackboard, prefix_, "", "grey");

      RCLCPP_INFO(
        node_->get_logger(),
        "%s: Condition met: object '%s' => exists=%s -> SUCCESS.",
        kName, object_id_.c_str(), (last_exists_ ? "true" : "false"));
      return BT::NodeStatus::SUCCESS;
    }

    // Otherwise, check if we've timed out
    if (timeout_ > 0.0) {
      double elapsed = (now - start_time_).seconds();
      if (elapsed >= timeout_) {
        // HMI message
        setHMIMessage(
          config().blackboard, prefix_, "WAITING FOR OBJECT " + object_id_ + " TIMED OUT", "red");

        RCLCPP_WARN(
          node_->get_logger(),
          "%s: Timeout (%.1f s) reached for object '%s' -> FAILURE.",
          kName, timeout_, object_id_.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    // Not timed out => schedule next check in poll_rate_ seconds
    goal_sent_ = false;
    result_received_ = false;
    last_exists_ = false;
    last_is_attached_ = false;
    last_link_name_.clear();

    next_check_time_ = now + rclcpp::Duration::from_seconds(poll_rate_);
  }

  // If it is not yet time to check again, just return RUNNING
  if (now < next_check_time_) {
    return BT::NodeStatus::RUNNING;
  }

  // If we haven't sent a goal yet, do it now
  if (!goal_sent_) {
    sendCheckRequest();
  }

  // HMI message
  setHMIMessage(config().blackboard, prefix_, "WAITING FOR OBJECT: " + object_id_, "yellow");

  return BT::NodeStatus::RUNNING;
}

void WaitForObjectAction::onHalted()
{
  static constexpr const char * kName = "WaitForObjectAction";

  RCLCPP_WARN(
    node_->get_logger(),
    "%s: onHalted() called. Canceling any outstanding goals...", kName);

  if (goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
    RCLCPP_INFO(node_->get_logger(), "%s: Goal canceled.", kName);
  }

  goal_sent_ = false;
  result_received_ = false;
}

// ---------------------------------------------------
// Private Helpers
// ---------------------------------------------------
void WaitForObjectAction::sendCheckRequest()
{
  static constexpr const char * kName = "WaitForObjectAction";

  RCLCPP_DEBUG(node_->get_logger(), "%s: Sending check_object_exists goal.", kName);

  CheckObjectExists::Goal goal_msg;
  goal_msg.object_id = object_id_;

  auto send_goal_options = rclcpp_action::Client<CheckObjectExists>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&WaitForObjectAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&WaitForObjectAction::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);

  goal_sent_ = true;
  result_received_ = false;
}

void WaitForObjectAction::goalResponseCallback(
  std::shared_ptr<GoalHandleCheckObjectExists> goal_handle)
{
  static constexpr const char * kName = "WaitForObjectAction";

  if (!goal_handle) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "%s: Goal was rejected by the server => We'll keep retrying.", kName);
    // Mark as "object not found" so that we do another attempt
    result_received_ = true;
    last_exists_ = false;
  } else {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "%s: Goal accepted by the server, waiting for result...", kName);
  }
}

void WaitForObjectAction::resultCallback(
  const GoalHandleCheckObjectExists::WrappedResult & wrapped_result)
{
  static constexpr const char * kName = "WaitForObjectAction";

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
        RCLCPP_DEBUG(node_->get_logger(), "%s: Goal succeeded.", kName);
        auto result = wrapped_result.result;
        last_exists_ = result->exists;
        last_is_attached_ = result->is_attached;
        last_link_name_ = result->link_name;
        break;
      }
    case rclcpp_action::ResultCode::ABORTED: {
        RCLCPP_ERROR(node_->get_logger(), "%s: Goal was aborted by server.", kName);
        last_exists_ = false;
        break;
      }
    case rclcpp_action::ResultCode::CANCELED: {
        RCLCPP_WARN(node_->get_logger(), "%s: Goal canceled by server.", kName);
        last_exists_ = false;
        break;
      }
    default: {
        RCLCPP_ERROR(
          node_->get_logger(), "%s: Unknown result code => treat as 'not found'.", kName);
        last_exists_ = false;
        break;
      }
  }

  // We set result_received_ so onRunning() knows we can evaluate the outcome
  result_received_ = true;
}

}  // namespace manymove_cpp_trees
