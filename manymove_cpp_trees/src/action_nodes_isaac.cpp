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

#include "manymove_cpp_trees/action_nodes_isaac.hpp"

#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/header.hpp>

#include "manymove_cpp_trees/bt_converters.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// (Quaternion include handled by compat header)
// TF2 linear algebra (compat)
#include "manymove_cpp_trees/compat/tf2_linear_compat.hpp"

using namespace std::chrono_literals;

namespace manymove_cpp_trees
{

// ======================================================================
// GetEntityPoseNode
// ======================================================================
GetEntityPoseNode::GetEntityPoseNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("GetEntityPoseNode: missing 'node' in blackboard");
  }
  current_get_service_name_ = "/isaacsim/GetEntityState";
  get_client_ = node_->create_client<GetEntityState>(current_get_service_name_);
}

BT::NodeStatus GetEntityPoseNode::onStart()
{
  // Read fixed ports (strings naming BB keys)
  std::string service_name = current_get_service_name_;
  getInput("service_name", service_name);

  if (!getInput("entity_path_key", entity_path_) || entity_path_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing input 'entity_path_key'", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("pose_key", pose_key_) || pose_key_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing input 'pose_key'", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Read entity path from BB
  if (!config().blackboard->get(entity_path_, entity_path_) || entity_path_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] BB key '%s' not found or empty", name().c_str(),
      entity_path_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // (Re)create client if service name changed
  if (!get_client_ || service_name != current_get_service_name_) {
    current_get_service_name_ = service_name;
    get_client_ = node_->create_client<GetEntityState>(current_get_service_name_);
  }

  // If service not up yet, keep trying (node will be ticked again)
  if (!get_client_->wait_for_service(0s)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "[%s] Waiting for service '%s'...",
      name().c_str(), current_get_service_name_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  auto req = std::make_shared<GetEntityState::Request>();
  req->entity = entity_path_;

  auto pending = get_client_->async_send_request(req);
  future_ = pending.future.share();
  request_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetEntityPoseNode::onRunning()
{
  if (!request_sent_) {
    // Shouldn't really happen, but be defensive.
    return BT::NodeStatus::FAILURE;
  }

  if (future_.wait_for(0s) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  auto resp = future_.get();
  request_sent_ = false;

  if (!resp || resp->result.result != 1) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] GetEntityState error: %s", name().c_str(),
      resp ? resp->result.error_message.c_str() : "null response");
    return BT::NodeStatus::FAILURE;
  }

  const auto pose = resp->state.pose;
  config().blackboard->set(pose_key_, pose);
  setOutput("pose", pose);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] pose of '%s' -> BB['%s'] "
    "(%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f)",
    name().c_str(), entity_path_.c_str(), pose_key_.c_str(), pose.position.x, pose.position.y,
    pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z,
    pose.orientation.w);

  return BT::NodeStatus::SUCCESS;
}

void GetEntityPoseNode::onHalted()
{
  // nothing to cancel for services; just reset flags
  request_sent_ = false;
}

// ======================================================================
// SetEntityPoseNode
// ======================================================================
SetEntityPoseNode::SetEntityPoseNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("SetEntityPoseNode: missing 'node' in blackboard");
  }
  current_set_service_name_ = "/isaacsim/SetEntityState";
  set_client_ = node_->create_client<SetEntityState>(current_set_service_name_);
}

BT::NodeStatus SetEntityPoseNode::onStart()
{
  std::string service_name = current_set_service_name_;
  getInput("service_name", service_name);

  if (!getInput("entity_path_key", entity_path_) || entity_path_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing input 'entity_path_key'", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("pose_key", pose_key_) || pose_key_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing input 'pose_key'", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Resolve values from BB
  if (!config().blackboard->get(entity_path_, entity_path_) || entity_path_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] BB key '%s' not found or empty", name().c_str(),
      entity_path_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get(pose_key_, pose_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] BB key '%s' not found (Pose)", name().c_str(), pose_key_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // (Re)create client if service name changed
  if (!set_client_ || service_name != current_set_service_name_) {
    current_set_service_name_ = service_name;
    set_client_ = node_->create_client<SetEntityState>(current_set_service_name_);
  }

  if (!set_client_->wait_for_service(0s)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "[%s] Waiting for service '%s'...",
      name().c_str(), current_set_service_name_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  auto req = std::make_shared<SetEntityState::Request>();
  req->entity = entity_path_;
  req->state.header = std_msgs::msg::Header();  // default
  req->state.pose = pose_;
  req->state.twist = geometry_msgs::msg::Twist();         // zeros
  req->state.acceleration = geometry_msgs::msg::Accel();  // zeros

  auto pending = set_client_->async_send_request(req);
  future_ = pending.future.share();
  request_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetEntityPoseNode::onRunning()
{
  if (!request_sent_) {
    return BT::NodeStatus::FAILURE;
  }

  if (future_.wait_for(0s) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  auto resp = future_.get();
  request_sent_ = false;

  if (!resp || resp->result.result != 1) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] SetEntityState error: %s", name().c_str(),
      resp ? resp->result.error_message.c_str() : "null response");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] pose set on '%s' from BB['%s'] "
    "(%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f)",
    name().c_str(), entity_path_.c_str(), pose_key_.c_str(), pose_.position.x, pose_.position.y,
    pose_.position.z, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z,
    pose_.orientation.w);

  return BT::NodeStatus::SUCCESS;
}

void SetEntityPoseNode::onHalted() {request_sent_ = false;}

namespace
{
constexpr double kEpsilon = 1e-6;

inline tf2::Vector3 projectOntoPlane(const tf2::Vector3 & vector, const tf2::Vector3 & normal)
{
  return vector - (vector.dot(normal)) * normal;
}

inline tf2::Vector3 pickPerpendicularFallback(const tf2::Vector3 & axis)
{
  const tf2::Vector3 world_y(0.0, 1.0, 0.0);
  tf2::Vector3 candidate = projectOntoPlane(world_y, axis);
  if (candidate.length2() > kEpsilon) {
    candidate.normalize();
    return candidate;
  }

  const tf2::Vector3 world_down(0.0, 0.0, -1.0);
  candidate = projectOntoPlane(world_down, axis);
  if (candidate.length2() > kEpsilon) {
    candidate.normalize();
    return candidate;
  }

  const tf2::Vector3 world_x(1.0, 0.0, 0.0);
  candidate = projectOntoPlane(world_x, axis);
  if (candidate.length2() > kEpsilon) {
    candidate.normalize();
    return candidate;
  }

  return tf2::Vector3(0.0, 0.0, 1.0);
}

}  // namespace

geometry_msgs::msg::Pose align_foundationpose_orientation(
  const geometry_msgs::msg::Pose & input_pose, bool force_z_vertical)
{
  tf2::Quaternion source_q(
    input_pose.orientation.x, input_pose.orientation.y, input_pose.orientation.z,
    input_pose.orientation.w);
  if (source_q.length2() > 0.0) {
    source_q.normalize();
  }

  tf2::Matrix3x3 source_matrix(source_q);
  const tf2::Vector3 x_axis = source_matrix.getColumn(0).normalized();
  const tf2::Vector3 world_down(0.0, 0.0, -1.0);

  if (force_z_vertical) {
    tf2::Vector3 new_x = projectOntoPlane(x_axis, world_down);
    if (new_x.length2() < kEpsilon) {
      new_x = tf2::Vector3(1.0, 0.0, 0.0);
    } else {
      new_x.normalize();
    }

    tf2::Vector3 new_y = world_down.cross(new_x);
    if (new_y.length2() < kEpsilon) {
      new_y = tf2::Vector3(0.0, 1.0, 0.0);
    } else {
      new_y.normalize();
    }

    const tf2::Vector3 corrected_z = world_down;

    tf2::Matrix3x3 corrected_matrix(
      new_x.x(), new_y.x(), corrected_z.x(), new_x.y(), new_y.y(), corrected_z.y(), new_x.z(),
      new_y.z(), corrected_z.z());

    tf2::Quaternion corrected_q;
    corrected_matrix.getRotation(corrected_q);

    geometry_msgs::msg::Pose result = input_pose;
    result.orientation.x = corrected_q.x();
    result.orientation.y = corrected_q.y();
    result.orientation.z = corrected_q.z();
    result.orientation.w = corrected_q.w();
    return result;
  }

  tf2::Vector3 projected_vertical = projectOntoPlane(world_down, x_axis);
  tf2::Vector3 new_z;
  if (projected_vertical.length2() > kEpsilon) {
    new_z = projected_vertical.normalized();
  } else {
    new_z = pickPerpendicularFallback(x_axis);
  }

  if (new_z.length2() < kEpsilon) {
    new_z = tf2::Vector3(0.0, 0.0, -1.0);
  }

  if (new_z.dot(world_down) < 0.0) {
    new_z = -new_z;
  }

  tf2::Vector3 new_y = new_z.cross(x_axis);
  if (new_y.length2() < kEpsilon) {
    tf2::Vector3 helper = pickPerpendicularFallback(x_axis);
    new_z = helper;
    if (new_z.dot(world_down) < 0.0) {
      new_z = -new_z;
    }
    new_y = new_z.cross(x_axis);
  }

  new_y.normalize();
  tf2::Vector3 corrected_z = x_axis.cross(new_y).normalized();

  if (corrected_z.dot(world_down) < 0.0) {
    new_y = -new_y;
    corrected_z = -corrected_z;
  }

  tf2::Matrix3x3 corrected_matrix(
    x_axis.x(), new_y.x(), corrected_z.x(), x_axis.y(), new_y.y(), corrected_z.y(), x_axis.z(),
    new_y.z(), corrected_z.z());

  tf2::Quaternion corrected_q;
  corrected_matrix.getRotation(corrected_q);

  geometry_msgs::msg::Pose result = input_pose;
  result.orientation.x = corrected_q.x();
  result.orientation.y = corrected_q.y();
  result.orientation.z = corrected_q.z();
  result.orientation.w = corrected_q.w();
  return result;
}

namespace
{

struct FoundationPoseAdaptComputationResult
{
  bool success{false};
  bool retryable{false};
  std::string failure_from_frame;
  std::string failure_to_frame;
  std::string error_message;
  geometry_msgs::msg::Pose final_pose;
  geometry_msgs::msg::Pose approach_pose;
  geometry_msgs::msg::Pose corrected_pose;
  std_msgs::msg::Header header;
  bool compute_approach{false};
};

bool didFoundationPoseAdaptationTimeout(
  const rclcpp::Clock::SharedPtr & clock, const rclcpp::Time & start_time, double timeout_seconds)
{
  if (timeout_seconds <= 0.0) {
    return false;
  }
  const rclcpp::Duration elapsed = clock->now() - start_time;
  return elapsed.seconds() > timeout_seconds;
}

geometry_msgs::msg::Pose applyLocalXyzRpyTransform(
  const geometry_msgs::msg::Pose & base, const std::vector<double> & xyzrpy)
{
  std::vector<double> v(6, 0.0);
  for (size_t i = 0; i < std::min<size_t>(6, xyzrpy.size()); ++i) {
    v[i] = xyzrpy[i];
  }

  tf2::Quaternion q_base(
    base.orientation.x, base.orientation.y, base.orientation.z, base.orientation.w);
  if (q_base.length2() > 0.0) {
    q_base.normalize();
  }
  tf2::Matrix3x3 rotation_base(q_base);

  tf2::Quaternion q_delta;
  q_delta.setRPY(v[3], v[4], v[5]);

  tf2::Vector3 translation_delta(v[0], v[1], v[2]);
  tf2::Vector3 world_translation = rotation_base * translation_delta;

  geometry_msgs::msg::Pose out = base;
  out.position.x += world_translation.x();
  out.position.y += world_translation.y();
  out.position.z += world_translation.z();

  tf2::Quaternion q_out = q_base * q_delta;
  if (q_out.length2() > 0.0) {
    q_out.normalize();
  }
  out.orientation.x = q_out.x();
  out.orientation.y = q_out.y();
  out.orientation.z = q_out.z();
  out.orientation.w = q_out.w();
  return out;
}

bool loadFoundationPoseAdaptConfig(
  BT::TreeNode & tree_node, const rclcpp::Node::SharedPtr & node,
  FoundationPoseAdaptPortConfig & config, bool require_pick_pose_key,
  bool require_approach_pose_key, bool require_object_pose_key)
{
  config = FoundationPoseAdaptPortConfig{};

  if (require_pick_pose_key) {
    if (!tree_node.getInput(
        "pick_pose_key",
        config.pick_pose_key) || config.pick_pose_key.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(), "[%s] Missing required input 'pick_pose_key'",
        tree_node.name().c_str());
      return false;
    }
  } else {
    (void)tree_node.getInput("pick_pose_key", config.pick_pose_key);
  }

  if (require_approach_pose_key) {
    if (
      !tree_node.getInput("approach_pose_key", config.approach_pose_key) ||
      config.approach_pose_key.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(), "[%s] Missing required input 'approach_pose_key'",
        tree_node.name().c_str());
      return false;
    }
  } else {
    (void)tree_node.getInput("approach_pose_key", config.approach_pose_key);
  }

  if (require_object_pose_key) {
    if (
      !tree_node.getInput("object_pose_key", config.object_pose_key) ||
      config.object_pose_key.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(), "[%s] Missing required input 'object_pose_key'",
        tree_node.name().c_str());
      return false;
    }
  } else {
    (void)tree_node.getInput("object_pose_key", config.object_pose_key);
  }

  (void)tree_node.getInput("header_key", config.header_key);
  (void)tree_node.getInput("pick_transform", config.pick_transform);
  (void)tree_node.getInput("approach_transform", config.approach_transform);
  (void)tree_node.getInput("planning_frame", config.planning_frame);
  (void)tree_node.getInput("transform_timeout", config.transform_timeout);
  (void)tree_node.getInput("z_threshold_activation", config.z_threshold_activation);
  (void)tree_node.getInput("z_threshold", config.z_threshold);
  (void)tree_node.getInput("normalize_pose", config.normalize_pose);
  (void)tree_node.getInput("force_z_vertical", config.force_z_vertical);

  config.transform_timeout = std::max(0.0, config.transform_timeout);
  config.store_pick_pose = !config.pick_pose_key.empty();
  config.store_header = !config.header_key.empty();
  config.store_approach = !config.approach_pose_key.empty();
  config.store_object_pose = !config.object_pose_key.empty();
  return true;
}

FoundationPoseAdaptComputationResult computeFoundationPoseAdaptation(
  tf2_ros::Buffer & tf_buffer, const geometry_msgs::msg::Pose & raw_pose,
  std_msgs::msg::Header detection_header, const FoundationPoseAdaptPortConfig & config)
{
  FoundationPoseAdaptComputationResult result;
  const std::string planning_frame =
    config.planning_frame.empty() ? "world" : config.planning_frame;
  const std::string alignment_frame = "world";

  geometry_msgs::msg::PoseStamped detection_pose;
  detection_pose.header = detection_header;
  detection_pose.pose = raw_pose;
  detection_pose.header.stamp = rclcpp::Time(0);

  if (detection_pose.header.frame_id.empty()) {
    result.error_message = "Detection header has empty frame_id; cannot transform pose";
    return result;
  }

  geometry_msgs::msg::PoseStamped pose_in_alignment;
  try {
    pose_in_alignment = tf_buffer.transform(
      detection_pose, alignment_frame, tf2::durationFromSec(config.transform_timeout));
  } catch (const tf2::TransformException & ex) {
    result.retryable = true;
    result.failure_from_frame = detection_pose.header.frame_id;
    result.failure_to_frame = alignment_frame;
    result.error_message = ex.what();
    return result;
  }

  geometry_msgs::msg::Pose aligned_pose = pose_in_alignment.pose;
  if (config.normalize_pose) {
    aligned_pose = align_foundationpose_orientation(aligned_pose, config.force_z_vertical);
  }

  geometry_msgs::msg::PoseStamped aligned_pose_ps = pose_in_alignment;
  aligned_pose_ps.pose = aligned_pose;

  geometry_msgs::msg::Pose corrected_pose = aligned_pose;
  geometry_msgs::msg::PoseStamped transformed_pose = aligned_pose_ps;
  if (planning_frame != alignment_frame) {
    geometry_msgs::msg::PoseStamped world_pose_for_transform = aligned_pose_ps;
    world_pose_for_transform.header.stamp = rclcpp::Time(0);

    try {
      transformed_pose = tf_buffer.transform(
        world_pose_for_transform, planning_frame, tf2::durationFromSec(config.transform_timeout));
      corrected_pose = transformed_pose.pose;
    } catch (const tf2::TransformException & ex) {
      result.retryable = true;
      result.failure_from_frame = alignment_frame;
      result.failure_to_frame = planning_frame;
      result.error_message = ex.what();
      return result;
    }
  }

  if (config.z_threshold_activation && corrected_pose.position.z < config.z_threshold) {
    corrected_pose.position.z = config.z_threshold;
  }

  geometry_msgs::msg::Pose final_pose = corrected_pose;
  if (!config.pick_transform.empty()) {
    final_pose = applyLocalXyzRpyTransform(corrected_pose, config.pick_transform);
  }

  geometry_msgs::msg::Pose approach_pose = corrected_pose;
  const bool compute_approach = !config.approach_transform.empty() || config.store_approach;
  if (!config.approach_transform.empty()) {
    approach_pose = applyLocalXyzRpyTransform(corrected_pose, config.approach_transform);
  }

  std_msgs::msg::Header header = transformed_pose.header;
  if (header.frame_id.empty()) {
    header = detection_header;
    header.frame_id = planning_frame;
  } else {
    header.frame_id = planning_frame;
  }

  result.success = true;
  result.final_pose = final_pose;
  result.approach_pose = approach_pose;
  result.corrected_pose = corrected_pose;
  result.header = header;
  result.compute_approach = compute_approach;
  return result;
}

BT::NodeStatus publishFoundationPoseAdaptation(
  BT::TreeNode & tree_node, const rclcpp::Node::SharedPtr & node, tf2_ros::Buffer & tf_buffer,
  const geometry_msgs::msg::Pose & raw_pose, std_msgs::msg::Header detection_header,
  const FoundationPoseAdaptPortConfig & config, const rclcpp::Time & start_time,
  double timeout_seconds)
{
  auto clock = node->get_clock();
  auto result = computeFoundationPoseAdaptation(tf_buffer, raw_pose, detection_header, config);

  if (!result.success) {
    if (result.retryable) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock, 2000, "[%s] Failed to transform pose from '%s' to '%s': %s",
        tree_node.name().c_str(), result.failure_from_frame.c_str(),
        result.failure_to_frame.c_str(),
        result.error_message.c_str());

      if (didFoundationPoseAdaptationTimeout(clock, start_time, timeout_seconds)) {
        RCLCPP_ERROR(
          node->get_logger(), "[%s] Timed out waiting for TF transform to '%s'",
          tree_node.name().c_str(), result.failure_to_frame.c_str());
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    RCLCPP_ERROR(
      node->get_logger(), "[%s] %s", tree_node.name().c_str(), result.error_message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (config.store_pick_pose) {
    tree_node.config().blackboard->set(config.pick_pose_key, result.final_pose);
  }
  if (config.store_object_pose) {
    tree_node.config().blackboard->set(config.object_pose_key, result.corrected_pose);
  }
  tree_node.setOutput("pose", result.final_pose);

  if (config.store_approach) {
    tree_node.config().blackboard->set(config.approach_pose_key, result.approach_pose);
  }
  tree_node.setOutput("approach_pose", result.approach_pose);

  if (result.header.stamp.nanosec == 0 && result.header.stamp.sec == 0) {
    result.header.stamp = node->get_clock()->now();
  }
  if (config.store_header) {
    tree_node.config().blackboard->set(config.header_key, result.header);
  }
  tree_node.setOutput("header", result.header);

  RCLCPP_INFO(
    node->get_logger(), "[%s] published pose (%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f)%s",
    tree_node.name().c_str(), result.final_pose.position.x, result.final_pose.position.y,
    result.final_pose.position.z, result.final_pose.orientation.x, result.final_pose.orientation.y,
    result.final_pose.orientation.z, result.final_pose.orientation.w,
    result.compute_approach ? " with approach pose" : "");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace

FoundationPoseFetchTopicNode::FoundationPoseFetchTopicNode(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("FoundationPoseFetchTopicNode: missing 'node' in blackboard");
  }
}

void FoundationPoseFetchTopicNode::ensureSubscription(const std::string & topic)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (subscription_ && topic == current_topic_ && subscription_->get_topic_name() == topic) {
      return;
    }
    current_topic_ = topic;
  }

  auto callback = [this](DetectionArray::SharedPtr msg) {detectionCallback(std::move(msg));};
  auto new_subscription =
    node_->create_subscription<DetectionArray>(topic, rclcpp::SensorDataQoS(), callback);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    subscription_ = new_subscription;
  }
}

void FoundationPoseFetchTopicNode::detectionCallback(const DetectionArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_detection_ = *msg;
  have_message_ = true;
  ++message_sequence_;
}

std::optional<FoundationPoseFetchTopicNode::DetectionSelection>
FoundationPoseFetchTopicNode::pickDetection(const DetectionArray & array)
{
  std::optional<DetectionSelection> best_selection;
  double best_score = -1.0;

  for (const auto & detection : array.detections) {
    for (const auto & result : detection.results) {
      const auto & hypothesis = result.hypothesis;
      if (!target_id_.empty() && hypothesis.class_id != target_id_) {
        continue;
      }
      if (hypothesis.score < minimum_score_) {
        continue;
      }
      if (!best_selection || hypothesis.score > best_score) {
        best_selection = DetectionSelection{detection, result};
        best_score = hypothesis.score;
      }
    }
  }

  return best_selection;
}

BT::NodeStatus FoundationPoseFetchTopicNode::onStart()
{
  std::string topic = "pose_estimation/output";
  (void)getInput("input_topic", topic);
  (void)getInput("target_id", target_id_);
  (void)getInput("minimum_score", minimum_score_);
  (void)getInput("timeout", timeout_seconds_);

  ensureSubscription(topic);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_processed_sequence_ = message_sequence_;
  }

  start_time_ = node_->get_clock()->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FoundationPoseFetchTopicNode::onRunning()
{
  DetectionArray message_snapshot;
  bool has_new_message = false;
  std::string topic_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_message_ && message_sequence_ != last_processed_sequence_) {
      message_snapshot = latest_detection_;
      last_processed_sequence_ = message_sequence_;
      has_new_message = true;
    }
    topic_snapshot = current_topic_;
  }

  auto clock = node_->get_clock();

  if (!has_new_message) {
    if (didFoundationPoseAdaptationTimeout(clock, start_time_, timeout_seconds_)) {
      RCLCPP_WARN(
        node_->get_logger(), "[%s] Timed out waiting for detections on '%s'", name().c_str(),
        topic_snapshot.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  const auto selection = pickDetection(message_snapshot);
  if (!selection) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *clock, 5000,
      "[%s] No detection passed filters (target_id='%s', min_score=%.3f)", name().c_str(),
      target_id_.c_str(), minimum_score_);

    if (didFoundationPoseAdaptationTimeout(clock, start_time_, timeout_seconds_)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[%s] Timed out waiting for a valid detection (target_id='%s', min_score=%.3f)",
        name().c_str(), target_id_.c_str(), minimum_score_);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  std_msgs::msg::Header detection_header = selection->detection.header;
  if (detection_header.frame_id.empty()) {
    detection_header = message_snapshot.header;
  }

  setOutput("raw_pose", selection->result.pose.pose);
  setOutput("raw_header", detection_header);
  setOutput("raw_detections", message_snapshot);
  return BT::NodeStatus::SUCCESS;
}

void FoundationPoseFetchTopicNode::onHalted()
{
  // Keep the last detection snapshot.
}

FoundationPoseAdaptPoseNode::FoundationPoseAdaptPoseNode(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("FoundationPoseAdaptPoseNode: missing 'node' in blackboard");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus FoundationPoseAdaptPoseNode::onStart()
{
  if (!getInput("raw_pose", raw_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input 'raw_pose'", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("raw_header", raw_header_)) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input 'raw_header'", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!loadFoundationPoseAdaptConfig(*this, node_, adapt_config_, true, false, false)) {
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput("timeout", timeout_seconds_);
  start_time_ = node_->get_clock()->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FoundationPoseAdaptPoseNode::onRunning()
{
  return publishFoundationPoseAdaptation(
    *this, node_, *tf_buffer_, raw_pose_, raw_header_, adapt_config_, start_time_,
    timeout_seconds_);
}

void FoundationPoseAdaptPoseNode::onHalted()
{
  // No specific action required.
}

FoundationPoseAlignmentNode::FoundationPoseAlignmentNode(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("FoundationPoseAlignmentNode: missing 'node' in blackboard");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void FoundationPoseAlignmentNode::ensureSubscription(const std::string & topic)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (subscription_ && topic == current_topic_ && subscription_->get_topic_name() == topic) {
      return;
    }
    current_topic_ = topic;
  }

  auto callback = [this](DetectionArray::SharedPtr msg) {detectionCallback(std::move(msg));};
  auto new_subscription =
    node_->create_subscription<DetectionArray>(topic, rclcpp::SensorDataQoS(), callback);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    subscription_ = new_subscription;
  }
}

void FoundationPoseAlignmentNode::detectionCallback(const DetectionArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_detection_ = *msg;
  have_message_ = true;
  ++message_sequence_;
}

std::optional<FoundationPoseAlignmentNode::DetectionSelection>
FoundationPoseAlignmentNode::pickDetection(const DetectionArray & array)
{
  std::optional<DetectionSelection> best_selection;
  double best_score = -1.0;

  for (const auto & detection : array.detections) {
    for (const auto & result : detection.results) {
      const auto & hypothesis = result.hypothesis;
      if (!target_id_.empty() && hypothesis.class_id != target_id_) {
        continue;
      }
      if (hypothesis.score < minimum_score_) {
        continue;
      }
      if (!best_selection || hypothesis.score > best_score) {
        best_selection = DetectionSelection{detection, result};
        best_score = hypothesis.score;
      }
    }
  }

  return best_selection;
}

BT::NodeStatus FoundationPoseAlignmentNode::onStart()
{
  // Topic is always used, but has a sensible default in providedPorts().
  // Treat missing as optional and fall back to the default.
  std::string topic = "pose_estimation/output";
  (void)getInput("input_topic", topic);

  if (!loadFoundationPoseAdaptConfig(*this, node_, adapt_config_, true, true, true)) {
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput("target_id", target_id_);
  (void)getInput("minimum_score", minimum_score_);
  (void)getInput("timeout", timeout_seconds_);

  ensureSubscription(topic);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_processed_sequence_ = message_sequence_;
  }

  start_time_ = node_->get_clock()->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FoundationPoseAlignmentNode::onRunning()
{
  DetectionArray message_snapshot;
  bool has_new_message = false;
  std::string topic_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_message_ && message_sequence_ != last_processed_sequence_) {
      message_snapshot = latest_detection_;
      last_processed_sequence_ = message_sequence_;
      has_new_message = true;
    }
    topic_snapshot = current_topic_;
  }

  auto clock = node_->get_clock();

  if (!has_new_message) {
    if (timeout_seconds_ > 0.0) {
      const rclcpp::Duration elapsed = clock->now() - start_time_;
      if (elapsed.seconds() > timeout_seconds_) {
        RCLCPP_WARN(
          node_->get_logger(), "[%s] Timed out waiting for detections on '%s'", name().c_str(),
          topic_snapshot.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  const auto selection = pickDetection(message_snapshot);
  if (!selection) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *clock, 5000,
      "[%s] No detection passed filters (target_id='%s', min_score=%.3f)", name().c_str(),
      target_id_.c_str(), minimum_score_);

    if (timeout_seconds_ > 0.0) {
      const rclcpp::Duration elapsed = clock->now() - start_time_;
      if (elapsed.seconds() > timeout_seconds_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[%s] Timed out waiting for a valid detection (target_id='%s', min_score=%.3f)",
          name().c_str(), target_id_.c_str(), minimum_score_);
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  std_msgs::msg::Header detection_header = selection->detection.header;
  if (detection_header.frame_id.empty()) {
    detection_header = message_snapshot.header;
  }

  return publishFoundationPoseAdaptation(
    *this, node_, *tf_buffer_, selection->result.pose.pose, detection_header, adapt_config_,
    start_time_, timeout_seconds_);
}

void FoundationPoseAlignmentNode::onHalted()
{
  // No specific action required; keep the last detection
}

}  // namespace manymove_cpp_trees
