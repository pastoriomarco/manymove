#include "manymove_cpp_trees/action_nodes_isaac.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#  include <tf2/LinearMath/Quaternion.hpp>
#else
#  include <tf2/LinearMath/Quaternion.h>
#endif
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace manymove_cpp_trees
{

// ======================================================================
// GetEntityPoseNode
// ======================================================================
GetEntityPoseNode::GetEntityPoseNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
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
      node_->get_logger(), "[%s] BB key '%s' not found or empty",
      name().c_str(), entity_path_.c_str());
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
      node_->get_logger(), *node_->get_clock(), 2000,
      "[%s] Waiting for service '%s'...",
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
      node_->get_logger(), "[%s] GetEntityState error: %s",
      name().c_str(),
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
    name().c_str(), entity_path_.c_str(), pose_key_.c_str(),
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

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
SetEntityPoseNode::SetEntityPoseNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
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
      node_->get_logger(), "[%s] BB key '%s' not found or empty",
      name().c_str(), entity_path_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get(pose_key_, pose_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] BB key '%s' not found (Pose)",
      name().c_str(), pose_key_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // (Re)create client if service name changed
  if (!set_client_ || service_name != current_set_service_name_) {
    current_set_service_name_ = service_name;
    set_client_ = node_->create_client<SetEntityState>(current_set_service_name_);
  }

  if (!set_client_->wait_for_service(0s)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[%s] Waiting for service '%s'...",
      name().c_str(), current_set_service_name_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  auto req = std::make_shared<SetEntityState::Request>();
  req->entity = entity_path_;
  req->state.header = std_msgs::msg::Header();       // default
  req->state.pose = pose_;
  req->state.twist = geometry_msgs::msg::Twist();              // zeros
  req->state.acceleration = geometry_msgs::msg::Accel();       // zeros

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
      node_->get_logger(), "[%s] SetEntityState error: %s",
      name().c_str(),
      resp ? resp->result.error_message.c_str() : "null response");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] pose set on '%s' from BB['%s'] "
    "(%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f)",
    name().c_str(), entity_path_.c_str(), pose_key_.c_str(),
    pose_.position.x, pose_.position.y, pose_.position.z,
    pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);

  return BT::NodeStatus::SUCCESS;
}

void SetEntityPoseNode::onHalted()
{
  request_sent_ = false;
}

namespace
{
constexpr double kEpsilon = 1e-6;

inline tf2::Vector3 projectOntoPlane(
  const tf2::Vector3 & vector,
  const tf2::Vector3 & normal)
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

}     // namespace

geometry_msgs::msg::Pose align_foundationpose_orientation(
  const geometry_msgs::msg::Pose & input_pose,
  bool force_z_vertical)
{
  tf2::Quaternion source_q(input_pose.orientation.x,
    input_pose.orientation.y,
    input_pose.orientation.z,
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
      new_x.x(), new_y.x(), corrected_z.x(),
      new_x.y(), new_y.y(), corrected_z.y(),
      new_x.z(), new_y.z(), corrected_z.z());

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
    x_axis.x(), new_y.x(), corrected_z.x(),
    x_axis.y(), new_y.y(), corrected_z.y(),
    x_axis.z(), new_y.z(), corrected_z.z());

  tf2::Quaternion corrected_q;
  corrected_matrix.getRotation(corrected_q);

  geometry_msgs::msg::Pose result = input_pose;
  result.orientation.x = corrected_q.x();
  result.orientation.y = corrected_q.y();
  result.orientation.z = corrected_q.z();
  result.orientation.w = corrected_q.w();
  return result;
}

FoundationPoseAlignmentNode::FoundationPoseAlignmentNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
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
  if (subscription_ && topic == current_topic_ && subscription_->get_topic_name() == topic) {
    return;
  }

  current_topic_ = topic;
  auto callback = [this](DetectionArray::SharedPtr msg)
    {
      detectionCallback(std::move(msg));
    };
  subscription_ = node_->create_subscription<DetectionArray>(
    topic, rclcpp::SensorDataQoS(), callback);
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

  if (!getInput("pick_pose_key", pick_pose_key_) || pick_pose_key_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] Missing required input 'pick_pose_key'",
      name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("approach_pose_key", approach_pose_key_) || approach_pose_key_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] Missing required input 'approach_pose_key'",
      name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("object_pose_key", object_pose_key_) || object_pose_key_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] Missing required input 'object_pose_key'",
      name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  // Optional inputs: if not provided, keep pre-initialized defaults
  (void)getInput("header_key", header_key_);
  (void)getInput("target_id", target_id_);
  (void)getInput("minimum_score", minimum_score_);
  (void)getInput("timeout", timeout_seconds_);
  // (void)getInput("approach_pose_key", approach_pose_key_);
  // (void)getInput("object_pose_key", object_pose_key_);
  (void)getInput("pick_transform", pick_transform_);
  (void)getInput("approach_transform", approach_transform_);
  (void)getInput("planning_frame", planning_frame_);
  (void)getInput("transform_timeout", transform_timeout_);
  (void)getInput("z_threshold_activation", z_threshold_activation_);
  (void)getInput("z_threshold", z_threshold_);
  (void)getInput("normalize_pose", normalize_pose_);
  (void)getInput("force_z_vertical", force_z_vertical_);
  transform_timeout_ = std::max(0.0, transform_timeout_);

  store_pick_pose_ = !pick_pose_key_.empty();
  store_header_ = !header_key_.empty();
  store_approach_ = !approach_pose_key_.empty();
  store_object_pose_ = !object_pose_key_.empty();

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
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_message_ && message_sequence_ != last_processed_sequence_) {
      message_snapshot = latest_detection_;
      last_processed_sequence_ = message_sequence_;
      has_new_message = true;
    }
  }

  auto clock = node_->get_clock();

  if (!has_new_message) {
    if (timeout_seconds_ > 0.0) {
      const rclcpp::Duration elapsed = clock->now() - start_time_;
      if (elapsed.seconds() > timeout_seconds_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[%s] Timed out waiting for detections on '%s'",
          name().c_str(), current_topic_.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  const auto selection = pickDetection(message_snapshot);
  if (!selection) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *clock, 5000,
      "[%s] No detection passed filters (target_id='%s', min_score=%.3f)",
      name().c_str(), target_id_.c_str(), minimum_score_);

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

  if (planning_frame_.empty()) {
    planning_frame_ = "world";
  }

  const std::string alignment_frame = "world";
  geometry_msgs::msg::Pose raw_pose = selection->result.pose.pose;
  geometry_msgs::msg::PoseStamped detection_pose;
  detection_pose.header = detection_header;
  detection_pose.pose = raw_pose;
  detection_pose.header.stamp = rclcpp::Time(0);

  geometry_msgs::msg::PoseStamped pose_in_alignment;
  if (detection_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] Detection header has empty frame_id; cannot transform pose",
      name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  try {
    pose_in_alignment = tf_buffer_->transform(
      detection_pose, alignment_frame, tf2::durationFromSec(transform_timeout_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *clock, 2000,
      "[%s] Failed to transform pose from '%s' to '%s': %s",
      name().c_str(), detection_pose.header.frame_id.c_str(),
      alignment_frame.c_str(), ex.what());
    if (timeout_seconds_ > 0.0) {
      const rclcpp::Duration elapsed = clock->now() - start_time_;
      if (elapsed.seconds() > timeout_seconds_) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "[%s] Timed out waiting for TF transform to '%s'", name().c_str(),
          alignment_frame.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  // Note: Z-threshold will be applied in the planning frame later,
  // so we do not modify pose_in_alignment here.

  geometry_msgs::msg::Pose aligned_pose = pose_in_alignment.pose;
  if (normalize_pose_) {
    aligned_pose = align_foundationpose_orientation(
      pose_in_alignment.pose, force_z_vertical_);
  }

  geometry_msgs::msg::PoseStamped aligned_pose_ps = pose_in_alignment;
  aligned_pose_ps.pose = aligned_pose;

  geometry_msgs::msg::Pose corrected_pose = aligned_pose;
  geometry_msgs::msg::PoseStamped transformed_pose = aligned_pose_ps;
  if (planning_frame_ != alignment_frame) {
    geometry_msgs::msg::PoseStamped world_pose_for_transform = aligned_pose_ps;
    world_pose_for_transform.header.stamp = rclcpp::Time(0);

    try {
      transformed_pose = tf_buffer_->transform(
        world_pose_for_transform, planning_frame_,
        tf2::durationFromSec(transform_timeout_));
      corrected_pose = transformed_pose.pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock, 2000,
        "[%s] Failed to transform pose from '%s' to '%s': %s",
        name().c_str(), alignment_frame.c_str(), planning_frame_.c_str(),
        ex.what());

      if (timeout_seconds_ > 0.0) {
        const rclcpp::Duration elapsed = clock->now() - start_time_;
        if (elapsed.seconds() > timeout_seconds_) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] Timed out waiting for TF transform to '%s'", name().c_str(),
            planning_frame_.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  // Apply Z threshold in the planning frame (final output frame)
  if (z_threshold_activation_ && corrected_pose.position.z < z_threshold_) {
    corrected_pose.position.z = z_threshold_;
  }

  // Helper to apply a local XYZRPY transform (6 elements) to a pose: T_out = T_pose * T_delta
  auto apply_local_xyzrpy = [](const geometry_msgs::msg::Pose & base,
      const std::vector<double> & xyzrpy) -> geometry_msgs::msg::Pose
    {
      std::vector<double> v(6, 0.0);
      for (size_t i = 0; i < std::min<size_t>(6, xyzrpy.size()); ++i) {
        v[i] = xyzrpy[i];
      }

      tf2::Quaternion q_base(base.orientation.x, base.orientation.y, base.orientation.z,
        base.orientation.w);
      if (q_base.length2() > 0.0) {
        q_base.normalize();
      }
      tf2::Matrix3x3 R_base(q_base);

      tf2::Quaternion q_delta;
      q_delta.setRPY(v[3], v[4], v[5]);

      tf2::Vector3 t_delta(v[0], v[1], v[2]);
      tf2::Vector3 t_world = R_base * t_delta;       // local -> world translation

      geometry_msgs::msg::Pose out = base;
      out.position.x += t_world.x();
      out.position.y += t_world.y();
      out.position.z += t_world.z();

      tf2::Quaternion q_out = q_base * q_delta;       // local rotation composition
      if (q_out.length2() > 0.0) {
        q_out.normalize();
      }
      out.orientation.x = q_out.x();
      out.orientation.y = q_out.y();
      out.orientation.z = q_out.z();
      out.orientation.w = q_out.w();
      return out;
    };

  // Compute final pick pose (pose output) by applying pick_transform after Z-thresholding
  geometry_msgs::msg::Pose final_pose = corrected_pose;
  if (!pick_transform_.empty()) {
    final_pose = apply_local_xyzrpy(corrected_pose, pick_transform_);
  }

  if (store_pick_pose_) {
    config().blackboard->set(pick_pose_key_, final_pose);
  }
  if (store_object_pose_) {
    config().blackboard->set(object_pose_key_, corrected_pose);
  }
  setOutput("pose", final_pose);

  geometry_msgs::msg::Pose approach_pose = corrected_pose;
  bool compute_approach = !approach_transform_.empty() || store_approach_;
  if (!approach_transform_.empty()) {
    approach_pose = apply_local_xyzrpy(corrected_pose, approach_transform_);
  }
  if (store_approach_) {
    config().blackboard->set(approach_pose_key_, approach_pose);
  }
  setOutput("approach_pose", approach_pose);

  std_msgs::msg::Header header = transformed_pose.header;
  if (header.frame_id.empty()) {
    header = detection_header;
    header.frame_id = planning_frame_;
    header.stamp = node_->get_clock()->now();
  } else {
    header.frame_id = planning_frame_;
  }
  if (store_header_) {
    config().blackboard->set(header_key_, header);
  }
  setOutput("header", header);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] published pose (%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f)%s",
    name().c_str(),
    final_pose.position.x, final_pose.position.y, final_pose.position.z,
    final_pose.orientation.x, final_pose.orientation.y,
    final_pose.orientation.z, final_pose.orientation.w,
    compute_approach ? " with approach pose" : "");

  return BT::NodeStatus::SUCCESS;
}

void FoundationPoseAlignmentNode::onHalted()
{
  // No specific action required; keep the last detection
}

} // namespace manymove_cpp_trees
