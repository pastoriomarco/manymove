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

#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP

#include <string>
#include <memory>
#include <future>
#include <mutex>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <simulation_interfaces/srv/get_entity_state.hpp>
#include <simulation_interfaces/srv/set_entity_state.hpp>

namespace manymove_cpp_trees
{

// ======================================================================
// GetEntityPoseNode (async, non-blocking)
// ======================================================================
/**
 * Node summary
 * - Queries Isaac Sim (simulation_interfaces/GetEntityState) to read the pose of an entity.
 * - Asynchronous and non-blocking: returns RUNNING while waiting for the service response.
 * - Writes the retrieved Pose to a blackboard key and also emits it on an output port.
 *
 * Ports
 * Required inputs (always used):
 * - entity_path_key (string): name of a blackboard key that stores the entity path string (e.g.
 *"/World/Cube").
 * - pose_key (string): name of a blackboard key to write the retrieved Pose into.
 *
 * Optional inputs:
 * - service_name (string, default: "/isaacsim/GetEntityState"): service to call.
 *
 * Outputs:
 * - pose (geometry_msgs/Pose): the retrieved Pose (also stored into the blackboard key specified by
 * pose_key).
 *
 * Behavior & failures
 * - If the service is not available yet, the node keeps returning RUNNING (logs throttled
 * warnings).
 * - FAILS if the required blackboard keys are not provided or if the service returns an error.
 */
class GetEntityPoseNode : public BT::StatefulActionNode
{
  public:
  using GetEntityState = simulation_interfaces::srv::GetEntityState;
  using GetClient = rclcpp::Client<GetEntityState>;
  using GetFuture = GetClient::SharedFuture;

  explicit GetEntityPoseNode(const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("service_name",
	  "/isaacsim/GetEntityState",
	  "GetEntityState service name"),
        // name of a BB key that stores the entity path string
        BT::InputPort<std::string>("entity_path_key",
	  "Blackboard key holding the entity path string"),
        // name of a BB key where we should store the retrieved Pose
        BT::InputPort<std::string>("pose_key",
	  "Blackboard key to write the retrieved Pose"),
        // optional direct output
        BT::OutputPort<geometry_msgs::msg::Pose>("pose", "Retrieved Pose")
      };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<GetClient> get_client_;
  std::string current_get_service_name_;

// per-run state
  bool request_sent_
  {
    false
  };
  std::string entity_path_;
  std::string pose_key_;
  GetFuture future_;
};

// ======================================================================
// SetEntityPoseNode (async, non-blocking)
// ======================================================================
/**
 * Node summary
 * - Calls Isaac Sim (simulation_interfaces/SetEntityState) to set the pose of an entity.
 * - Asynchronous and non-blocking: returns RUNNING while waiting for the service response.
 * - Reads the entity path and the desired Pose from blackboard keys.
 *
 * Ports
 * Required inputs (always used):
 * - entity_path_key (string): name of a blackboard key that stores the entity path string.
 * - pose_key (string): name of a blackboard key that stores the Pose to set.
 *
 * Optional inputs:
 * - service_name (string, default: "/isaacsim/SetEntityState"): service to call.
 *
 * Outputs: none
 *
 * Behavior & failures
 * - If the service is not available yet, the node keeps returning RUNNING (logs throttled
 * warnings).
 * - FAILS if the required blackboard keys are missing, or the service returns an error.
 */
class SetEntityPoseNode : public BT::StatefulActionNode
{
  public:
  using SetEntityState = simulation_interfaces::srv::SetEntityState;
  using SetClient = rclcpp::Client<SetEntityState>;
  using SetFuture = SetClient::SharedFuture;

  explicit SetEntityPoseNode(const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("service_name",
	  "/isaacsim/SetEntityState",
	  "SetEntityState service name"),
        // name of a BB key that stores the entity path string
        BT::InputPort<std::string>("entity_path_key",
	  "Blackboard key holding the entity path string"),
        // name of a BB key that stores the Pose to set
        BT::InputPort<std::string>("pose_key",
	  "Blackboard key holding the Pose to set")
      };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SetClient> set_client_;
  std::string current_set_service_name_;

// per-run state
  bool request_sent_
  {
    false
  };
  std::string entity_path_;
  std::string pose_key_;
  geometry_msgs::msg::Pose pose_;
  SetFuture future_;
};

/**
 * @brief Normalize the orientation of a pose for grasp alignment.
 *
 * The returned pose is a copy of @p input_pose with the position preserved and
 * the orientation (quaternion) potentially adjusted to form an orthonormal frame
 * consistent with world vertical. The world "down" direction is assumed to be (0, 0, -1).
 *
 * Behavior by @p force_z_vertical:
 *   - true  => Force the local Z axis to be exactly vertical (world down). The X axis is
 *              recomputed as the original X projected onto the horizontal plane; Y is Z × X.
 *              This changes both Z and X to guarantee Z is perfectly vertical.
 *   - false => Preserve the original X axis exactly. Choose Z to be as close as possible to
 *              world down while remaining orthogonal to X (rotate about X); Y is Z × X.
 *
 * Robust fallbacks are used if the input quaternion is degenerate or nearly aligned with
 * world axes; the output quaternion is normalized.
 *
 * @param input_pose       Source pose. Position is returned unchanged.
 * @param force_z_vertical If true, force Z to world vertical; otherwise keep X and make Z as
 *                         vertical as possible subject to X orthogonality.
 * @return Pose with adjusted orientation quaternion.
 */
geometry_msgs::msg::Pose align_foundationpose_orientation(
  const geometry_msgs::msg::Pose & input_pose,
  bool force_z_vertical = false);

/**
 * Node summary
 * - Subscribes to a FoundationPose Detection3DArray topic, selects a detection according to
 * filters,
 *   transforms and optionally normalizes the pose, applies local offsets, and returns/stores
 * results.
 * - Asynchronous: returns RUNNING while waiting for messages or TF transforms.
 *
 * Ports
 * Always used (but many have defaults):
 * - input_topic (string, default: "pose_estimation/output"): Detection3DArray topic.
 * - pick_pose_key (string, REQUIRED): blackboard key to store the final pick pose. If empty or
 * missing, the node
 *   will not store into the blackboard (only outputs), but downstream behaviors that expect the key
 * may fail.
 * - planning_frame (string, default: "world"): target frame for outputs (pose, approach_pose,
 * header).
 * - transform_timeout (double, default: 0.1): TF timeout when transforming poses to planning_frame.
 * - minimum_score (double, default: 0.0): discard results with lower score.
 * - timeout (double, default: 1.0): how long to wait for a valid detection (<=0 means forever).
 *
 * Optional inputs (used only if provided/non-empty):
 * - target_id (string, default: ""): filter by class id; empty accepts any class.
 * - header_key (string, default: ""): blackboard key to store the (possibly rewritten) header.
 * - approach_pose_key (string, default: ""): blackboard key to store the computed approach pose.
 * - object_pose_key (string, default: ""): blackboard key to store the aligned pose before local
 * offsets
 *   (useful for planning scene updates).
 * - pick_transform (double[6], xyzrpy, optional): local transform applied after alignment (pick
 * pose).
 * - approach_transform (double[6], xyzrpy, optional): local transform applied after alignment
 *(approach pose).
 * - z_threshold_activation (bool, default: false) and z_threshold (double, default: 0.0): when
 * enabled, enforce a
 *   minimum Z value in planning_frame before applying local transforms.
 * - normalize_pose (bool, default: false): when true, normalize the orientation to align with
 * vertical.
 * - force_z_vertical (bool, default: false): when normalizing, forces Z axis to be exactly
 * vertical.
 *
 * Outputs
 * - pose (geometry_msgs/Pose): final pick pose after alignment and pick_transform.
 * - approach_pose (geometry_msgs/Pose): approach pose after alignment and approach_transform.
 * - header (std_msgs/Header): header corresponding to the output poses (in planning_frame).
 *
 * Behavior & failures
 * - RUNNING while waiting for new detections or TF transforms.
 * - FAILS if no valid detection arrives within timeout, or if TF transform to alignment/planning
 * frames times out.
 */
class FoundationPoseAlignmentNode : public BT::StatefulActionNode
{
  public:
  using DetectionArray = vision_msgs::msg::Detection3DArray;

  explicit FoundationPoseAlignmentNode(const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("input_topic",
	  "pose_estimation/output",
	  "Detection3DArray topic from FoundationPose"),
        BT::InputPort<std::string>("pick_pose_key",
	  "Blackboard key to write the aligned pick pose"),
        BT::InputPort<std::string>("header_key", "",
	  "Blackboard key to write the detection header"),
        BT::InputPort<std::string>("target_id", "",
	  "Filter detections by class id (empty = any)"),
        BT::InputPort<double>("minimum_score", 0.0,
	  "Minimum hypothesis score to accept"),
        BT::InputPort<double>("timeout", 1.0,
	  "Seconds to wait for a valid detection (<=0: wait forever)"),
        BT::InputPort<std::vector<double> >("pick_transform",
	  "Local transform [x,y,z,r,p,y] applied after alignment to 'pose'"),
        BT::InputPort<std::vector<double> >("approach_transform",
	  "Local transform [x,y,z,r,p,y] applied after alignment to 'approach_pose'"),
        BT::InputPort<std::string>("approach_pose_key", "",
	  "Blackboard key to write computed approach pose"),
        BT::InputPort<std::string>("object_pose_key", "",
	  "Blackboard key to write the aligned pose for planning scene"),
        BT::InputPort<std::string>("planning_frame", "world",
	  "Frame where the aligned pose should be expressed"),
        BT::InputPort<double>("transform_timeout", 0.1,
	  "Timeout (s) when waiting for TF transform to the planning frame"),
        BT::InputPort<bool>("z_threshold_activation", false,
	  "Enable enforcement of a minimum Z value for the pose"),
        BT::InputPort<double>("z_threshold", 0.0,
	  "Minimum allowed Z value when the threshold is enabled"),
        BT::InputPort<bool>("normalize_pose", false,
	  "If false, skip orientation normalization; if true, apply alignment"),
        BT::InputPort<bool>("force_z_vertical", false,
	  "If true, align the pose so its Z axis is perfectly vertical"),
        BT::OutputPort<geometry_msgs::msg::Pose>("pose",
	  "Aligned pose output"),
        BT::OutputPort<geometry_msgs::msg::Pose>("approach_pose",
	  "Aligned approach pose output"),
        BT::OutputPort<std_msgs::msg::Header>("header",
	  "Header associated with the aligned detection")
      };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  private:
  struct DetectionSelection
  {
    vision_msgs::msg::Detection3D detection;
    vision_msgs::msg::ObjectHypothesisWithPose result;
  };

  void ensureSubscription(const std::string & topic);
  void detectionCallback(const DetectionArray::SharedPtr msg);
  std::optional<DetectionSelection> pickDetection(const DetectionArray & array);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<DetectionArray>::SharedPtr subscription_;
  std::string current_topic_;

  std::mutex mutex_;
  DetectionArray latest_detection_;
  bool have_message_
  {
    false
  };
  uint64_t message_sequence_
  {
    0
  };
  uint64_t last_processed_sequence_
  {
    0
  };

  rclcpp::Time start_time_;
  double timeout_seconds_
  {
    0.0
  };
  double minimum_score_
  {
    0.0
  };
  std::string target_id_;
  std::string pick_pose_key_;
  std::string header_key_;
  std::string approach_pose_key_;
  std::string object_pose_key_;
  std::vector<double> pick_transform_;
  std::vector<double> approach_transform_;
  bool z_threshold_activation_
  {
    false
  };
  double z_threshold_
  {
    0.0
  };
  bool normalize_pose_
  {
    false
  };
  bool force_z_vertical_
  {
    false
  };
  bool store_pick_pose_
  {
    false
  };
  bool store_header_
  {
    false
  };
  bool store_approach_
  {
    false
  };
  bool store_object_pose_
  {
    false
  };
  std::string planning_frame_
  {
    "world"
  };
  double transform_timeout_
  {
    0.1
  };

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP
