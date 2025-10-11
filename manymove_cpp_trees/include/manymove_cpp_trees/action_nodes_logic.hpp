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

#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/decorator_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <cmath>

namespace manymove_cpp_trees
{
/**
 * @brief A custom retry node that keeps retrying indefinitely but
 *        if the "reset" blackboard key is true, it halts the child and returns FAILURE;
 *        if the "stop_execution" key is true, it halts the child and returns RUNNING (i.e. it
 * pauses execution).
 *        Otherwise, it ticks its child.
 */
class RetryPauseResetNode : public BT::DecoratorNode
{
  public:
  RetryPauseResetNode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<bool>("stop_execution", false, "Pause execution when true"),
        BT::InputPort<bool>("collision_detected", false,
	  "Stops current move when true, then retries planning"),
        BT::InputPort<bool>("reset", false, "Reset branch when true"),
        BT::InputPort<std::string>("robot_prefix",
	  "Robot prefix for setting correct blackboard key")
      };
  }

  BT::NodeStatus tick() override;
  void halt() override;
};

/**
 * @class CheckKeyBoolValue
 * @brief A simple condition node that checks if a blackboard key
 *        matches an expected string value.
 */
class CheckKeyBoolValue : public BT::ConditionNode
{
  public:
/**
 * @brief Constructor
 * @param name The node's name in the XML
 * @param config The node's configuration (ports, blackboard, etc.)
 */
  CheckKeyBoolValue(const std::string & name,
    const BT::NodeConfiguration & config);

/**
 * @brief Required BT ports: "key" (the blackboard key) and "value" (the expected value).
 */
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("robot_prefix",
	  "Prefix only for the HMI output, use \"hmi_\" for generic output"),
        BT::InputPort<std::string>("key", "Name of the blackboard key to check"),
        BT::InputPort<bool>("value", "Expected value"),
        BT::InputPort<bool>("hmi_message_logic",
	  "If true, outputs on hmi when check succeeds. If false, when it fails"),
      };
  }

  protected:
/**
 * @brief The main check. Returns SUCCESS if the blackboard's "key"
 *        equals the expected "value", otherwise FAILURE.
 */
  BT::NodeStatus tick() override;
};

/**
 * @class SetKeyBoolValue
 * @brief A node that sets a blackboard key to a given bool value.
 *
 * Usage example in XML:
 *   <SetKeyBoolValue name="SetKeyExample" key="some_key" value="foo"/>
 */
class SetKeyBoolValue : public BT::SyncActionNode
{
  public:
// Constructor
  SetKeyBoolValue(const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
  {
  }

// Required interface: which ports are needed/offered?
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("robot_prefix",
	  "Prefix only for the HMI output, use \"hmi_\" for generic output"),
        BT::InputPort<std::string>("key", "Blackboard key to set"),
        BT::InputPort<bool>("value", "Value to set (as a bool)")
      };
  }

// The main tick function; sets the blackboard key to the specified string
  BT::NodeStatus tick() override;
};

/**
 * @class WaitForKeyBool
 * @brief Periodically checks a blackboard key for a bool value = expected_value.
 *        If it matches => SUCCESS, if timeout is reached => FAILURE.
 *        Timeout=0 => infinite wait. Poll_rate => how often to re-check.
 *
 * Ports:
 *   - "key" (string) : blackboard key name
 *   - "expected_value" (bool) : the value we want
 *   - "timeout" (double) : seconds, 0 => infinite
 *   - "poll_rate" (double) : frequency in s
 */
class WaitForKeyBool : public BT::StatefulActionNode
{
  public:
  WaitForKeyBool(const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("key", "Blackboard key to read"),
        BT::InputPort<bool>("expected_value", "Desired bool value"),
        BT::InputPort<double>("timeout", 10.0, "Seconds before giving up (0 => infinite)"),
        BT::InputPort<double>("poll_rate", 0.25, "Check frequency (seconds)"),
        BT::InputPort<std::string>("prefix", "Prefix for HMI messages, optional")
      };
  }

  protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  private:
// read from ports:
  std::string key_;
  bool expected_value_;
  double timeout_;
  double poll_rate_;
  std::string prefix_;

// time management
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  rclcpp::Time next_check_time_;

  bool condition_met_;
};

// ---------------------------------------------------------------------------
// GetLinkPoseAction  (sync action – returns the current pose of a link)
// ---------------------------------------------------------------------------

/**
 * @brief Retrieve the current pose of a link (frame) relative to a reference
 *        frame using TF2.
 *
 *  INPUT PORTS
 *    - link_name        (string, *required*)  Source frame (e.g. "link_tcp")
 *    - reference_frame  (string, default="world")  Target frame
 *    - pose_key         (string, default="")  If non-empty, store pose to this
 *                                             blackboard key as well.
 *
 *  OUTPUT PORTS
 *    - pose             (geometry_msgs::msg::Pose)  Resulting pose
 */
class GetLinkPoseAction : public BT::SyncActionNode
{
  public:
  GetLinkPoseAction(const std::string & name,
    const BT::NodeConfiguration & cfg);

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("link_name", "Source link / frame"),
        BT::InputPort<std::string>("reference_frame", "", "Target frame (default world)"),
        BT::InputPort<std::vector<double> >("pre_transform_xyz_rpy", "6-tuple applied FIRST"),
        BT::InputPort<std::vector<double> >("post_transform_xyz_rpy",
	  "6-tuple applied AFTER link pose"),
        BT::InputPort<std::string>("pose_key", "", "If set, store pose in blackboard"),
        BT::OutputPort<geometry_msgs::msg::Pose>("pose", "Final pose")
      };
  }

  BT::NodeStatus tick() override;

  private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

/**
 * @brief Condition node that compares two poses stored on the blackboard.
 *        It succeeds if the Euclidean distance between them is below a
 *        specified tolerance, otherwise it fails.
 *
 * INPUT PORTS
 *   - reference_pose_key (string, *required*)  Blackboard key for the reference pose
 *   - target_pose_key  (string, *required*)  Blackboard key for the target pose
 *   - tolerance        (double, default=0.01) Distance tolerance in meters
 */
class CheckPoseDistance : public BT::ConditionNode
{
  public:
  CheckPoseDistance(const std::string & name, const BT::NodeConfiguration & cfg);

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("reference_pose_key", "Blackboard key for reference pose"),
        BT::InputPort<std::string>("target_pose_key", "Blackboard key for target pose"),
        BT::InputPort<double>("tolerance", 0.01, "Distance tolerance")
      };
  }

  BT::NodeStatus tick() override;

  private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Condition node that verifies a pose is within axis-aligned bounds.
 *
 * INPUT PORTS
 *   - pose_key (string, *required*)  Blackboard key for the pose to check
 *   - bounds   (vector<double>, size 6) [min_x, min_y, min_z, max_x, max_y, max_z]
 *   - inclusive  (bool, default=true)    Use inclusive comparisons
 */
class CheckPoseBounds : public BT::ConditionNode
{
  public:
  CheckPoseBounds(const std::string & name, const BT::NodeConfiguration & cfg);

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("pose_key", "Blackboard key for pose to check"),
        BT::InputPort<std::vector<double> >("bounds", "[min_x, min_y, min_z, max_x, max_y, max_z]"),
        BT::InputPort<bool>("inclusive", true, "Inclusive bounds check")
      };
  }

  BT::NodeStatus tick() override;

  private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Action node that copies a geometry_msgs::msg::Pose from one
 *        blackboard key to another.
 *
 * INPUT PORTS
 *   - source_key (string, required)  Blackboard key to read from
 *   - target_key (string, required)  Blackboard key to write to
 *   - robot_prefix (string, optional) Prefix for HMI messages (e.g., "R_" or "hmi_")
 */
class CopyPoseKey : public BT::SyncActionNode
{
  public:
  CopyPoseKey(const std::string & name, const BT::NodeConfiguration & cfg)
    : BT::SyncActionNode(name, cfg)
  {
  }

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("source_key", "Blackboard key (Pose) to read"),
        BT::InputPort<std::string>("target_key", "Blackboard key (Pose) to write"),
        BT::InputPort<std::string>("robot_prefix", "Prefix for HMI messages, optional")
      };
  }

  BT::NodeStatus tick() override;
};

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP
