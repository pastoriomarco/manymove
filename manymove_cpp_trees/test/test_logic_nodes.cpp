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

#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>

#include <chrono>
#include <thread>


#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "manymove_cpp_trees/action_nodes_logic.hpp"

using namespace std::chrono_literals;

namespace
{
// A simple SyncActionNode that returns a programmed status
class DummyNode : public BT::SyncActionNode
{
public:
  explicit DummyNode(const std::string & name, const BT::NodeConfiguration & cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("return", std::string("SUCCESS"), "Desired return status")};
  }

  BT::NodeStatus tick() override
  {
    std::string desired;
    (void)getInput("return", desired);
    if (desired == "RUNNING") {
      return BT::NodeStatus::RUNNING;
    } else if (desired == "FAILURE") {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace

class LogicNodesFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(LogicNodesFixture, SetAndCheckKeyBool)
{
  auto bb = BT::Blackboard::create();
  auto node = std::make_shared<rclcpp::Node>("logic_test_node");
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::SetKeyBoolValue>("SetKeyBoolValue");
  factory.registerNodeType<manymove_cpp_trees::CheckKeyBoolValue>("CheckKeyBoolValue");

  // Set
  const std::string set_xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <SetKeyBoolValue name="set" robot_prefix="hmi_" key="flag" value="true"/>
      </BehaviorTree>
    </root>
  )";
  auto set_tree = factory.createTreeFromText(set_xml, bb);
  EXPECT_EQ(set_tree.tickRoot(), BT::NodeStatus::SUCCESS);

  // Check success
  const std::string chk_ok_xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <CheckKeyBoolValue name="chk" robot_prefix="hmi_" key="flag" value="true" hmi_message_logic="true"/>
      </BehaviorTree>
    </root>
  )";
  auto chk_ok_tree = factory.createTreeFromText(chk_ok_xml, bb);
  EXPECT_EQ(chk_ok_tree.tickRoot(), BT::NodeStatus::SUCCESS);

  // Check failure
  const std::string chk_fail_xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <CheckKeyBoolValue name="chk" robot_prefix="hmi_" key="flag" value="false" hmi_message_logic="false"/>
      </BehaviorTree>
    </root>
  )";
  auto chk_fail_tree = factory.createTreeFromText(chk_fail_xml, bb);
  EXPECT_EQ(chk_fail_tree.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(LogicNodesFixture, CopyPoseAndDistanceAndBounds)
{
  auto bb = BT::Blackboard::create();
  auto node = std::make_shared<rclcpp::Node>("logic_test_node2");
  bb->set("node", node);

  geometry_msgs::msg::Pose a;
  a.position.x = 1.0; a.position.y = 2.0; a.position.z = 3.0; a.orientation.w = 1.0;
  bb->set("pose_src", a);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::CopyPoseKey>("CopyPoseKey");
  factory.registerNodeType<manymove_cpp_trees::CheckPoseDistance>("CheckPoseDistance");
  factory.registerNodeType<manymove_cpp_trees::CheckPoseBounds>("CheckPoseBounds");

  // CopyPoseKey
  const std::string copy_xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <CopyPoseKey name="copy" source_key="pose_src" target_key="pose_dst" robot_prefix="hmi_"/>
      </BehaviorTree>
    </root>
  )";
  auto copy_tree = factory.createTreeFromText(copy_xml, bb);
  EXPECT_EQ(copy_tree.tickRoot(), BT::NodeStatus::SUCCESS);

  // CheckPoseDistance OK
  const std::string dist_ok_xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <CheckPoseDistance name="dist" reference_pose_key="pose_src" target_pose_key="pose_dst" tolerance="1e-6"/>
      </BehaviorTree>
    </root>
  )";
  auto dist_ok_tree = factory.createTreeFromText(dist_ok_xml, bb);
  EXPECT_EQ(dist_ok_tree.tickRoot(), BT::NodeStatus::SUCCESS);

  // Move target outside tolerance, re-check
  geometry_msgs::msg::Pose b = a; b.position.x += 0.1; bb->set("pose_dst", b);
  auto dist_fail_tree = factory.createTreeFromText(dist_ok_xml, bb);
  EXPECT_EQ(dist_fail_tree.tickRoot(), BT::NodeStatus::FAILURE);

  // CheckPoseBounds
  const std::string bounds_xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <CheckPoseBounds name="bounds" pose_key="pose_src" bounds="[0.0,0.0,0.0,2.0,3.0,4.0]" inclusive="true"/>
      </BehaviorTree>
    </root>
  )";
  auto bounds_tree = factory.createTreeFromText(bounds_xml, bb);
  EXPECT_EQ(bounds_tree.tickRoot(), BT::NodeStatus::SUCCESS);

  const std::string bounds_fail_xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <CheckPoseBounds name="bounds" pose_key="pose_src" bounds="[1.1,0.0,0.0,2.0,3.0,4.0]" inclusive="true"/>
      </BehaviorTree>
    </root>
  )";
  auto bounds_fail_tree = factory.createTreeFromText(bounds_fail_xml, bb);
  EXPECT_EQ(bounds_fail_tree.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(LogicNodesFixture, RetryPauseResetNodeBasic)
{
  // Build a small tree: RetryPauseResetNode -> Dummy
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::RetryPauseResetNode>("RetryPauseResetNode");
  factory.registerNodeType<DummyNode>("Dummy");

  const char * xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <RetryPauseResetNode stop_execution="{hmi_stop_execution}" collision_detected="{hmi_collision_detected}" reset="{hmi_reset}" robot_prefix="hmi_">
          <Dummy name="dummy" return="{dummy_return}"/>
        </RetryPauseResetNode>
      </BehaviorTree>
    </root>
  )";
  auto bb = BT::Blackboard::create();
  bb->set("hmi_stop_execution", false);
  bb->set("hmi_collision_detected", false);
  bb->set("hmi_reset", false);
  bb->set("dummy_return", std::string("SUCCESS"));
  auto tree = factory.createTreeFromText(xml, bb);

  // Program dummy via blackboard
  bb->set("dummy_return", std::string("FAILURE"));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);

  // stop_execution pauses => RUNNING
  bb->set("hmi_stop_execution", true);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  bb->set("hmi_stop_execution", false);

  // reset causes FAILURE
  bb->set("hmi_reset", true);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  bb->set("hmi_reset", false);

  // Now child succeeds => SUCCESS
  bb->set("dummy_return", std::string("SUCCESS"));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}
