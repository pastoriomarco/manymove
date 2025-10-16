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
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "manymove_cpp_trees/action_nodes_logic.hpp"

using namespace std::chrono_literals;

class TFNodeFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {rclcpp::shutdown();}
  }
};

TEST_F(TFNodeFixture, GetLinkPoseWithPreTransform)
{
  auto node = std::make_shared<rclcpp::Node>("tf_node_test");
  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  // Publish static transform world -> link_tcp (identity rotation, translation 1,2,3)
  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = node->now();
  ts.header.frame_id = "world";
  ts.child_frame_id = "link_tcp";
  ts.transform.translation.x = 1.0;
  ts.transform.translation.y = 2.0;
  ts.transform.translation.z = 3.0;
  ts.transform.rotation.x = 0;
  ts.transform.rotation.y = 0;
  ts.transform.rotation.z = 0;
  ts.transform.rotation.w = 1;
  broadcaster.sendTransform(ts);

  // Allow TF buffer to get the transform
  std::this_thread::sleep_for(100ms);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::GetLinkPoseAction>("GetLinkPoseAction");

  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <GetLinkPoseAction name="get" link_name="link_tcp" reference_frame="world" pre_transform_xyz_rpy="[0.1,-0.1,0.2,0,0,0]" post_transform_xyz_rpy="[]" pose_key="tcp_pose"/>
      </BehaviorTree>
    </root>
  )";
  auto tree = factory.createTreeFromText(xml, bb);
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  geometry_msgs::msg::Pose out;
  ASSERT_TRUE(bb->get("tcp_pose", out));
  EXPECT_NEAR(out.position.x, 1.1, 1e-6);
  EXPECT_NEAR(out.position.y, 1.9, 1e-6);
  EXPECT_NEAR(out.position.z, 3.2, 1e-6);
  EXPECT_NEAR(out.orientation.w, 1.0, 1e-9);
}
