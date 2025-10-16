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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "manymove_cpp_trees/action_nodes_gripper.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"

using namespace std::chrono_literals;

// Fake GripperCommand server
class FakeGripperCommandServer
{
public:
  using Gripper = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Gripper>;
  explicit FakeGripperCommandServer(const rclcpp::Node::SharedPtr & node, const std::string & name)
  : node_(node)
  {
    server_ = rclcpp_action::create_server<Gripper>(
      node_, name,
      std::bind(
        &FakeGripperCommandServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeGripperCommandServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeGripperCommandServer::handle_accept, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Gripper::Goal>)
  {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;}
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> &)
  {return rclcpp_action::CancelResponse::REJECT;}
  void handle_accept(const std::shared_ptr<GoalHandle> & gh)
  {
    auto result = std::make_shared<Gripper::Result>();
    result->position = 0.5;
    result->effort = 0.0;
    result->stalled = false;  // Humble/Jazzy field name
    result->reached_goal = true;
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Gripper>::SharedPtr server_;
};

// Fake FollowJointTrajectory server
class FakeFJTServer
{
public:
  using FJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FJT>;
  explicit FakeFJTServer(const rclcpp::Node::SharedPtr & node, const std::string & name)
  : node_(node)
  {
    server_ = rclcpp_action::create_server<FJT>(
      node_, name,
      std::bind(
        &FakeFJTServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeFJTServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeFJTServer::handle_accept, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FJT::Goal>)
  {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;}
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> &)
  {return rclcpp_action::CancelResponse::REJECT;}
  void handle_accept(const std::shared_ptr<GoalHandle> & gh)
  {
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::SUCCESSFUL;
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<FJT>::SharedPtr server_;
};

class GripperNodesFixture : public ::testing::Test
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

TEST_F(GripperNodesFixture, GripperCommandActionSucceeds)
{
  auto node = std::make_shared<rclcpp::Node>("gripper_nodes_test");
  FakeGripperCommandServer srv(node, "gripper_command");
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::GripperCommandAction>("GripperCommandAction");

  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <GripperCommandAction name="gc" action_server="gripper_command" position="0.5" max_effort="10.0"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, bb);
  BT::NodeStatus st = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 10 && st == BT::NodeStatus::RUNNING; ++i) {
    std::this_thread::sleep_for(50ms);
    st = tree.tickRoot();
  }
  EXPECT_EQ(st, BT::NodeStatus::SUCCESS);

  exec.cancel();
  th.join();
}

TEST_F(GripperNodesFixture, GripperTrajActionSucceeds)
{
  auto node = std::make_shared<rclcpp::Node>("gripper_nodes_test2");
  FakeFJTServer srv(node, "gripper_fjt");
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::GripperTrajAction>("GripperTrajAction");

  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <GripperTrajAction name="gt" action_server="gripper_fjt" joint_names="[gripper_joint]" positions="[0.3]" time_from_start="0.1"/>
      </BehaviorTree>
    </root>
  )";
  auto tree = factory.createTreeFromText(xml, bb);
  BT::NodeStatus st = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 10 && st == BT::NodeStatus::RUNNING; ++i) {
    std::this_thread::sleep_for(50ms);
    st = tree.tickRoot();
  }
  EXPECT_EQ(st, BT::NodeStatus::SUCCESS);

  exec.cancel();
  th.join();
}

TEST_F(GripperNodesFixture, PublishJointStatePublishes)
{
  auto node = std::make_shared<rclcpp::Node>("gripper_nodes_test3");
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  sensor_msgs::msg::JointState last_msg;
  std::mutex m;
  std::condition_variable cv;
  bool got = false;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/test_joint_states", 10,
    [&](const sensor_msgs::msg::JointState & msg) {
      std::lock_guard<std::mutex> lk(m);
      last_msg = msg;
      got = true;
      cv.notify_one();
    });
  (void)sub;

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::PublishJointStateAction>("PublishJointStateAction");

  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <PublishJointStateAction name="pub" topic="/test_joint_states" joint_names="[j1,j2]" joint_positions="[1.0]" stamp_now="true"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  // wait for message
  std::unique_lock<std::mutex> lk(m);
  ASSERT_TRUE(cv.wait_for(lk, 300ms, [&]() {return got;}));
  ASSERT_EQ(last_msg.name.size(), 2u);
  EXPECT_EQ(last_msg.name[0], "j1");
  EXPECT_EQ(last_msg.name[1], "j2");
  ASSERT_EQ(last_msg.position.size(), 2u);
  EXPECT_DOUBLE_EQ(last_msg.position[0], 1.0);
  EXPECT_DOUBLE_EQ(last_msg.position[1], 1.0);

  exec.cancel();
  th.join();
}
