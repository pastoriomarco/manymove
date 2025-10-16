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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <manymove_msgs/action/check_object_exists.hpp>
#include <manymove_msgs/action/add_collision_object.hpp>
#include <manymove_msgs/action/get_object_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "manymove_cpp_trees/action_nodes_objects.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"  // vector<string>/double parsing

using namespace std::chrono_literals;

// Simple fake server for CheckObjectExists
class FakeCheckObjectExistsServer
{
public:
  using CheckObjectExists = manymove_msgs::action::CheckObjectExists;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CheckObjectExists>;

  explicit FakeCheckObjectExistsServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    server_ = rclcpp_action::create_server<CheckObjectExists>(
      node_, "check_object_exists",
      std::bind(
        &FakeCheckObjectExistsServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeCheckObjectExistsServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeCheckObjectExistsServer::handle_accept, this, std::placeholders::_1));
  }

  void set_exists(bool e, bool attached = false, const std::string & link = {})
  {
    exists_.store(e);
    is_attached_.store(attached);
    link_name_ = link;
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const CheckObjectExists::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> &)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accept(const std::shared_ptr<GoalHandle> & gh)
  {
    auto result = std::make_shared<CheckObjectExists::Result>();
    result->exists = exists_.load();
    result->is_attached = is_attached_.load();
    result->link_name = link_name_;
    gh->succeed(result);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<CheckObjectExists>::SharedPtr server_;
  std::atomic<bool> exists_{false};
  std::atomic<bool> is_attached_{false};
  std::string link_name_;
};

// Fake server for AddCollisionObject
class FakeAddCollisionObjectServer
{
public:
  using Action = manymove_msgs::action::AddCollisionObject;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  explicit FakeAddCollisionObjectServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    server_ = rclcpp_action::create_server<Action>(
      node_, "add_collision_object",
      std::bind(
        &FakeAddCollisionObjectServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeAddCollisionObjectServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeAddCollisionObjectServer::handle_accept, this, std::placeholders::_1));
  }
  Action::Goal last_goal() const {return last_goal_;}

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
  {
    last_goal_ = *goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> &)
  {return rclcpp_action::CancelResponse::REJECT;}
  void handle_accept(const std::shared_ptr<GoalHandle> & gh)
  {
    auto result = std::make_shared<Action::Result>();
    result->success = true;
    result->message = "added";
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  Action::Goal last_goal_;
};

// Fake server for GetObjectPose
class FakeGetObjectPoseServer
{
public:
  using Action = manymove_msgs::action::GetObjectPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  explicit FakeGetObjectPoseServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    server_ = rclcpp_action::create_server<Action>(
      node_, "get_object_pose",
      std::bind(
        &FakeGetObjectPoseServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeGetObjectPoseServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeGetObjectPoseServer::handle_accept, this, std::placeholders::_1));
  }
  void set_pose(const geometry_msgs::msg::Pose & p)
  {
    pose_ = p;
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>)
  {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;}
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> &)
  {return rclcpp_action::CancelResponse::REJECT;}
  void handle_accept(const std::shared_ptr<GoalHandle> & gh)
  {
    auto result = std::make_shared<Action::Result>();
    result->success = true;
    result->message = "ok";
    result->pose = pose_;
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  geometry_msgs::msg::Pose pose_{};
};

class ObjectsNodesFixture : public ::testing::Test
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

TEST_F(ObjectsNodesFixture, CheckObjectExistsOutputsPorts)
{
  auto node = std::make_shared<rclcpp::Node>("objects_nodes_test");
  FakeCheckObjectExistsServer server(node);
  server.set_exists(true, true, "tool0");

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::CheckObjectExistsAction>("CheckObjectExistsAction");
  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <CheckObjectExistsAction name="check" object_id="obj" exists="{exists_out}" is_attached="{attached_out}" link_name="{link_out}"/>
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

  bool out_exists = false, out_attached = false; std::string out_link;
  ASSERT_TRUE(bb->get("exists_out", out_exists));
  ASSERT_TRUE(bb->get("attached_out", out_attached));
  ASSERT_TRUE(bb->get("link_out", out_link));
  EXPECT_TRUE(out_exists);
  EXPECT_TRUE(out_attached);
  EXPECT_EQ(out_link, "tool0");

  exec.cancel();
  th.join();
}

TEST_F(ObjectsNodesFixture, WaitForObjectActionPollsAndSucceeds)
{
  auto node = std::make_shared<rclcpp::Node>("objects_nodes_test_2");
  FakeCheckObjectExistsServer server(node);
  server.set_exists(false);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::WaitForObjectAction>("WaitForObjectAction");
  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <WaitForObjectAction name="wait" object_id="obj" exists="true" timeout="1.0"
                             poll_rate="0.05" prefix="hmi_"/>
      </BehaviorTree>
    </root>
  )";
  auto tree = factory.createTreeFromText(xml, bb);
  // first tick -> RUNNING
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  // flip server and tick until success
  server.set_exists(true, false, "");
  BT::NodeStatus st = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 20 && st == BT::NodeStatus::RUNNING; ++i) {
    std::this_thread::sleep_for(60ms);
    st = tree.tickRoot();
  }
  EXPECT_EQ(st, BT::NodeStatus::SUCCESS);

  exec.cancel();
  th.join();
}

TEST_F(ObjectsNodesFixture, AddCollisionObjectActionSendsGoal)
{
  auto node = std::make_shared<rclcpp::Node>("objects_nodes_test_3");
  FakeAddCollisionObjectServer server(node);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::AddCollisionObjectAction>(
    "AddCollisionObjectAction"
  );

  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <AddCollisionObjectAction name="add" object_id="box1" shape="box"
          dimensions="[0.1,0.2,0.3]"
          pose="position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}"/>
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

  auto g = server.last_goal();
  EXPECT_EQ(g.id, "box1");
  EXPECT_EQ(g.shape, "box");
  ASSERT_EQ(g.dimensions.size(), 3u);
  EXPECT_DOUBLE_EQ(g.dimensions[0], 0.1);
  EXPECT_DOUBLE_EQ(g.dimensions[1], 0.2);
  EXPECT_DOUBLE_EQ(g.dimensions[2], 0.3);

  exec.cancel();
  th.join();
}

TEST_F(ObjectsNodesFixture, GetObjectPoseActionWritesBlackboard)
{
  auto node = std::make_shared<rclcpp::Node>("objects_nodes_test_4");
  FakeGetObjectPoseServer server(node);
  geometry_msgs::msg::Pose p; p.position.x = 0.5; p.orientation.w = 1.0; server.set_pose(p);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::GetObjectPoseAction>("GetObjectPoseAction");
  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <GetObjectPoseAction name="get" object_id="obj"
                             pre_transform_xyz_rpy="[0,0,0,0,0,0]"
                             post_transform_xyz_rpy="[0,0,0,0,0,0]"
                             pose_key="obj_pose"/>
      </BehaviorTree>
    </root>
  )";
  auto tree = factory.createTreeFromText(xml, bb);
  BT::NodeStatus st2 = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 10 && st2 == BT::NodeStatus::RUNNING; ++i) {
    std::this_thread::sleep_for(50ms);
    st2 = tree.tickRoot();
  }
  EXPECT_EQ(st2, BT::NodeStatus::SUCCESS);

  geometry_msgs::msg::Pose pose_out;
  ASSERT_TRUE(bb->get("obj_pose", pose_out));
  EXPECT_NEAR(pose_out.position.x, 0.5, 1e-9);

  exec.cancel();
  th.join();
}
