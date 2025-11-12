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
#include <memory>
#include <string>
#include <thread>
#include <manymove_msgs/action/move_manipulator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "manymove_cpp_trees/tree_helper.hpp"

using namespace std::chrono_literals;

// Minimal fake MoveManipulator server
class FakeMoveManipulatorServer
{
public:
  using Action = manymove_msgs::action::MoveManipulator;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  explicit FakeMoveManipulatorServer(const rclcpp::Node::SharedPtr & node, const std::string & name)
  : node_(node)
  {
    server_ = rclcpp_action::create_server<Action>(
      node_, name,
      std::bind(
        &FakeMoveManipulatorServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeMoveManipulatorServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeMoveManipulatorServer::handle_accept, this, std::placeholders::_1));
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
    // Just echo an empty trajectory
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

class BTIntegrationFixture : public ::testing::Test
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

TEST_F(BTIntegrationFixture, BuildMoveXMLAndTickTree)
{
  auto node = std::make_shared<rclcpp::Node>("bt_integration_test");
  // Fake server with prefix R_
  FakeMoveManipulatorServer server(node, "R_move_manipulator");

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  BT::BehaviorTreeFactory factory;
  manymove_cpp_trees::registerAllNodeTypes(factory);

  auto bb = BT::Blackboard::create();
  bb->set("node", node);
  // Prepopulate required control flags
  bb->set("R_collision_detected", false);
  bb->set("R_stop_execution", false);
  bb->set("R_reset", false);

  // Build one move and corresponding XML
  manymove_msgs::msg::MovementConfig cfg;
  cfg.velocity_scaling_factor = 1.0;
  cfg.acceleration_scaling_factor = 1.0;
  cfg.max_cartesian_speed = 0.1;
  manymove_cpp_trees::Move m("R_", "tool0", "joint", cfg, "", {0.1, 0.2, 0.3});
  std::string seq_xml = manymove_cpp_trees::buildMoveXML("R_", "blockA", {m}, bb, false, 1);
  std::string xml = manymove_cpp_trees::mainTreeWrapperXML("Main", seq_xml);

  auto tree = factory.createTreeFromText(xml, bb);
  // Tick a few times until it stabilizes
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 10 && status == BT::NodeStatus::RUNNING; ++i) {
    std::this_thread::sleep_for(50ms);
    status = tree.tickRoot();
  }
  EXPECT_NE(status, BT::NodeStatus::FAILURE);

  exec.cancel();
  th.join();
}
