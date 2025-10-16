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

#include <manymove_msgs/action/get_input.hpp>
#include <manymove_msgs/action/set_output.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "manymove_cpp_trees/action_nodes_signals.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"

using namespace std::chrono_literals;

class FakeSetOutputServer
{
public:
  using SetOutput = manymove_msgs::action::SetOutput;
  using GoalHandle = rclcpp_action::ServerGoalHandle<SetOutput>;
  explicit FakeSetOutputServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    server_ = rclcpp_action::create_server<SetOutput>(
      node_, "set_output",
      std::bind(
        &FakeSetOutputServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeSetOutputServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeSetOutputServer::handle_accept, this, std::placeholders::_1));
  }
  void set_success(bool v) {success_.store(v);}  // respond with success/failure

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const SetOutput::Goal>)
  {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;}
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> &)
  {return rclcpp_action::CancelResponse::REJECT;}
  void handle_accept(const std::shared_ptr<GoalHandle> & gh)
  {
    auto result = std::make_shared<SetOutput::Result>();
    result->success = success_.load();
    result->message = result->success ? "OK" : "FAIL";
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<SetOutput>::SharedPtr server_;
  std::atomic<bool> success_{true};
};

class FakeGetInputServer
{
public:
  using GetInput = manymove_msgs::action::GetInput;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GetInput>;
  explicit FakeGetInputServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    server_ = rclcpp_action::create_server<GetInput>(
      node_, "get_input",
      std::bind(
        &FakeGetInputServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeGetInputServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeGetInputServer::handle_accept, this, std::placeholders::_1));
  }
  void set_value(bool ok, int v) {ok_.store(ok); value_.store(v);}  // change next result

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GetInput::Goal>)
  {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;}
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> &)
  {return rclcpp_action::CancelResponse::REJECT;}
  void handle_accept(const std::shared_ptr<GoalHandle> & gh)
  {
    auto result = std::make_shared<GetInput::Result>();
    result->success = ok_.load();
    result->value = static_cast<int8_t>(value_.load());
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<GetInput>::SharedPtr server_;
  std::atomic<bool> ok_{true};
  std::atomic<int> value_{0};
};

class SignalsNodesFixture : public ::testing::Test
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

TEST_F(SignalsNodesFixture, SetOutputActionSuccess)
{
  auto node = std::make_shared<rclcpp::Node>("signals_nodes_test");
  FakeSetOutputServer srv(node);
  srv.set_success(true);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::SetOutputAction>("SetOutputAction");
  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <SetOutputAction name="set" io_type="tool" ionum="3" value="1" robot_prefix=""/>
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

TEST_F(SignalsNodesFixture, WaitForInputActionPollAndTimeout)
{
  auto node = std::make_shared<rclcpp::Node>("signals_nodes_test2");
  FakeGetInputServer srv(node);
  srv.set_value(true, 0);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread th([&]() {exec.spin();});

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<manymove_cpp_trees::WaitForInputAction>("WaitForInputAction");
  const std::string xml =
    R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <WaitForInputAction name="wait" io_type="controller" ionum="1" desired_value="1" timeout="0.25" poll_rate="0.05" robot_prefix="hmi_"/>
      </BehaviorTree>
    </root>
  )";
  auto tree = factory.createTreeFromText(xml, bb);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 20 && status == BT::NodeStatus::RUNNING; ++i) {
    std::this_thread::sleep_for(60ms);
    status = tree.tickRoot();
  }
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  exec.cancel();
  th.join();
}
