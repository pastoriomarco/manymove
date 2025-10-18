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

#include <cstdlib>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <action_msgs/srv/cancel_goal.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <manymove_msgs/action/load_traj_controller.hpp>
#include <manymove_msgs/action/move_manipulator.hpp>
#include <manymove_msgs/action/unload_traj_controller.hpp>
#include <manymove_msgs/msg/move_manipulator_goal.hpp>
#include <manymove_msgs/msg/movement_config.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "manymove_planner/action_server.hpp"
#include "manymove_planner/planner_interface.hpp"
namespace robot_trajectory
{
class RobotTrajectory;
}  // namespace robot_trajectory

namespace
{
const bool kConfigureRmwEnv = []() {
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    setenv("RCUTILS_LOGGING_USE_STDOUT", "1", 1);
    return true;
  }();
}  // namespace

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using LoadTrajController = manymove_msgs::action::LoadTrajController;
using MoveManipulator = manymove_msgs::action::MoveManipulator;
using UnloadTrajController = manymove_msgs::action::UnloadTrajController;

namespace
{

constexpr char kFollowJointTrajectoryActionName[] = "/fake_traj_controller/follow_joint_trajectory";

template<typename FutureT>
void wait_for_future_ready(const FutureT & future, const std::chrono::seconds timeout)
{
  ASSERT_EQ(future.wait_for(timeout), std::future_status::ready);
}

template<typename PredicateT>
bool wait_until(
  PredicateT && predicate,
  const std::chrono::milliseconds timeout,
  const std::chrono::milliseconds poll_interval = std::chrono::milliseconds{50})
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(poll_interval);
  }
  return predicate();
}

moveit_msgs::msg::RobotTrajectory makeTrajectory(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & start_positions,
  const std::vector<double> & end_positions,
  double duration_seconds = 1.0)
{
  moveit_msgs::msg::RobotTrajectory traj;
  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint start_pt;
  start_pt.positions = start_positions;
  start_pt.time_from_start.sec = 0;
  start_pt.time_from_start.nanosec = 0;

  trajectory_msgs::msg::JointTrajectoryPoint end_pt;
  end_pt.positions = end_positions;
  end_pt.time_from_start.sec = static_cast<int32_t>(duration_seconds);
  end_pt.time_from_start.nanosec =
    static_cast<uint32_t>((duration_seconds - static_cast<int32_t>(duration_seconds)) * 1e9);

  jt.points.push_back(start_pt);
  jt.points.push_back(end_pt);

  traj.joint_trajectory = jt;
  return traj;
}

manymove_msgs::action::MoveManipulator::Goal makeMoveGoal(
  const moveit_msgs::msg::RobotTrajectory & traj)
{
  manymove_msgs::action::MoveManipulator::Goal goal;
  goal.existing_trajectory = traj;
  goal.plan_request.movement_type = "joint";
  goal.plan_request.joint_values = traj.joint_trajectory.points.back().positions;
  goal.plan_request.config.rotational_precision = 0.1;
  goal.plan_request.config.linear_precision = 0.01;
  goal.plan_request.config.velocity_scaling_factor = 1.0;
  goal.plan_request.config.acceleration_scaling_factor = 1.0;
  goal.plan_request.config.max_cartesian_speed = 2.0;
  goal.plan_request.config.deceleration_time = 0.5;
  goal.plan_request.config.min_stop_time = 0.1;
  goal.plan_request.config.tcp_frame = "tool0";
  goal.plan_request.config.plan_number_limit = 5;
  goal.plan_request.config.plan_number_target = 1;
  return goal;
}

class ControllerManagerMock
{
public:
  ControllerManagerMock()
  : node_(std::make_shared<rclcpp::Node>("controller_manager_mock"))
  {
    unload_service_ = node_->create_service<controller_manager_msgs::srv::UnloadController>(
      "/controller_manager/unload_controller",
      [this](
        const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
        std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_unload_name_ = request->name;
        response->ok = unload_ok_;
      });

    load_service_ = node_->create_service<controller_manager_msgs::srv::LoadController>(
      "/controller_manager/load_controller",
      [this](
        const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
        std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_load_name_ = request->name;
        response->ok = load_ok_;
      });

    switch_service_ = node_->create_service<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller",
      [this](
        const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
        std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!request->activate_controllers.empty()) {
          last_activate_name_ = request->activate_controllers.front();
          response->ok = activate_ok_;
        } else if (!request->deactivate_controllers.empty()) {
          last_deactivate_name_ = request->deactivate_controllers.front();
          response->ok = deactivate_ok_;
        } else {
          response->ok = true;
        }
      });

    configure_service_ = node_->create_service<controller_manager_msgs::srv::ConfigureController>(
      "/controller_manager/configure_controller",
      [this](
        const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
        std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_configure_name_ = request->name;
        response->ok = configure_ok_;
      });
  }

  rclcpp::Node::SharedPtr node() const {return node_;}

  void set_load_ok(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    load_ok_ = value;
  }

  void set_configure_ok(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    configure_ok_ = value;
  }

  void set_activate_ok(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    activate_ok_ = value;
  }

  void set_deactivate_ok(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    deactivate_ok_ = value;
  }

  void set_unload_ok(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    unload_ok_ = value;
  }

  std::string last_loaded() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_load_name_;
  }

  std::string last_unloaded() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_unload_name_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadController>::SharedPtr load_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_service_;

  mutable std::mutex mutex_;
  bool load_ok_{true};
  bool configure_ok_{true};
  bool activate_ok_{true};
  bool deactivate_ok_{true};
  bool unload_ok_{true};
  std::string last_load_name_;
  std::string last_unload_name_;
  std::string last_activate_name_;
  std::string last_deactivate_name_;
  std::string last_configure_name_;
};

class FakeFollowJointTrajectoryServer
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  explicit FakeFollowJointTrajectoryServer(
    const rclcpp::Node::SharedPtr & node,
    std::chrono::milliseconds feedback_period = std::chrono::milliseconds(20))
  : node_(node),
    feedback_period_(feedback_period)
  {
    server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      node_, kFollowJointTrajectoryActionName,
      std::bind(
        &FakeFollowJointTrajectoryServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&FakeFollowJointTrajectoryServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeFollowJointTrajectoryServer::handle_accept, this, std::placeholders::_1));
  }

  ~FakeFollowJointTrajectoryServer()
  {
    shutdown();
  }

  void shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_requested_ = true;
    }
    goal_ready_cv_.notify_all();
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    workers_.clear();
  }

  void set_success(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    succeed_ = value;
  }

  void set_hold_until_cancel(bool hold)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_until_cancel_ = hold;
  }

  bool wait_for_active_goal(const std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return goal_ready_cv_.wait_for(lock, timeout, [this]() {return goal_active_;});
  }

  const std::string & action_name() const
  {
    return action_name_;
  }

  bool cancel_requested() const
  {
    return cancel_requested_.load();
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accept(const std::shared_ptr<GoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_active_ = true;
    }
    goal_ready_cv_.notify_all();

    workers_.emplace_back(
      [this, goal_handle]() {
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        const auto goal_traj = goal_handle->get_goal()->trajectory;
        size_t iteration = 0;
        while (rclcpp::ok()) {
          if (shutdown_requested_) {
            return;
          }
          if (cancel_requested_.load()) {
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            goal_handle->canceled(result);
            return;
          }

          if (!hold_until_cancel_ && iteration >= max_feedback_cycles_) {
            break;
          }

          feedback->actual.time_from_start.sec = static_cast<int32_t>(iteration + 1);
          feedback->actual.time_from_start.nanosec = 0;
          if (!goal_traj.points.empty()) {
            const size_t sample_index = std::min(iteration, goal_traj.points.size() - 1);
            feedback->actual.positions = goal_traj.points[sample_index].positions;
          } else {
            feedback->actual.positions.clear();
          }
          goal_handle->publish_feedback(feedback);
          ++iteration;
          std::this_thread::sleep_for(feedback_period_);
        }

        if (cancel_requested_.load()) {
          auto result = std::make_shared<FollowJointTrajectory::Result>();
          result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
          goal_handle->canceled(result);
          return;
        }

        auto result = std::make_shared<FollowJointTrajectory::Result>();
        if (succeed_) {
          result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
          goal_handle->succeed(result);
        } else {
          result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
          result->error_string = "Forced failure";
          goal_handle->abort(result);
        }
      });
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
  std::chrono::milliseconds feedback_period_;

  std::atomic<bool> cancel_requested_{false};
  bool succeed_{true};
  bool hold_until_cancel_{false};
  bool shutdown_requested_{false};
  bool goal_active_{false};
  const std::string action_name_{kFollowJointTrajectoryActionName};
  const size_t max_feedback_cycles_{5};

  std::mutex mutex_;
  std::condition_variable goal_ready_cv_;
  std::vector<std::thread> workers_;
};

class FakePlanner : public PlannerInterface
{
public:
  explicit FakePlanner(
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr fjt_client)
  : fjt_client_(std::move(fjt_client))
  {}

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_calls_ = 0;
    controlled_stop_calls_ = 0;
    controlled_stop_result_ = PlannerInterface::ControlledStopResult::STOP_SENT;
    start_valid_responses_ = std::queue<bool>();
    default_start_valid_ = true;
    end_valid_ = true;
    plan_success_ = true;
    planned_traj_ = moveit_msgs::msg::RobotTrajectory();
    trajectory_valid_responses_ = std::queue<bool>();
    default_trajectory_valid_response_ = true;
  }

  void set_plan_result(bool success, const moveit_msgs::msg::RobotTrajectory & traj)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_success_ = success;
    planned_traj_ = traj;
  }

  void set_start_valid(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    default_start_valid_ = value;
    start_valid_responses_ = std::queue<bool>();
  }

  void set_start_valid_responses(
    const std::vector<bool> & responses, bool default_value = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    start_valid_responses_ = std::queue<bool>();
    for (bool response : responses) {
      start_valid_responses_.push(response);
    }
    default_start_valid_ = default_value;
  }

  void set_end_valid(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    end_valid_ = value;
  }

  void set_trajectory_valid_responses(
    const std::vector<bool> & responses,
    bool default_response = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory_valid_responses_ = std::queue<bool>();
    for (bool value : responses) {
      trajectory_valid_responses_.push(value);
    }
    default_trajectory_valid_response_ = default_response;
  }

  size_t plan_call_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return plan_calls_;
  }

  size_t controlled_stop_call_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return controlled_stop_calls_;
  }

  std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(
    const manymove_msgs::action::PlanManipulator::Goal &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++plan_calls_;
    return {plan_success_, planned_traj_};
  }

  std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParameterization(
    const moveit_msgs::msg::RobotTrajectory & input_traj,
    const manymove_msgs::msg::MovementConfig &) override
  {
    return {true, input_traj};
  }

  PlannerInterface::ControlledStopResult sendControlledStop(
    const manymove_msgs::msg::MovementConfig &,
    const moveit_msgs::msg::RobotTrajectory &, double) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++controlled_stop_calls_;
    return controlled_stop_result_;
  }

  void set_controlled_stop_result(PlannerInterface::ControlledStopResult result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    controlled_stop_result_ = result;
  }

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
  getFollowJointTrajClient() const override
  {
    return fjt_client_;
  }

  bool isJointStateValid(const std::vector<double> &) const override {return true;}

  bool isTrajectoryStartValid(
    const moveit_msgs::msg::RobotTrajectory &,
    const manymove_msgs::msg::MoveManipulatorGoal &,
    const std::vector<double> &) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!start_valid_responses_.empty()) {
      bool value = start_valid_responses_.front();
      start_valid_responses_.pop();
      return value;
    }
    return default_start_valid_;
  }

  bool isTrajectoryEndValid(
    const moveit_msgs::msg::RobotTrajectory &,
    const manymove_msgs::msg::MoveManipulatorGoal &) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return end_valid_;
  }

  bool isTrajectoryValid(
    const trajectory_msgs::msg::JointTrajectory &,
    const moveit_msgs::msg::Constraints &, const double) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!trajectory_valid_responses_.empty()) {
      bool value = trajectory_valid_responses_.front();
      trajectory_valid_responses_.pop();
      return value;
    }
    return default_trajectory_valid_response_;
  }

  bool isTrajectoryValid(
    const robot_trajectory::RobotTrajectory &,
    const moveit_msgs::msg::Constraints &, const double) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!trajectory_valid_responses_.empty()) {
      bool value = trajectory_valid_responses_.front();
      trajectory_valid_responses_.pop();
      return value;
    }
    return default_trajectory_valid_response_;
  }

private:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr fjt_client_;
  mutable std::mutex mutex_;
  mutable size_t plan_calls_{0};
  mutable size_t controlled_stop_calls_{0};
  PlannerInterface::ControlledStopResult controlled_stop_result_{
    PlannerInterface::ControlledStopResult::STOP_SENT};
  mutable std::queue<bool> start_valid_responses_;
  bool default_start_valid_{true};
  bool end_valid_{true};
  bool plan_success_{true};
  moveit_msgs::msg::RobotTrajectory planned_traj_;
  mutable std::queue<bool> trajectory_valid_responses_;
  bool default_trajectory_valid_response_{true};
};

class ManipulatorActionServerFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    namespace fs = std::filesystem;
    const fs::path ros_home = fs::temp_directory_path() / "manymove_planner_tests_ros";
    fs::create_directories(ros_home);
    const std::string ros_home_str = ros_home.string();
    setenv("ROS_HOME", ros_home_str.c_str(), 1);
    rcl_ready_ = false;
    init_error_.clear();
    // Default: enable ROS-backed tests.
    // Opt-out with MANYMOVE_PLANNER_DISABLE_RCL_TESTS=1|true, or
    // legacy opt-in var MANYMOVE_PLANNER_ENABLE_RCL_TESTS=0|false.
    const char * disable_env = std::getenv("MANYMOVE_PLANNER_DISABLE_RCL_TESTS");
    const char * enable_env = std::getenv("MANYMOVE_PLANNER_ENABLE_RCL_TESTS");
    auto is_truthy = [](const char * v) {
        if (!v) {
          return false;
        }
        std::string s(v);
        for (auto & c : s) {
          c = static_cast<char>(std::tolower(c));
        }
        return s == "1" || s == "true" || s == "yes";
      };
    auto is_falsy = [](const char * v) {
        if (!v) {
          return false;
        }
        std::string s(v);
        for (auto & c : s) {
          c = static_cast<char>(std::tolower(c));
        }
        return s == "0" || s == "false" || s == "no";
      };

    if (is_truthy(disable_env) || is_falsy(enable_env)) {
      rcl_ready_ = false;
      init_error_ = "ROS-backed tests disabled by environment";
      return;
    }
    const char * current_rmw = std::getenv("RMW_IMPLEMENTATION");
    std::vector<std::string> rmw_candidates;
    auto append_unique = [&rmw_candidates](const std::string & rmw) {
        if (std::find(rmw_candidates.begin(), rmw_candidates.end(), rmw) == rmw_candidates.end()) {
          rmw_candidates.push_back(rmw);
        }
      };
    if (current_rmw && current_rmw[0] != '\0') {
      append_unique(current_rmw);
    } else {
      append_unique("rmw_fastrtps_cpp");
    }
    append_unique("rmw_fastrtps_cpp");
    append_unique("rmw_cyclonedds_cpp");

    std::string errors;
    for (const auto & rmw : rmw_candidates) {
      setenv("RMW_IMPLEMENTATION", rmw.c_str(), 1);
      setenv("RCL_ASSERT_RMW_ID_MATCHES", rmw.c_str(), 1);
      try {
        rclcpp::init(0, nullptr);
        auto probe = std::make_shared<rclcpp::Node>("manymove_planner_test_probe");
        (void)probe;
        rcl_ready_ = true;
        break;
      } catch (const std::exception & e) {
        if (!errors.empty()) {
          errors.append("; ");
        }
        errors.append(rmw + std::string(": ") + e.what());
        if (rclcpp::ok()) {
          rclcpp::shutdown();
        }
      }
    }

    if (!rcl_ready_) {
      init_error_ = errors.empty() ? "Failed to initialize rclcpp" : errors;
      return;
    }
  }

  static void TearDownTestSuite()
  {
    if (rcl_ready_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    if (!rcl_ready_) {
      GTEST_SKIP() << "Skipping ROS-dependent tests: " << init_error_;
      return;
    }
    using namespace std::chrono_literals;

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    controller_manager_ = std::make_shared<ControllerManagerMock>();
    executor_->add_node(controller_manager_->node());

    action_node_ = std::make_shared<rclcpp::Node>("manipulator_action_server_under_test");
    executor_->add_node(action_node_);

    fjt_server_node_ = std::make_shared<rclcpp::Node>("fake_follow_joint_server");
    executor_->add_node(fjt_server_node_);

    planner_node_ = std::make_shared<rclcpp::Node>("fake_planner_node");
    executor_->add_node(planner_node_);

    client_node_ = std::make_shared<rclcpp::Node>("manipulator_action_client_node");
    executor_->add_node(client_node_);

    executor_thread_ = std::thread([this]() {executor_->spin();});

    fake_fjt_server_ = std::make_shared<FakeFollowJointTrajectoryServer>(fjt_server_node_);

    fjt_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      planner_node_, fake_fjt_server_->action_name());
    ASSERT_TRUE(fjt_client_->wait_for_action_server(2s));

    fake_planner_ = std::make_shared<FakePlanner>(fjt_client_);
    fake_planner_->reset();

    action_server_ = std::make_shared<ManipulatorActionServer>(action_node_, fake_planner_, "");

    move_client_ =
      rclcpp_action::create_client<MoveManipulator>(client_node_, "/move_manipulator");
    ASSERT_TRUE(move_client_->wait_for_action_server(2s));

    load_client_ = rclcpp_action::create_client<LoadTrajController>(
      client_node_, "/load_trajectory_controller");
    ASSERT_TRUE(load_client_->wait_for_action_server(2s));

    unload_client_ = rclcpp_action::create_client<UnloadTrajController>(
      client_node_, "/unload_trajectory_controller");
    ASSERT_TRUE(unload_client_->wait_for_action_server(2s));

    joint_state_pub_ = client_node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS());
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    if (fake_fjt_server_) {
      fake_fjt_server_->shutdown();
    }
    joint_state_pub_.reset();
    move_client_.reset();
    load_client_.reset();
    unload_client_.reset();
    action_server_.reset();
    fake_planner_.reset();
    fjt_client_.reset();
    fake_fjt_server_.reset();
    client_node_.reset();
    planner_node_.reset();
    fjt_server_node_.reset();
    action_node_.reset();
    controller_manager_.reset();
    executor_.reset();
  }

  void publish_joint_state(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions) const
  {
    sensor_msgs::msg::JointState msg;
    msg.name = joint_names;
    msg.position = positions;
    joint_state_pub_->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<ControllerManagerMock> controller_manager_;
  std::shared_ptr<FakeFollowJointTrajectoryServer> fake_fjt_server_;
  std::shared_ptr<FakePlanner> fake_planner_;
  std::shared_ptr<ManipulatorActionServer> action_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr fjt_client_;
  rclcpp::Node::SharedPtr action_node_;
  rclcpp::Node::SharedPtr fjt_server_node_;
  rclcpp::Node::SharedPtr planner_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::thread executor_thread_;
  rclcpp_action::Client<MoveManipulator>::SharedPtr move_client_;
  rclcpp_action::Client<LoadTrajController>::SharedPtr load_client_;
  rclcpp_action::Client<UnloadTrajController>::SharedPtr unload_client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  static inline bool rcl_ready_{false};
  static inline std::string init_error_{};
};

TEST_F(ManipulatorActionServerFixture, ExecutesProvidedTrajectoryWhenValid)
{
  using namespace std::chrono_literals;

  const std::vector<std::string> joints{"joint1", "joint2"};
  const std::vector<double> start{0.0, 0.1};
  const std::vector<double> target{0.5, 0.2};

  auto existing_traj = makeTrajectory(joints, start, target);

  fake_planner_->reset();
  fake_planner_->set_start_valid(true);
  fake_planner_->set_end_valid(true);
  fake_planner_->set_plan_result(true, existing_traj);
  fake_planner_->set_trajectory_valid_responses({true, true}, true);
  fake_fjt_server_->set_success(true);
  fake_fjt_server_->set_hold_until_cancel(false);

  publish_joint_state(joints, start);

  auto goal = makeMoveGoal(existing_traj);
  auto send_future = move_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  // Ensure the fake FollowJointTrajectory server has accepted the goal
  // before we start waiting on the result. This avoids a rare race on
  // Humble where the result is observed too early.
  ASSERT_TRUE(fake_fjt_server_->wait_for_active_goal(200ms));

  auto result_future = move_client_->async_get_result(goal_handle);
  // Give a bit more time on Humble to avoid flakiness.
  wait_for_future_ready(result_future, 5s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(fake_planner_->plan_call_count(), 0u);
  EXPECT_EQ(fake_planner_->controlled_stop_call_count(), 0u);
}

TEST_F(ManipulatorActionServerFixture, FallsBackToPlanningWhenExistingTrajectoryRejects)
{
  using namespace std::chrono_literals;

  const std::vector<std::string> joints{"joint_a", "joint_b", "joint_c"};
  const std::vector<double> start{0.0, 0.0, 0.0};
  const std::vector<double> plan_target{0.4, 0.2, -0.1};

  auto existing_traj = makeTrajectory(joints, start, plan_target);
  auto planned_traj = makeTrajectory(joints, start, plan_target);

  fake_planner_->reset();
  fake_planner_->set_start_valid_responses({false, true}, true);
  fake_planner_->set_end_valid(true);
  fake_planner_->set_plan_result(true, planned_traj);
  fake_planner_->set_trajectory_valid_responses({true, true}, true);
  fake_fjt_server_->set_success(true);

  publish_joint_state(joints, start);

  auto goal = makeMoveGoal(existing_traj);
  auto send_future = move_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = move_client_->async_get_result(goal_handle);
  wait_for_future_ready(result_future, 3s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(fake_planner_->plan_call_count(), 1u);
  EXPECT_EQ(fake_planner_->controlled_stop_call_count(), 0u);
  EXPECT_EQ(
    wrapped_result.result->final_trajectory.joint_trajectory.points.size(),
    planned_traj.joint_trajectory.points.size());
}

TEST_F(ManipulatorActionServerFixture, PlanningFailureAbortsMoveGoal)
{
  using namespace std::chrono_literals;

  const std::vector<std::string> joints{"joint1", "joint2"};
  const std::vector<double> start{0.0, 0.0};
  const std::vector<double> end{0.5, 0.5};

  auto existing_traj = makeTrajectory(joints, start, end);

  fake_planner_->reset();
  fake_planner_->set_start_valid(false);
  fake_planner_->set_plan_result(false, moveit_msgs::msg::RobotTrajectory());
  fake_planner_->set_trajectory_valid_responses({true}, true);

  publish_joint_state(joints, start);

  auto goal = makeMoveGoal(existing_traj);
  auto send_future = move_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = move_client_->async_get_result(goal_handle);
  wait_for_future_ready(result_future, 2s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_EQ(fake_planner_->plan_call_count(), 1u);
}

TEST_F(ManipulatorActionServerFixture, CollisionDetectedDuringExecutionAborts)
{
  using namespace std::chrono_literals;

  const std::vector<std::string> joints{"joint1", "joint2", "joint3"};
  const std::vector<double> start{0.1, -0.2, 0.0};
  const std::vector<double> end{0.5, 0.1, 0.2};

  auto existing_traj = makeTrajectory(joints, start, end);

  fake_planner_->reset();
  fake_planner_->set_start_valid(true);
  fake_planner_->set_end_valid(true);
  fake_planner_->set_plan_result(true, existing_traj);
  // Let pre-execution validations pass once and surface the collision during feedback.
  fake_planner_->set_trajectory_valid_responses({true, true, false}, false);
  fake_fjt_server_->set_success(true);

  publish_joint_state(joints, start);

  auto goal = makeMoveGoal(existing_traj);
  auto send_future = move_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  // Ensure the fake FollowJointTrajectory server has accepted the goal before we observe results.
  ASSERT_TRUE(fake_fjt_server_->wait_for_active_goal(200ms));

  auto result_future = move_client_->async_get_result(goal_handle);
  wait_for_future_ready(result_future, 5s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_NE(
    wrapped_result.result->message.find("collision"),
    std::string::npos);
}

TEST_F(ManipulatorActionServerFixture, CancelDuringExecutionTriggersControlledStop)
{
  using namespace std::chrono_literals;

  const std::vector<std::string> joints{"joint_x", "joint_y"};
  const std::vector<double> start{0.0, 0.0};
  const std::vector<double> end{0.5, -0.1};

  auto existing_traj = makeTrajectory(joints, start, end);

  fake_planner_->reset();
  fake_planner_->set_start_valid(true);
  fake_planner_->set_end_valid(true);
  fake_planner_->set_plan_result(true, existing_traj);
  fake_planner_->set_trajectory_valid_responses({true, true, true}, true);

  fake_fjt_server_->set_success(true);
  fake_fjt_server_->set_hold_until_cancel(true);

  publish_joint_state(joints, start);

  auto goal = makeMoveGoal(existing_traj);
  auto send_future = move_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(fake_fjt_server_->wait_for_active_goal(200ms));

  auto cancel_future = move_client_->async_cancel_goal(goal_handle);
  wait_for_future_ready(cancel_future, 4s);
  auto cancel_response = cancel_future.get();
  ASSERT_NE(cancel_response, nullptr);
  EXPECT_EQ(
    cancel_response->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);

  auto result_future = move_client_->async_get_result(goal_handle);
  wait_for_future_ready(result_future, 6s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(fake_planner_->controlled_stop_call_count(), 1u);
  EXPECT_TRUE(fake_fjt_server_->cancel_requested());
}

TEST_F(ManipulatorActionServerFixture, CancelDuringExecutionNearCompletionFinishesNaturally)
{
  using namespace std::chrono_literals;

  const std::vector<std::string> joints{"joint_x", "joint_y"};
  const std::vector<double> start{0.0, 0.0};
  const std::vector<double> end{0.5, -0.1};

  auto existing_traj = makeTrajectory(joints, start, end);

  fake_planner_->reset();
  fake_planner_->set_start_valid(true);
  fake_planner_->set_end_valid(true);
  fake_planner_->set_plan_result(true, existing_traj);
  fake_planner_->set_trajectory_valid_responses({true, true, true}, true);
  fake_planner_->set_controlled_stop_result(
    PlannerInterface::ControlledStopResult::NOT_REQUIRED);

  fake_fjt_server_->set_success(true);
  fake_fjt_server_->set_hold_until_cancel(false);

  publish_joint_state(joints, start);

  auto goal = makeMoveGoal(existing_traj);
  auto send_future = move_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(fake_fjt_server_->wait_for_active_goal(200ms));

  auto cancel_future = move_client_->async_cancel_goal(goal_handle);
  wait_for_future_ready(cancel_future, 4s);
  auto cancel_response = cancel_future.get();
  ASSERT_NE(cancel_response, nullptr);
  EXPECT_EQ(
    cancel_response->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);

  auto result_future = move_client_->async_get_result(goal_handle);
  wait_for_future_ready(result_future, 6s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(fake_planner_->controlled_stop_call_count(), 1u);
  EXPECT_FALSE(fake_fjt_server_->cancel_requested());
}

TEST_F(ManipulatorActionServerFixture, LoadControllerActionSucceeds)
{
  using namespace std::chrono_literals;

  controller_manager_->set_load_ok(true);
  controller_manager_->set_configure_ok(true);
  controller_manager_->set_activate_ok(true);

  LoadTrajController::Goal goal;
  goal.controller_name = "test_controller";

  auto send_future = load_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = load_client_->async_get_result(goal_handle);
  wait_for_future_ready(result_future, 2s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(controller_manager_->last_loaded(), "test_controller");
}

TEST_F(ManipulatorActionServerFixture, LoadControllerActionFailsWhenActivationFails)
{
  using namespace std::chrono_literals;

  controller_manager_->set_load_ok(true);
  controller_manager_->set_configure_ok(true);
  controller_manager_->set_activate_ok(false);

  LoadTrajController::Goal goal;
  goal.controller_name = "faulty_controller";

  auto send_future = load_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = load_client_->async_get_result(goal_handle);
  wait_for_future_ready(result_future, 2s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_NE(
    wrapped_result.result->message.find("Activate error"),
    std::string::npos);
}

TEST_F(ManipulatorActionServerFixture, UnloadControllerActionSucceeds)
{
  using namespace std::chrono_literals;

  controller_manager_->set_deactivate_ok(true);
  controller_manager_->set_unload_ok(true);

  UnloadTrajController::Goal goal;
  goal.controller_name = "controller_to_unload";

  auto send_future = unload_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  const auto expected_unloaded = goal.controller_name;
  auto result_future = unload_client_->async_get_result(goal_handle);
  ASSERT_TRUE(
    wait_until(
      [this, expected_unloaded]() {
        return controller_manager_->last_unloaded() == expected_unloaded;
      },
      std::chrono::milliseconds{3000}))
    << "Timed out waiting for controller_manager to record unload request";

  wait_for_future_ready(result_future, 5s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(controller_manager_->last_unloaded(), "controller_to_unload");
}

TEST_F(ManipulatorActionServerFixture, UnloadControllerActionFailsWhenDeactivationFails)
{
  using namespace std::chrono_literals;

  controller_manager_->set_deactivate_ok(false);

  UnloadTrajController::Goal goal;
  goal.controller_name = "stubborn_controller";

  auto send_future = unload_client_->async_send_goal(goal);
  wait_for_future_ready(send_future, 2s);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = unload_client_->async_get_result(goal_handle);
  wait_for_future_ready(result_future, 2s);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_NE(
    wrapped_result.result->message.find("Deactivate error"),
    std::string::npos);
}

}  // namespace
