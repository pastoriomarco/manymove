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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <manymove_msgs/action/add_collision_object.hpp>
#include <manymove_msgs/action/remove_collision_object.hpp>
#include <manymove_msgs/action/attach_detach_object.hpp>
#include <manymove_msgs/action/get_object_pose.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "fake_planning_scene_server.hpp"
#include "manymove_object_manager/object_manager.hpp"

using AddCollisionObject = manymove_msgs::action::AddCollisionObject;
using RemoveCollisionObject = manymove_msgs::action::RemoveCollisionObject;
using AttachDetachObject = manymove_msgs::action::AttachDetachObject;
using GetObjectPose = manymove_msgs::action::GetObjectPose;

namespace
{
moveit_msgs::msg::CollisionObject makeBoxCollisionObject(
  const std::string & id, const std::string & frame_id = "world")
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame_id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.1, 0.1, 0.1};
  object.primitives.push_back(primitive);

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(pose);

  return object;
}

AddCollisionObject::Goal makeAddGoal(const std::string & id)
{
  AddCollisionObject::Goal goal;
  goal.id = id;
  goal.shape = "box";
  goal.dimensions = {0.1, 0.1, 0.1};
  goal.pose.orientation.w = 1.0;
  return goal;
}

moveit_msgs::msg::AttachedCollisionObject makeAttachedCollisionObject(
  const std::string & id, const std::string & link_name)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.object = makeBoxCollisionObject(id, link_name);
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached.link_name = link_name;
  return attached;
}
}  // namespace

class ObjectManagerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    fake_server_ = std::make_shared<FakePlanningSceneServer>();
    fake_server_->setDefaultScene(FakePlanningSceneServer::SceneState{});
    fake_server_->clearSceneResponses();

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2);
    executor_->add_node(fake_server_->node());

    test_node_ = std::make_shared<rclcpp::Node>("object_manager_node_test_client");
    executor_->add_node(test_node_);

    executor_thread_ = std::thread(
      [this]() {
        executor_->spin();
      });

    // Give the executor time to register the service
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    object_manager_node_ = std::make_shared<manymove_object_manager::ObjectManagerNode>();
    executor_->add_node(object_manager_node_);

    add_client_ = rclcpp_action::create_client<AddCollisionObject>(
      test_node_, "add_collision_object");
    ASSERT_TRUE(add_client_->wait_for_action_server(std::chrono::seconds(2)));
    remove_client_ = rclcpp_action::create_client<RemoveCollisionObject>(
      test_node_, "remove_collision_object");
    ASSERT_TRUE(remove_client_->wait_for_action_server(std::chrono::seconds(2)));
    attach_detach_client_ = rclcpp_action::create_client<AttachDetachObject>(
      test_node_, "attach_detach_object");
    ASSERT_TRUE(attach_detach_client_->wait_for_action_server(std::chrono::seconds(2)));
    get_pose_client_ = rclcpp_action::create_client<GetObjectPose>(
      test_node_, "get_object_pose");
    ASSERT_TRUE(get_pose_client_->wait_for_action_server(std::chrono::seconds(2)));
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    if (object_manager_node_) {
      executor_->remove_node(object_manager_node_);
      object_manager_node_.reset();
    }
    if (test_node_) {
      executor_->remove_node(test_node_);
      test_node_.reset();
    }
    if (fake_server_) {
      executor_->remove_node(fake_server_->node());
      fake_server_.reset();
    }

    add_client_.reset();
    remove_client_.reset();
    attach_detach_client_.reset();
    get_pose_client_.reset();
    executor_.reset();
  }

  FakePlanningSceneServer::SceneState sceneWithObject(
    const moveit_msgs::msg::CollisionObject & object)
  {
    FakePlanningSceneServer::SceneState state;
    state.collision_objects.push_back(object);
    return state;
  }

  FakePlanningSceneServer::SceneState sceneWithAttachedObject(
    const moveit_msgs::msg::AttachedCollisionObject & attached_object)
  {
    FakePlanningSceneServer::SceneState state;
    state.attached_objects.push_back(attached_object);
    return state;
  }

  std::shared_ptr<FakePlanningSceneServer> fake_server_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<manymove_object_manager::ObjectManagerNode> object_manager_node_;
  std::shared_ptr<rclcpp_action::Client<AddCollisionObject>> add_client_;
  std::shared_ptr<rclcpp_action::Client<RemoveCollisionObject>> remove_client_;
  std::shared_ptr<rclcpp_action::Client<AttachDetachObject>> attach_detach_client_;
  std::shared_ptr<rclcpp_action::Client<GetObjectPose>> get_pose_client_;
  std::thread executor_thread_;
};

TEST_F(ObjectManagerNodeTest, RejectsAddGoalWhenObjectAlreadyExists)
{
  fake_server_->setDefaultScene(sceneWithObject(makeBoxCollisionObject("existing_object")));

  auto goal = makeAddGoal("existing_object");
  auto send_future = add_client_->async_send_goal(goal);

  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  EXPECT_EQ(goal_handle, nullptr);
}

TEST_F(ObjectManagerNodeTest, AddGoalSucceedsAfterObjectAppears)
{
  fake_server_->setDefaultScene(FakePlanningSceneServer::SceneState{});

  std::mutex mutex;
  std::condition_variable cv;
  bool message_received = false;
  moveit_msgs::msg::CollisionObject published_object;

  auto subscription =
    test_node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/collision_object", rclcpp::SystemDefaultsQoS(),
    [&](const moveit_msgs::msg::CollisionObject::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        published_object = *msg;
        message_received = true;
      }
      fake_server_->setDefaultScene(sceneWithObject(*msg));
      cv.notify_one();
    });
  (void)subscription;

  auto goal = makeAddGoal("new_box");
  auto send_future = add_client_->async_send_goal(goal);

  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&]() {return message_received;}));
  }

  auto result_future = add_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Object added successfully.");

  EXPECT_EQ(published_object.id, "new_box");
}

TEST_F(ObjectManagerNodeTest, AddGoalFailsWhenObjectNeverAppears)
{
  fake_server_->setDefaultScene(FakePlanningSceneServer::SceneState{});

  auto goal = makeAddGoal("missing_box");
  auto send_future = add_client_->async_send_goal(goal);

  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = add_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Failed to verify addition of object.");
}

TEST_F(ObjectManagerNodeTest, RemoveGoalSucceedsWhenObjectRemoved)
{
  const auto object = makeBoxCollisionObject("removable_box");
  fake_server_->setDefaultScene(sceneWithObject(object));

  std::mutex mutex;
  std::condition_variable cv;
  bool message_received = false;
  moveit_msgs::msg::CollisionObject remove_message;

  auto subscription =
    test_node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/collision_object", rclcpp::SystemDefaultsQoS(),
    [&](const moveit_msgs::msg::CollisionObject::SharedPtr msg) {
      if (
        msg->operation == moveit_msgs::msg::CollisionObject::REMOVE &&
        msg->id == object.id)
      {
        {
          std::lock_guard<std::mutex> lock(mutex);
          remove_message = *msg;
          message_received = true;
        }
        fake_server_->setDefaultScene(FakePlanningSceneServer::SceneState{});
        cv.notify_one();
      }
    });
  (void)subscription;

  RemoveCollisionObject::Goal goal;
  goal.id = object.id;

  auto send_future = remove_client_->async_send_goal(goal);
  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&]() {return message_received;}));
  }

  auto result_future = remove_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Object removed successfully.");
  EXPECT_EQ(remove_message.id, object.id);
  EXPECT_EQ(remove_message.operation, moveit_msgs::msg::CollisionObject::REMOVE);
}

TEST_F(ObjectManagerNodeTest, RemoveGoalReportsAlreadyMissingObject)
{
  fake_server_->setDefaultScene(FakePlanningSceneServer::SceneState{});

  RemoveCollisionObject::Goal goal;
  goal.id = "ghost_box";

  auto send_future = remove_client_->async_send_goal(goal);
  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = remove_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Object not found in the scene.");
}

TEST_F(ObjectManagerNodeTest, RemoveGoalFailsWhenObjectAttached)
{
  auto attached_object = makeAttachedCollisionObject("attached_box", "tool0");
  fake_server_->setDefaultScene(sceneWithAttachedObject(attached_object));

  RemoveCollisionObject::Goal goal;
  goal.id = attached_object.object.id;

  auto send_future = remove_client_->async_send_goal(goal);
  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = remove_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Object attached, unable to remove.");
}

TEST_F(ObjectManagerNodeTest, AttachGoalSucceeds)
{
  const auto object = makeBoxCollisionObject("attachable_box");
  fake_server_->setDefaultScene(sceneWithObject(object));

  std::mutex mutex;
  std::condition_variable cv;
  bool message_received = false;

  auto subscription =
    test_node_->create_subscription<moveit_msgs::msg::AttachedCollisionObject>(
    "/attached_collision_object", rclcpp::SystemDefaultsQoS(),
    [&](const moveit_msgs::msg::AttachedCollisionObject::SharedPtr msg) {
      if (
        msg->object.id == object.id &&
        msg->object.operation == moveit_msgs::msg::CollisionObject::ADD)
      {
        {
          std::lock_guard<std::mutex> lock(mutex);
          message_received = true;
        }
        fake_server_->setDefaultScene(
          sceneWithAttachedObject(makeAttachedCollisionObject(object.id, msg->link_name)));
        cv.notify_one();
      }
    });
  (void)subscription;

  AttachDetachObject::Goal goal;
  goal.object_id = object.id;
  goal.link_name = "tool0";
  goal.attach = true;
  goal.touch_links = {"tool0", "finger_left"};

  auto send_future = attach_detach_client_->async_send_goal(goal);
  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&]() {return message_received;}));
  }

  auto result_future = attach_detach_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Object attached successfully.");
}

TEST_F(ObjectManagerNodeTest, AttachGoalFailsWhenObjectMissing)
{
  fake_server_->setDefaultScene(FakePlanningSceneServer::SceneState{});

  AttachDetachObject::Goal goal;
  goal.object_id = "missing_box";
  goal.link_name = "tool0";
  goal.attach = true;

  auto send_future = attach_detach_client_->async_send_goal(goal);
  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = attach_detach_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_EQ(
    wrapped_result.result->message,
    "'missing_box' does not exist in the planning scene.");
}

TEST_F(ObjectManagerNodeTest, DetachGoalSucceeds)
{
  auto attached_object = makeAttachedCollisionObject("detachable_box", "tool0");
  fake_server_->setDefaultScene(sceneWithAttachedObject(attached_object));

  std::mutex mutex;
  std::condition_variable cv;
  bool message_received = false;

  auto subscription =
    test_node_->create_subscription<moveit_msgs::msg::AttachedCollisionObject>(
    "/attached_collision_object", rclcpp::SystemDefaultsQoS(),
    [&](const moveit_msgs::msg::AttachedCollisionObject::SharedPtr msg) {
      if (
        msg->object.id == attached_object.object.id &&
        msg->object.operation == moveit_msgs::msg::CollisionObject::REMOVE)
      {
        {
          std::lock_guard<std::mutex> lock(mutex);
          message_received = true;
        }
        fake_server_->setDefaultScene(
          sceneWithObject(makeBoxCollisionObject(attached_object.object.id)));
        cv.notify_one();
      }
    });
  (void)subscription;

  AttachDetachObject::Goal goal;
  goal.object_id = attached_object.object.id;
  goal.link_name = attached_object.link_name;
  goal.attach = false;

  auto send_future = attach_detach_client_->async_send_goal(goal);
  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&]() {return message_received;}));
  }

  auto result_future = attach_detach_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Object detached successfully.");
}

TEST_F(ObjectManagerNodeTest, GetObjectPoseFailsWhenObjectMissing)
{
  fake_server_->setDefaultScene(FakePlanningSceneServer::SceneState{});

  GetObjectPose::Goal goal;
  goal.object_id = "ghost";
  goal.pre_transform_xyz_rpy.assign(6, 0.0);
  goal.post_transform_xyz_rpy.assign(6, 0.0);

  auto send_future = get_pose_client_->async_send_goal(goal);
  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = get_pose_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Object not found in planning scene.");
}

TEST_F(ObjectManagerNodeTest, GetObjectPoseSucceedsWithIdentityTransforms)
{
  auto object = makeBoxCollisionObject("pose_box");
  object.pose.position.x = 0.5;
  object.pose.position.y = -0.3;
  object.pose.position.z = 0.2;
  object.pose.orientation.x = 0.0;
  object.pose.orientation.y = 0.0;
  object.pose.orientation.z = 0.0;
  object.pose.orientation.w = 1.0;
  fake_server_->setDefaultScene(sceneWithObject(object));

  GetObjectPose::Goal goal;
  goal.object_id = object.id;
  goal.pre_transform_xyz_rpy.assign(6, 0.0);
  goal.post_transform_xyz_rpy.assign(6, 0.0);

  auto send_future = get_pose_client_->async_send_goal(goal);
  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = get_pose_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(
    wrapped_result.result->message,
    "Pose updated successfully with updated transformation logic.");
  const auto & pose = wrapped_result.result->pose;
  EXPECT_NEAR(pose.position.x, object.pose.position.x, 1e-6);
  EXPECT_NEAR(pose.position.y, object.pose.position.y, 1e-6);
  EXPECT_NEAR(pose.position.z, object.pose.position.z, 1e-6);
  EXPECT_NEAR(pose.orientation.x, object.pose.orientation.x, 1e-6);
  EXPECT_NEAR(pose.orientation.y, object.pose.orientation.y, 1e-6);
  EXPECT_NEAR(pose.orientation.z, object.pose.orientation.z, 1e-6);
  EXPECT_NEAR(pose.orientation.w, object.pose.orientation.w, 1e-6);
}

TEST_F(ObjectManagerNodeTest, ActionsAbortWhenPlanningSceneUnavailable)
{
  fake_server_->stopService();

  auto goal = makeAddGoal("service_down_box");
  auto send_future = add_client_->async_send_goal(goal);

  ASSERT_EQ(send_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = add_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->message, "Planning scene service unavailable.");

  fake_server_->startService();
}
