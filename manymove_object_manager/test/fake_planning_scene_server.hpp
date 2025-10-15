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

#ifndef FAKE_PLANNING_SCENE_SERVER_HPP_
#define FAKE_PLANNING_SCENE_SERVER_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

class FakePlanningSceneServer
{
public:
  struct SceneState
  {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_objects;
  };

  explicit FakePlanningSceneServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp::Node::SharedPtr node() const;

  void setDefaultScene(const SceneState & state);
  void pushSceneResponse(const SceneState & state);
  void clearSceneResponses();
  void setResponseDelay(std::chrono::milliseconds delay);

  void stopService();
  void startService();
  bool serviceIsReady() const;

private:
  void handleRequest(
    const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request> request,
    std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> response);

  SceneState nextState();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr service_;

  mutable std::mutex mutex_;
  SceneState default_state_;
  std::deque<SceneState> queued_states_;
  std::chrono::milliseconds response_delay_{0};
};

#endif  // FAKE_PLANNING_SCENE_SERVER_HPP_
