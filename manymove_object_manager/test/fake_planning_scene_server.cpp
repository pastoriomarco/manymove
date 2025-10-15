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

#include "fake_planning_scene_server.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <rclcpp/rclcpp.hpp>

FakePlanningSceneServer::FakePlanningSceneServer(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("fake_planning_scene_server", options))
{
  service_ = node_->create_service<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene", std::bind(
      &FakePlanningSceneServer::handleRequest, this, std::placeholders::_1,
      std::placeholders::_2));
}

rclcpp::Node::SharedPtr FakePlanningSceneServer::node() const
{
  return node_;
}

void FakePlanningSceneServer::setDefaultScene(const SceneState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  default_state_ = state;
}

void FakePlanningSceneServer::pushSceneResponse(const SceneState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queued_states_.push_back(state);
}

void FakePlanningSceneServer::clearSceneResponses()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queued_states_.clear();
}

void FakePlanningSceneServer::setResponseDelay(std::chrono::milliseconds delay)
{
  std::lock_guard<std::mutex> lock(mutex_);
  response_delay_ = delay;
}

void FakePlanningSceneServer::stopService()
{
  std::lock_guard<std::mutex> lock(mutex_);
  service_.reset();
}

void FakePlanningSceneServer::startService()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (service_) {
    return;
  }
  service_ = node_->create_service<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene", std::bind(
      &FakePlanningSceneServer::handleRequest, this, std::placeholders::_1,
      std::placeholders::_2));
}

bool FakePlanningSceneServer::serviceIsReady() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<bool>(service_);
}

FakePlanningSceneServer::SceneState FakePlanningSceneServer::nextState()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!queued_states_.empty()) {
    auto state = queued_states_.front();
    queued_states_.pop_front();
    return state;
  }
  return default_state_;
}

void FakePlanningSceneServer::handleRequest(
  const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request> request,
  std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> response)
{
  if (!service_) {
    throw std::runtime_error("Planning scene service unavailable");
  }

  const auto state = nextState();

  std::chrono::milliseconds delay{0};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    delay = response_delay_;
  }
  if (delay.count() > 0) {
    rclcpp::sleep_for(delay);
  }

  const auto components = request->components.components;

  if (
    components == 0 ||
    components & moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES ||
    components & moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY)
  {
    response->scene.world.collision_objects = state.collision_objects;
  }

  if (
    components == 0 ||
    components & moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS)
  {
    response->scene.robot_state.attached_collision_objects = state.attached_objects;
  }

  response->scene.is_diff = true;
}
