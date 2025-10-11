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

#ifndef ROS2_WORKER_HPP
#define ROS2_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <manymove_msgs/srv/set_blackboard_values.hpp>
#include "manymove_hmi/hmi_gui.hpp"
#include "manymove_hmi/app_module.hpp"

// Forward declarations
class HmiGui;
class AppModule;

class Ros2Worker : public rclcpp::Node
{
public:
Ros2Worker(const std::string&node_name, HmiGui*gui, const std::string&robot_prefix = "");

void callStartExecution();
void callStopExecution();
void callResetProgram();

const std::string& getRobotPrefix() const
{
  return robot_prefix_;
}

private:
void statusCallback(const std_msgs::msg::String::SharedPtr msg);

HmiGui*gui_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
rclcpp::Client<manymove_msgs::srv::SetBlackboardValues>::SharedPtr update_blackboard_client_;

std::string robot_prefix_;
};

#endif // ROS2_WORKER_HPP
