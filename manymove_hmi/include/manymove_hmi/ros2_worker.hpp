/*
BSD 3-Clause License
Copyright (c) 2024-2025, Flexin Group SRL
All rights reserved.
See LICENSE file in the project root for full license text.
*/

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
  Ros2Worker(const std::string & node_name, HmiGui * gui, const std::string & robot_prefix = "");

  void callStartExecution();
  void callStopExecution();
  void callResetProgram();

  const std::string & getRobotPrefix() const {return robot_prefix_;}

private:
  void statusCallback(const std_msgs::msg::String::SharedPtr msg);

  HmiGui * gui_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<manymove_msgs::srv::SetBlackboardValues>::SharedPtr update_blackboard_client_;

  std::string robot_prefix_;
};

#endif // ROS2_WORKER_HPP
