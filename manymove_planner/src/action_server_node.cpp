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

#include <rclcpp/rclcpp.hpp>
#include "manymove_planner/move_group_planner.hpp"
#include "manymove_planner/moveit_cpp_planner.hpp"
#include "manymove_planner/action_server.hpp"

int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);

  // Building loader node to get parameters
  rclcpp::NodeOptions loader_options;
  loader_options.automatically_declare_parameters_from_overrides(true);
  auto loader_node = rclcpp::Node::make_shared("param_loader_node", loader_options);

  std::string node_prefix;
  loader_node->get_parameter_or<std::string>("node_prefix", node_prefix, "");

  // Build the “real” node name from the prefix.
  // For multiple robots, e.g. "L_action_server_node" or "R_action_server_node".
  std::string node_name = node_prefix + "action_server_node";
  RCLCPP_INFO(loader_node->get_logger(), "Action Server Node name: %s", node_name.c_str());

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared(node_name, "", node_options);

  // Retrieve parameters
  std::string planner_type;
  node->get_parameter_or<std::string>("planner_type", planner_type, "movegroup");

  std::string planner_prefix;
  loader_node->get_parameter_or<std::string>("planner_prefix", planner_prefix, "");
  std::string planning_group;
  node->get_parameter_or<std::string>("planning_group", planning_group, "lite6");
  std::string base_frame;
  node->get_parameter_or<std::string>("base_link", base_frame, "link_base");
  std::string tcp_frame;
  node->get_parameter_or<std::string>("tcp_frame", tcp_frame, "link_tcp");
  std::string traj_controller;
  node->get_parameter_or<std::string>("traj_controller", traj_controller, "lite6_traj_controller");

  planning_group = planner_prefix + planning_group;
  base_frame = planner_prefix + base_frame;
  tcp_frame = planner_prefix + tcp_frame;
  traj_controller = planner_prefix + traj_controller;

  // Instantiate the appropriate planner based on the planner_type parameter
  std::shared_ptr<PlannerInterface> planner;

  if (planner_type == "moveitcpp") {
    planner = std::make_shared<MoveItCppPlanner>(node, planning_group, base_frame, traj_controller);
    RCLCPP_INFO(node->get_logger(), "===================================================");
    RCLCPP_INFO(node->get_logger(), "Using MoveItCppPlanner.");
    RCLCPP_INFO(node->get_logger(), "===================================================");
  }
  else if (planner_type == "movegroup") {
    planner = std::make_shared<MoveGroupPlanner>(node, planning_group, base_frame, traj_controller);
    RCLCPP_INFO(node->get_logger(), "===================================================");
    RCLCPP_INFO(node->get_logger(), "Using MoveGroupPlanner.");
    RCLCPP_INFO(node->get_logger(), "===================================================");
  }
  else {
    RCLCPP_ERROR(node->get_logger(),
                 "Unknown planner_type: %s. Valid options are 'moveitcpp' and 'movegroup'.",
                 planner_type.c_str());
    return 1;
  }

  auto server = std::make_shared<ManipulatorActionServer>(node, planner, planner_prefix);

  loader_node.reset();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
