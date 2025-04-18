#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include "manymove_planner/moveit_cpp_planner.hpp"
#include "manymove_planner/planner_interface.hpp"
#include "manymove_planner/action_server.hpp"

class MoveItCppActionServerNode : public rclcpp::Node
{
public:
  static std::shared_ptr<MoveItCppActionServerNode> create(
      const std::shared_ptr<moveit_cpp::MoveItCpp> &moveit_cpp,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  {
    auto node = std::shared_ptr<MoveItCppActionServerNode>(
        new MoveItCppActionServerNode(moveit_cpp, options));
    node->initialize();
    return node;
  }

private:
  MoveItCppActionServerNode(
      const std::shared_ptr<moveit_cpp::MoveItCpp> &moveit_cpp,
      const rclcpp::NodeOptions &options)
      : Node("moveitcpp_action_server_node", options),
        moveit_cpp_(moveit_cpp)
  {
    RCLCPP_INFO(get_logger(), "MoveItCppActionServerNode constructed.");
  }

  void initialize()
  {
    auto self = shared_from_this();

    // Retrieve parameters
    std::vector<std::string> robot_prefixes = this->get_parameter("node_prefixes").as_string_array();
    std::vector<std::string> planning_groups = this->get_parameter("planning_groups").as_string_array();
    std::vector<std::string> base_frames = this->get_parameter("base_frames").as_string_array();
    std::vector<std::string> tcp_frames = this->get_parameter("tcp_frames").as_string_array();
    std::vector<std::string> traj_controllers = this->get_parameter("traj_controllers").as_string_array();

    if (robot_prefixes.size() != planning_groups.size() ||
        robot_prefixes.size() != base_frames.size() ||
        robot_prefixes.size() != tcp_frames.size() ||
        robot_prefixes.size() != traj_controllers.size())
    {
      RCLCPP_ERROR(get_logger(), "Mismatch in sizes of parameters!");
      rclcpp::shutdown();
      return;
    }

    for (size_t i = 0; i < robot_prefixes.size(); ++i)
    {
      std::string full_planning_group = robot_prefixes[i] + planning_groups[i];
      std::string full_base_frame = robot_prefixes[i] + base_frames[i];
      std::string full_controller = robot_prefixes[i] + traj_controllers[i];

      RCLCPP_INFO(get_logger(),
                  "Creating planner and action server for robot_prefix=%s, planning_group=%s",
                  robot_prefixes[i].c_str(), full_planning_group.c_str());

      auto planner = std::make_shared<MoveItCppPlanner>(
          self, full_planning_group, full_base_frame, full_controller, moveit_cpp_);

      auto action_server = std::make_shared<ManipulatorActionServer>(
          self, planner, robot_prefixes[i]);

      action_servers_.push_back(action_server);
    }

    RCLCPP_INFO(get_logger(), "All planners and action servers initialized successfully.");
  }

  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  std::vector<std::shared_ptr<ManipulatorActionServer>> action_servers_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_opts;
  node_opts.automatically_declare_parameters_from_overrides(true);

  // MoveItCpp Node (Dedicated for PlanningScene)
  auto moveitcpp_node = std::make_shared<rclcpp::Node>("moveitcpp_node", node_opts);
  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(moveitcpp_node);
  moveit_cpp->getPlanningSceneMonitor()->providePlanningSceneService();

  rclcpp::executors::SingleThreadedExecutor moveitcpp_executor;
  moveitcpp_executor.add_node(moveitcpp_node);

  // Thread dedicated to the MoveItCpp node's execution.
  std::thread moveitcpp_thread([&moveitcpp_executor = moveitcpp_executor]()
                               { moveitcpp_executor.spin(); });

  // Node and executor for Planners/Action Servers
  auto planners_node = MoveItCppActionServerNode::create(moveit_cpp, node_opts);
  rclcpp::executors::MultiThreadedExecutor planners_executor;
  planners_executor.add_node(planners_node);
  planners_executor.spin();

  // Cleanup
  rclcpp::shutdown();
  moveitcpp_thread.join();
  return 0;
}
