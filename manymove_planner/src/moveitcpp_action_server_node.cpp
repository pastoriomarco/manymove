#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include "manymove_planner/moveit_cpp_planner.hpp"
#include "manymove_planner/planner_interface.hpp"
#include "action_server.cpp"

class MoveItCppActionServerNode : public rclcpp::Node
{
public:
  // Factory method for safe creation.
  static std::shared_ptr<MoveItCppActionServerNode> create(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    // Create the node via new so that shared_from_this() will work afterward.
    auto node = std::shared_ptr<MoveItCppActionServerNode>(new MoveItCppActionServerNode(options));
    node->initialize();
    return node;
  }

private:
  // Private constructor: do not call shared_from_this() here!
  explicit MoveItCppActionServerNode(const rclcpp::NodeOptions &options)
    : Node("moveitcpp_action_server_node", options)
  {
    RCLCPP_INFO(get_logger(), "MoveItCppActionServerNode constructor called.");
  }

  // Initialization that uses shared_from_this()
  void initialize()
  {
    // Now that the node is fully constructed, obtain a shared pointer.
    auto self = shared_from_this();

    // 1) Create a single shared MoveItCpp instance
    try
    {
      moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(self);
      moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
      RCLCPP_INFO(get_logger(), "Successfully created shared MoveItCpp instance.");
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Failed to create MoveItCpp: %s", ex.what());
      rclcpp::shutdown();
      return;
    }

    // 2) Retrieve parameters from the parameter server.
    // Make sure your launch file sets these parameters with these exact names.
    std::vector<std::string> robot_prefixes = this->get_parameter("node_prefixes").as_string_array();
    std::vector<std::string> planning_groups  = this->get_parameter("planning_groups").as_string_array();
    std::vector<std::string> base_frames      = this->get_parameter("base_frames").as_string_array();
    std::vector<std::string> tcp_frames       = this->get_parameter("tcp_frames").as_string_array();
    std::vector<std::string> traj_controllers = this->get_parameter("traj_controllers").as_string_array();

    if (robot_prefixes.size() != planning_groups.size() ||
        robot_prefixes.size() != base_frames.size() ||
        robot_prefixes.size() != tcp_frames.size() ||
        robot_prefixes.size() != traj_controllers.size())
    {
      RCLCPP_ERROR(get_logger(),
                   "Mismatch in sizes of node_prefixes / planning_groups / base_frames / tcp_frames / traj_controllers!");
      rclcpp::shutdown();
      return;
    }

    // 3) Create one planner and one action server per robot using the shared MoveItCpp instance
    for (size_t i = 0; i < robot_prefixes.size(); ++i)
    {
      std::string full_planning_group = robot_prefixes[i] + planning_groups[i];
      std::string full_base_frame     = robot_prefixes[i] + base_frames[i];
      std::string full_tcp_frame      = robot_prefixes[i] + tcp_frames[i];
      std::string full_controller     = robot_prefixes[i] + traj_controllers[i];

      RCLCPP_INFO(get_logger(),
                  "Creating planner for robot_prefix=%s, planning group=%s (full: %s)",
                  robot_prefixes[i].c_str(), planning_groups[i].c_str(), full_planning_group.c_str());

      auto planner = std::make_shared<MoveItCppPlanner>(
          self, full_planning_group, full_base_frame, full_tcp_frame, full_controller, moveit_cpp_);

      RCLCPP_INFO(get_logger(),
                  "Creating action server for robot_prefix [%s]. Action topic: '%splan_manipulator'",
                  robot_prefixes[i].c_str(), robot_prefixes[i].c_str());

      auto action_server = std::make_shared<ManipulatorActionServer>(self, planner, robot_prefixes[i]);
      action_servers_.push_back(action_server);
    }

    RCLCPP_INFO(get_logger(), "All action servers created successfully.");
  }

private:
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  std::vector<std::shared_ptr<ManipulatorActionServer>> action_servers_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_opts;
  node_opts.automatically_declare_parameters_from_overrides(true);

  // Create the node using the factory method.
  auto node = MoveItCppActionServerNode::create(node_opts);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
