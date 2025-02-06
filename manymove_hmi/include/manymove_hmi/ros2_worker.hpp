#ifndef ROS2_WORKER_HPP
#define ROS2_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <chrono>
#include <memory>
#include <string>

class HmiGui;

class Ros2Worker : public rclcpp::Node {
public:
    /// The constructor remains the same.
    Ros2Worker(const std::string &node_name, HmiGui *gui, const std::string &robot_prefix = "");

    // Methods to call the three services.
    void callStartExecution();
    void callStopExecution();
    void callResetProgram();

    // Getter for the robot prefix (derived from the node name).
    const std::string & getRobotPrefix() const { return robot_prefix_; }

private:
    /// Callback for the blackboard_status topic.
    void statusCallback(const std_msgs::msg::String::SharedPtr msg);

    HmiGui *gui_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;

    // For simplicity, we extract the robot prefix from the node name.
    std::string robot_prefix_;
};

#endif // ROS2_WORKER_HPP
