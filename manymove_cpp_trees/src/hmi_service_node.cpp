#include "manymove_cpp_trees/hmi_service_node.hpp"
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

namespace manymove_cpp_trees
{

    HMIServiceNode::HMIServiceNode(const std::string &node_name, BT::Blackboard::Ptr blackboard, std::string robot_prefix)
        : Node(node_name), blackboard_(blackboard), robot_prefix_(robot_prefix)
    {
        // Create the three services.
        start_execution_srv_ = this->create_service<std_srvs::srv::Empty>(
            robot_prefix_ + "start_execution",
            std::bind(&HMIServiceNode::handle_start_execution, this,
                      std::placeholders::_1, std::placeholders::_2));

        stop_execution_srv_ = this->create_service<std_srvs::srv::Empty>(
            robot_prefix_ + "stop_execution",
            std::bind(&HMIServiceNode::handle_stop_execution, this,
                      std::placeholders::_1, std::placeholders::_2));

        reset_program_srv_ = this->create_service<std_srvs::srv::Empty>(
            robot_prefix_ + "reset_program",
            std::bind(&HMIServiceNode::handle_reset_program, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "HMI Service Node started.");

        // Create a publisher for blackboard status using a standard string message.
        publisher_ = this->create_publisher<std_msgs::msg::String>(robot_prefix_ + "blackboard_status", 10);

        // Create a timer that calls publishBlackboardStatus() every 250ms.
        status_timer_ = this->create_wall_timer(
            250ms, std::bind(&HMIServiceNode::publishBlackboardStatus, this));
    }

    void HMIServiceNode::handle_start_execution(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution to false and execution_resumed to true.
        blackboard_->set(robot_prefix_ + "stop_execution", false);
        blackboard_->set(robot_prefix_ + "execution_resumed", true);
        RCLCPP_INFO_STREAM(this->get_logger(), "start_execution: "
                                                   << robot_prefix_ << "stop_execution=false, "
                                                   << robot_prefix_ << "execution_resumed=true.");
    }

    void HMIServiceNode::handle_stop_execution(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution to true.
        blackboard_->set(robot_prefix_ + "stop_execution", true);
        RCLCPP_INFO_STREAM(this->get_logger(), "stop_execution: " << robot_prefix_ << "stop_execution=true.");
    }

    void HMIServiceNode::handle_reset_program(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution and abort_mission to true and execution_resumed to false.
        blackboard_->set(robot_prefix_ + "stop_execution", true);
        blackboard_->set(robot_prefix_ + "abort_mission", true);
        blackboard_->set(robot_prefix_ + "execution_resumed", false);
        RCLCPP_INFO_STREAM(this->get_logger(), "reset_program: "
                                                   << robot_prefix_ << "stop_execution=true, "
                                                   << robot_prefix_ << "abort_mission=true, "
                                                   << robot_prefix_ << "execution_resumed=false.");
    }

    void HMIServiceNode::publishBlackboardStatus()
    {
        // Retrieve the three keys from the blackboard.
        bool execution_resumed = false;
        bool stop_execution = false;
        bool abort_mission = false;
        bool collision_detected = false;
        blackboard_->get(robot_prefix_ + "execution_resumed", execution_resumed);
        blackboard_->get(robot_prefix_ + "stop_execution", stop_execution);
        blackboard_->get(robot_prefix_ + "abort_mission", abort_mission);
        blackboard_->get(robot_prefix_ + "collision_detected", collision_detected);

        // Create a JSON string with the status.
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << "{\"" << robot_prefix_ << "execution_resumed\": " << (execution_resumed ? "true" : "false")
           << ", \"" << robot_prefix_ << "stop_execution\": " << (stop_execution ? "true" : "false")
           << ", \"" << robot_prefix_ << "abort_mission\": " << (abort_mission ? "true" : "false")
           << ", \"" << robot_prefix_ << "collision_detected\": " << (collision_detected ? "true" : "false") << "}";
        msg.data = ss.str();

        publisher_->publish(msg);
    }

} // namespace manymove_cpp_trees
