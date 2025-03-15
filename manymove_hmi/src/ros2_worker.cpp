#include "manymove_hmi/ros2_worker.hpp"
#include "manymove_hmi/hmi_gui.hpp"
#include <sstream>
#include <QMetaObject>

using namespace std::chrono_literals;

Ros2Worker::Ros2Worker(const std::string &node_name, HmiGui *gui, const std::string &robot_prefix)
    : Node(node_name), gui_(gui), robot_prefix_(robot_prefix)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Ros2Worker node started with prefix: " << robot_prefix_);

    // Now use the prefix for topics:
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        robot_prefix_ + "blackboard_status", 10,
        std::bind(&Ros2Worker::statusCallback, this, std::placeholders::_1));

    // Create service clients using the prefix.
    start_client_ = this->create_client<std_srvs::srv::Empty>(robot_prefix_ + "start_execution");
    stop_client_ = this->create_client<std_srvs::srv::Empty>(robot_prefix_ + "stop_execution");
    reset_client_ = this->create_client<std_srvs::srv::Empty>(robot_prefix_ + "reset_program");

    RCLCPP_INFO(this->get_logger(), "Ros2Worker fully initialized.");
}

void Ros2Worker::statusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string data = msg->data;
    bool stop_execution = (data.find("\"" + robot_prefix_ + "stop_execution\": true") != std::string::npos);
    bool reset = (data.find("\"" + robot_prefix_ + "reset\": true") != std::string::npos);
    bool collision_detected = (data.find("\"" + robot_prefix_ + "collision_detected\": true") != std::string::npos);

    QMetaObject::invokeMethod(gui_, "updateStatus", Qt::QueuedConnection,
                              Q_ARG(QString, QString::fromStdString(robot_prefix_)),
                              Q_ARG(bool, stop_execution),
                              Q_ARG(bool, reset),
                              Q_ARG(bool, collision_detected));
}

void Ros2Worker::callStartExecution()
{
    if (!start_client_->wait_for_service(1s))
    {
        RCLCPP_ERROR(this->get_logger(), "start_execution service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    start_client_->async_send_request(request);
}

void Ros2Worker::callStopExecution()
{
    if (!stop_client_->wait_for_service(1s))
    {
        RCLCPP_ERROR(this->get_logger(), "stop_execution service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    stop_client_->async_send_request(request);
}

void Ros2Worker::callResetProgram()
{
    if (!reset_client_->wait_for_service(1s))
    {
        RCLCPP_ERROR(this->get_logger(), "reset_program service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_client_->async_send_request(request);
}
