#include "manymove_hmi/ros2_worker.hpp"
#include "manymove_hmi/hmi_gui.hpp"
#include <sstream>
#include <QMetaObject>

#include <manymove_msgs/srv/set_blackboard_values.hpp>

using namespace std::chrono_literals;

Ros2Worker::Ros2Worker(const std::string &node_name, HmiGui *gui, const std::string &robot_prefix)
    : Node(node_name), gui_(gui), robot_prefix_(robot_prefix)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Ros2Worker node started with prefix: " << robot_prefix_);

    // Subscribe to the blackboard_status topic (as before)
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "blackboard_status", 10,
        std::bind(&Ros2Worker::statusCallback, this, std::placeholders::_1));

    // Create a single client for the "update_blackboard" service of HMIServiceNode
    update_blackboard_client_ =
        this->create_client<manymove_msgs::srv::SetBlackboardValues>("update_blackboard");

    // Optionally wait a little for the service
    if (!update_blackboard_client_->wait_for_service(2s))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Service 'update_blackboard' not available yet. Will still attempt calls.");
    }

    RCLCPP_INFO(this->get_logger(), "Ros2Worker fully initialized for prefix '%s'.",
                robot_prefix_.c_str());
}

void Ros2Worker::statusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string data = msg->data;

    bool stop_execution = (data.find("\"" + robot_prefix_ + "stop_execution\":true") != std::string::npos ||
                           data.find("\"" + robot_prefix_ + "stop_execution\": true") != std::string::npos);
    bool reset = (data.find("\"" + robot_prefix_ + "reset\":true") != std::string::npos ||
                  data.find("\"" + robot_prefix_ + "reset\": true") != std::string::npos);
    bool collision_detected = (data.find("\"" + robot_prefix_ + "collision_detected\":true") != std::string::npos ||
                               data.find("\"" + robot_prefix_ + "collision_detected\": true") != std::string::npos);

    // Update the Qt GUI in the main thread
    QMetaObject::invokeMethod(gui_, "updateStatus", Qt::QueuedConnection,
                              Q_ARG(QString, QString::fromStdString(robot_prefix_)),
                              Q_ARG(bool, stop_execution),
                              Q_ARG(bool, reset),
                              Q_ARG(bool, collision_detected));
}

void Ros2Worker::callStartExecution()
{
    auto request = std::make_shared<manymove_msgs::srv::SetBlackboardValues::Request>();

    request->key.push_back(robot_prefix_ + "stop_execution");
    request->value_type.push_back("bool");
    request->value_data.push_back("false"); // JSON "false"

    auto future = update_blackboard_client_->async_send_request(request);

    if (!update_blackboard_client_->wait_for_service(1s))
    {
        RCLCPP_WARN(this->get_logger(), "callStartExecution() => service not available yet.");
    }
}

void Ros2Worker::callStopExecution()
{
    // "STOP" => set <prefix + "stop_execution">=true
    auto request = std::make_shared<manymove_msgs::srv::SetBlackboardValues::Request>();
    request->key.push_back(robot_prefix_ + "stop_execution");
    request->value_type.push_back("bool");
    request->value_data.push_back("true");

    auto future = update_blackboard_client_->async_send_request(request);
    if (!update_blackboard_client_->wait_for_service(1s))
    {
        RCLCPP_WARN(this->get_logger(), "callStopExecution() => service not available yet.");
    }
}

void Ros2Worker::callResetProgram()
{
    // The old logic set BOTH "stop_execution"=true and "reset"=true
    // We'll replicate that in a single request with two entries:
    auto request = std::make_shared<manymove_msgs::srv::SetBlackboardValues::Request>();

    // 1st key: <prefix>stop_execution = true
    request->key.push_back(robot_prefix_ + "stop_execution");
    request->value_type.push_back("bool");
    request->value_data.push_back("true");

    // 2nd key: <prefix>reset = true
    request->key.push_back(robot_prefix_ + "reset");
    request->value_type.push_back("bool");
    request->value_data.push_back("true");

    auto future = update_blackboard_client_->async_send_request(request);
    if (!update_blackboard_client_->wait_for_service(1s))
    {
        RCLCPP_WARN(this->get_logger(), "callResetProgram() => service not available yet.");
    }
}
