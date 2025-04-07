#include "manymove_hmi/ros2_worker.hpp"

#include <sstream>
#include <QMetaObject>
#include <algorithm>
#include <cctype>
#include <stdexcept>

using namespace std::chrono_literals;

Ros2Worker::Ros2Worker(const std::string &node_name, HmiGui *gui, const std::string &robot_prefix)
    : Node(node_name), gui_(gui), robot_prefix_(robot_prefix)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Ros2Worker node started with prefix: " << robot_prefix_);

    // Subscribe to the blackboard_status topic.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "blackboard_status", 10,
        std::bind(&Ros2Worker::statusCallback, this, std::placeholders::_1));

    // Create a client for the update_blackboard service.
    update_blackboard_client_ =
        this->create_client<manymove_msgs::srv::SetBlackboardValues>("update_blackboard");

    // Wait a little for the service.
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

    // --- Process the boolean fields used for button updates (unchanged) ---
    bool stop_execution = (data.find("\"" + robot_prefix_ + "stop_execution\":true") != std::string::npos ||
                           data.find("\"" + robot_prefix_ + "stop_execution\": true") != std::string::npos);
    bool reset = (data.find("\"" + robot_prefix_ + "reset\":true") != std::string::npos ||
                  data.find("\"" + robot_prefix_ + "reset\": true") != std::string::npos);
    bool collision_detected = (data.find("\"" + robot_prefix_ + "collision_detected\":true") != std::string::npos ||
                               data.find("\"" + robot_prefix_ + "collision_detected\": true") != std::string::npos);

    // Update GUI status for buttons.
    QMetaObject::invokeMethod(gui_, "updateStatus", Qt::QueuedConnection,
                              Q_ARG(QString, QString::fromStdString(robot_prefix_)),
                              Q_ARG(bool, stop_execution),
                              Q_ARG(bool, reset),
                              Q_ARG(bool, collision_detected));

    // --- Now process the keys for the AppModule ---
    // Try to find the AppModule widget as a child of HmiGui.
    AppModule *appModule = gui_->findChild<AppModule *>();
    if (!appModule)
    {
        RCLCPP_DEBUG(this->get_logger(), "No AppModule found in the GUI. Blackboard keys not updated.");
        return;
    }

    // Get the known keys from AppModule.
    const std::vector<BlackboardKey> &knownKeys = AppModule::getKnownKeys();

    // For each known key, try to extract its value from the JSON string.
    for (const auto &bk : knownKeys)
    {
        // Build a pattern from the key. We convert bk.key to std::string for search.
        std::string pattern = "\"" + bk.key.toStdString() + "\":";
        size_t pos = data.find(pattern);
        if (pos == std::string::npos)
        {
            // Key not found; update with an empty value (which AppModule shows as "N/A").
            QMetaObject::invokeMethod(appModule, "updateField", Qt::QueuedConnection,
                                      Q_ARG(QString, bk.key),
                                      Q_ARG(QString, QString()));
            continue;
        }
        // Find the beginning of the value (after the colon).
        size_t valStart = pos + pattern.length();
        // Skip any whitespace.
        while (valStart < data.size() && std::isspace(data[valStart]))
        {
            ++valStart;
        }

        std::string valueStr;
        if (bk.type == "double")
        {
            // Read until the next comma or closing brace.
            size_t valEnd = data.find_first_of(",}", valStart);
            if (valEnd == std::string::npos)
                valEnd = data.size();
            valueStr = data.substr(valStart, valEnd - valStart);
            // Trim whitespace.
            valueStr.erase(valueStr.begin(),
                           std::find_if(valueStr.begin(), valueStr.end(),
                                        [](unsigned char ch)
                                        { return !std::isspace(ch); }));
            valueStr.erase(std::find_if(valueStr.rbegin(), valueStr.rend(),
                                        [](unsigned char ch)
                                        { return !std::isspace(ch); })
                               .base(),
                           valueStr.end());
            try
            {
                double dval = std::stod(valueStr);
                valueStr = std::to_string(dval);
            }
            catch (const std::exception &)
            {
                valueStr = "";
            }
        }
        else if (bk.type == "int")
        {
            size_t valEnd = data.find_first_of(",}", valStart);
            if (valEnd == std::string::npos)
                valEnd = data.size();
            valueStr = data.substr(valStart, valEnd - valStart);
            valueStr.erase(valueStr.begin(),
                           std::find_if(valueStr.begin(), valueStr.end(),
                                        [](unsigned char ch)
                                        { return !std::isspace(ch); }));
            valueStr.erase(std::find_if(valueStr.rbegin(), valueStr.rend(),
                                        [](unsigned char ch)
                                        { return !std::isspace(ch); })
                               .base(),
                           valueStr.end());
            try
            {
                int ival = std::stoi(valueStr);
                valueStr = std::to_string(ival);
            }
            catch (const std::exception &)
            {
                valueStr = "";
            }
        }
        else if (bk.type == "double_array")
        {
            // Expecting a JSON array, e.g.: [1.23, 4.56, ...]
            if (data[valStart] == '[')
            {
                size_t bracketEnd = data.find_first_of("]", valStart);
                if (bracketEnd == std::string::npos)
                    bracketEnd = data.size();
                valueStr = data.substr(valStart, bracketEnd - valStart + 1); // include closing ']'
            }
            else
            {
                size_t valEnd = data.find_first_of(",}", valStart);
                if (valEnd == std::string::npos)
                    valEnd = data.size();
                valueStr = data.substr(valStart, valEnd - valStart);
            }
        }
        else if (bk.type == "pose")
        {
            // For a pose, assume the value is a JSON object starting with '{'
            if (data[valStart] == '{')
            {
                int braceCount = 0;
                size_t idx = valStart;
                while (idx < data.size())
                {
                    if (data[idx] == '{')
                        braceCount++;
                    else if (data[idx] == '}')
                        braceCount--;
                    idx++;
                    if (braceCount == 0)
                        break;
                }
                valueStr = data.substr(valStart, idx - valStart);
            }
            else
            {
                size_t valEnd = data.find_first_of(",}", valStart);
                if (valEnd == std::string::npos)
                    valEnd = data.size();
                valueStr = data.substr(valStart, valEnd - valStart);
            }
        }
        else if (bk.type == "string")
        {
            // For a string, assume it is enclosed in quotes.
            size_t valEnd = data.find_first_of(",}", valStart);
            if (valEnd == std::string::npos)
                valEnd = data.size();
            valueStr = data.substr(valStart, valEnd - valStart);
            if (!valueStr.empty() && valueStr.front() == '"')
            {
                valueStr.erase(0, 1);
                if (!valueStr.empty() && valueStr.back() == '"')
                    valueStr.pop_back();
            }
        }
        else
        {
            // For bool or unknown types, read until comma or brace.
            size_t valEnd = data.find_first_of(",}", valStart);
            if (valEnd == std::string::npos)
                valEnd = data.size();
            valueStr = data.substr(valStart, valEnd - valStart);
            if (!valueStr.empty() && valueStr.front() == '"')
            {
                valueStr.erase(0, 1);
                if (!valueStr.empty() && valueStr.back() == '"')
                    valueStr.pop_back();
            }
        }

        // Invoke updateField with the extracted value.
        QMetaObject::invokeMethod(appModule, "updateField", Qt::QueuedConnection,
                                  Q_ARG(QString, bk.key),
                                  Q_ARG(QString, QString::fromStdString(valueStr)));
    }
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
    // "STOP" => set <prefix + "stop_execution"> = true
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
    // Set both <prefix>stop_execution = true and <prefix>reset = true
    auto request = std::make_shared<manymove_msgs::srv::SetBlackboardValues::Request>();

    request->key.push_back(robot_prefix_ + "stop_execution");
    request->value_type.push_back("bool");
    request->value_data.push_back("true");

    request->key.push_back(robot_prefix_ + "reset");
    request->value_type.push_back("bool");
    request->value_data.push_back("true");

    auto future = update_blackboard_client_->async_send_request(request);
    if (!update_blackboard_client_->wait_for_service(1s))
    {
        RCLCPP_WARN(this->get_logger(), "callResetProgram() => service not available yet.");
    }
}
