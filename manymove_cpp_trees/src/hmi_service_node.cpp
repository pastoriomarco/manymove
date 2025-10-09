#include "manymove_cpp_trees/hmi_service_node.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#  include <tf2/LinearMath/Quaternion.hpp>
#else
#  include <tf2/LinearMath/Quaternion.h>
#endif
#include <tf2/LinearMath/Matrix3x3.h>

namespace manymove_cpp_trees
{

    HMIServiceNode::HMIServiceNode(const std::string &node_name,
                                   BT::Blackboard::Ptr blackboard,
                                   std::vector<BlackboardEntry> keys)
        : Node(node_name),
          blackboard_(blackboard),
          keys_(keys)
    {
        // Create the update_blackboard service.
        update_blackboard_srv_ =
            this->create_service<manymove_msgs::srv::SetBlackboardValues>(
                "update_blackboard",
                std::bind(&HMIServiceNode::handleUpdateBlackboard,
                          this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
                    "[%s] HMIServiceNode started: service 'update_blackboard' is ready.",
                    node_name.c_str());

        // Create the publisher for blackboard status.
        publisher_ = this->create_publisher<std_msgs::msg::String>("blackboard_status", 10);

        // Timer to publish status every 250ms.
        using namespace std::chrono_literals;
        status_timer_ = this->create_wall_timer(
            250ms, std::bind(&HMIServiceNode::publishBlackboardStatus, this));
    }

    void HMIServiceNode::handleUpdateBlackboard(
        const std::shared_ptr<manymove_msgs::srv::SetBlackboardValues::Request> request,
        std::shared_ptr<manymove_msgs::srv::SetBlackboardValues::Response> response)
    {
        size_t n = request->key.size();
        if (request->value_type.size() != n || request->value_data.size() != n)
        {
            response->success = false;
            response->message = "Mismatched array lengths among key, value_type, value_data";
            RCLCPP_ERROR(this->get_logger(), "update_blackboard: array lengths differ!");
            return;
        }

        // Update each blackboard key.
        for (size_t i = 0; i < n; i++)
        {
            std::string key = request->key[i];
            std::string type = request->value_type[i];
            std::string data = request->value_data[i];

            RCLCPP_INFO(this->get_logger(),
                        "Updating BB key='%s' type='%s' data='%s'",
                        key.c_str(), type.c_str(), data.c_str());

            try
            {
                if (type == "bool")
                {
                    bool val = (data.find("true") != std::string::npos);
                    blackboard_->set(key, val);
                }
                else if (type == "double")
                {
                    double d = std::stod(data);
                    blackboard_->set(key, d);
                }
                else if (type == "string")
                {
                    blackboard_->set(key, data);
                }
                else if (type == "double_array")
                {
                    std::vector<double> arr = parseJsonDoubleArray(data);
                    blackboard_->set(key, arr);
                }
                else if (type == "pose")
                {
                    geometry_msgs::msg::Pose pose = parseJsonPose(data);
                    blackboard_->set(key, pose);
                }
                else
                {
                    throw std::runtime_error("Unsupported value_type or wrong nomenclature: " + type);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Error updating key='%s': %s", key.c_str(), e.what());
            }
        }

        response->success = true;
        response->message = "Updated " + std::to_string(n) + " blackboard keys";
    }

    std::string HMIServiceNode::serializePoseRPY(const geometry_msgs::msg::Pose &pose)
    {
        // Convert quaternion to roll, pitch, yaw.
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Build a JSON-like string for the pose.
        std::ostringstream oss;
        oss << "{\"x\":" << pose.position.x
            << ", \"y\":" << pose.position.y
            << ", \"z\":" << pose.position.z
            << ", \"roll\":" << roll
            << ", \"pitch\":" << pitch
            << ", \"yaw\":" << yaw << "}";
        return oss.str();
    }

    void HMIServiceNode::publishBlackboardStatus()
    {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << "{";
        bool first_item = true;
        for (const auto &entry : keys_)
        {
            if (!first_item)
                ss << ", ";
            first_item = false;

            // For each key, check its value_type and retrieve accordingly.
            if (entry.value_type == "bool")
            {
                bool b = false;
                if (blackboard_->get(entry.key, b))
                    ss << "\"" << entry.key << "\": " << (b ? "true" : "false");
                else
                    ss << "\"" << entry.key << "\": \"\"";
            }
            else if (entry.value_type == "double")
            {
                double d = 0.0;
                if (blackboard_->get(entry.key, d))
                    ss << "\"" << entry.key << "\": " << d;
                else
                    ss << "\"" << entry.key << "\": \"\"";
            }
            else if (entry.value_type == "string")
            {
                std::string s;
                if (blackboard_->get(entry.key, s))
                    ss << "\"" << entry.key << "\": \"" << s << "\"";
                else
                    ss << "\"" << entry.key << "\": \"\"";
            }
            else if (entry.value_type == "pose")
            {
                geometry_msgs::msg::Pose pose;
                if (blackboard_->get(entry.key, pose))
                {
                    // Use the helper function to convert the Pose to a JSON string with RPY.
                    std::string poseStr = serializePoseRPY(pose);
                    ss << "\"" << entry.key << "\": " << poseStr;
                }
                else
                {
                    ss << "\"" << entry.key << "\": \"\"";
                }
            }
            else if (entry.value_type == "double_array")
            {
                std::vector<double> arr;
                if (blackboard_->get(entry.key, arr))
                {
                    ss << "\"" << entry.key << "\": [";
                    bool first = true;
                    for (auto &v : arr)
                    {
                        if (!first)
                            ss << ", ";
                        first = false;
                        ss << v;
                    }
                    ss << "]";
                }
                else
                {
                    ss << "\"" << entry.key << "\": \"\"";
                }
            }
            else
            {
                // Fallback: try to retrieve as a string.
                std::string s;
                if (blackboard_->get(entry.key, s))
                    ss << "\"" << entry.key << "\": \"" << s << "\"";
                else
                    ss << "\"" << entry.key << "\": \"\"";
            }
        }
        ss << "}";
        msg.data = ss.str();
        publisher_->publish(msg);
    }

    std::vector<double> HMIServiceNode::parseJsonDoubleArray(const std::string &json_str)
    {
        auto start = json_str.find_first_not_of(" \t\r\n");
        auto end = json_str.find_last_not_of(" \t\r\n");
        if (start == std::string::npos || end == std::string::npos)
        {
            throw std::runtime_error("Empty array string");
        }
        std::string s = json_str.substr(start, end - start + 1);
        if (s.front() != '[' || s.back() != ']')
        {
            throw std::runtime_error("parseJsonDoubleArray: no surrounding brackets");
        }
        s = s.substr(1, s.size() - 2);
        std::vector<double> result;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, ','))
        {
            auto st = item.find_first_not_of(" \t\r\n");
            auto en = item.find_last_not_of(" \t\r\n");
            if (st == std::string::npos)
                continue;
            std::string piece = item.substr(st, en - st + 1);
            try
            {
                result.push_back(std::stod(piece));
            }
            catch (...)
            {
                throw std::runtime_error("parseJsonDoubleArray: invalid numeric in '" + piece + "'");
            }
        }
        return result;
    }

    geometry_msgs::msg::Pose HMIServiceNode::parseJsonPose(const std::string &json_str)
    {
        auto findValue = [&](const std::string &label) -> double
        {
            auto pos = json_str.find(label);
            if (pos == std::string::npos)
            {
                throw std::runtime_error("Missing field: " + label + " in pose");
            }
            auto colon_pos = json_str.find(":", pos + label.size());
            if (colon_pos == std::string::npos)
            {
                throw std::runtime_error("Missing colon after " + label);
            }
            auto comma_pos = json_str.find(",", colon_pos + 1);
            auto brace_pos = json_str.find("}", colon_pos + 1);
            size_t endpos;
            if (comma_pos == std::string::npos && brace_pos == std::string::npos)
            {
                throw std::runtime_error("Cannot find end of numeric for " + label);
            }
            else if (comma_pos == std::string::npos)
            {
                endpos = brace_pos;
            }
            else if (brace_pos == std::string::npos)
            {
                endpos = comma_pos;
            }
            else
            {
                endpos = std::min(comma_pos, brace_pos);
            }
            std::string val_str = json_str.substr(colon_pos + 1, endpos - (colon_pos + 1));
            auto st = val_str.find_first_not_of(" \t\r\n");
            auto en = val_str.find_last_not_of(" \t\r\n");
            if (st == std::string::npos)
                throw std::runtime_error("Empty numeric for " + label);
            val_str = val_str.substr(st, en - st + 1);
            return std::stod(val_str);
        };

        geometry_msgs::msg::Pose pose;
        double x = findValue("\"x\"");
        double y = findValue("\"y\"");
        double z = findValue("\"z\"");
        double roll = findValue("\"roll\"");
        double pitch = findValue("\"pitch\"");
        double yaw = findValue("\"yaw\"");
        pose = createPoseRPY(x, y, z, roll, pitch, yaw);
        return pose;
    }

} // namespace manymove_cpp_trees
