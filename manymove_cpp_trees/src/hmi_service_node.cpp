#include "manymove_cpp_trees/hmi_service_node.hpp"
#include "manymove_cpp_trees/tree_helper.hpp" // for createPoseRPY
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

namespace manymove_cpp_trees
{

    HMIServiceNode::HMIServiceNode(const std::string &node_name,
                                   BT::Blackboard::Ptr blackboard,
                                   std::vector<std::string> robot_prefix)
        : Node(node_name),
          blackboard_(blackboard),
          robot_prefix_(robot_prefix)
    {
        // Create a single "update_blackboard" service.
        update_blackboard_srv_ =
            this->create_service<manymove_msgs::srv::SetBlackboardValues>(
                "update_blackboard",
                std::bind(&HMIServiceNode::handleUpdateBlackboard,
                          this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
                    "[%s] HMIServiceNode started: service 'update_blackboard' is ready.",
                    node_name.c_str());

        // Create a publisher for blackboard status
        publisher_ = this->create_publisher<std_msgs::msg::String>("blackboard_status", 10);

        // Timer => publish status every 250ms
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

        // We'll try to update each item in a loop
        for (size_t i = 0; i < n; i++)
        {
            std::string key = request->key[i];
            std::string type = request->value_type[i];
            std::string data = request->value_data[i]; // supposed to be JSON

            RCLCPP_INFO(this->get_logger(),
                        "Updating BB key='%s' type='%s' data='%s'",
                        key.c_str(), type.c_str(), data.c_str());

            try
            {
                if (type == "bool")
                {
                    // Expect "true" or "false"
                    bool val = (data.find("true") != std::string::npos);
                    blackboard_->set(key, val);
                }
                else if (type == "double")
                {
                    // parse numeric from JSON string, e.g. "3.14"
                    double d = std::stod(data);
                    blackboard_->set(key, d);
                }
                else if (type == "string")
                {
                    // Right now we store data as-is.
                    // If you want real JSON string parse, remove quotes, etc.
                    blackboard_->set(key, data);
                }
                else if (type == "double_array")
                {
                    // Expect something like "[1.0,2.0,3.0]"
                    std::vector<double> arr = parseJsonDoubleArray(data);
                    blackboard_->set(key, arr);
                }
                else if (type == "pose")
                {
                    // Expect something like: {"x":0.1,"y":0.2,"z":0.3,"roll":1.57,"pitch":0.0,"yaw":3.14}
                    geometry_msgs::msg::Pose pose = parseJsonPose(data);
                    blackboard_->set(key, pose);
                }
                else
                {
                    throw std::runtime_error("Unsupported value_type: " + type);
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

    void HMIServiceNode::publishBlackboardStatus()
    {
        // We want JSON that includes info for each robot in robot_prefix_.

        // If no robot_prefix => we assume a single "global" set of keys
        //   (like "stop_execution", "reset", "collision_detected").
        // If multiple => we publish a JSON object with an entry for each prefix.

        std_msgs::msg::String msg;

        if (robot_prefix_.empty())
        {
            // No robot prefixes => use unprefixed Blackboard keys
            bool stop_execution = false;
            bool reset = false;
            bool collision_detected = false;

            blackboard_->get("stop_execution", stop_execution);
            blackboard_->get("reset", reset);
            blackboard_->get("collision_detected", collision_detected);

            std::ostringstream ss;
            ss << "{"
               << "\"stop_execution\": " << (stop_execution ? "true" : "false") << ", "
               << "\"reset\" :" << (reset ? "true" : "false") << ", "
               << "\"collision_detected\": " << (collision_detected ? "true" : "false")
               << "}";
            msg.data = ss.str();
        }
        else
        {
            // One or more prefixes => build a single JSON object with multiple fields
            std::ostringstream ss;
            ss << "{";

            bool first_item = true;
            for (size_t i = 0; i < robot_prefix_.size(); i++)
            {
                const std::string &prefix = robot_prefix_[i];

                bool stop_execution = false;
                bool reset = false;
                bool collision_detected = false;

                blackboard_->get(prefix + "stop_execution", stop_execution);
                blackboard_->get(prefix + "reset", reset);
                blackboard_->get(prefix + "collision_detected", collision_detected);

                // Insert a comma before all but the first item
                if (!first_item)
                {
                    ss << ", ";
                }
                first_item = false;

                // We produce fields named e.g. "robot1stop_execution"
                // If prefix is empty, that results in e.g. "stop_execution" anyway
                ss << "\"" << prefix << "stop_execution\": " << (stop_execution ? "true" : "false") << ", "
                   << "\"" << prefix << "reset\": " << (reset ? "true" : "false") << ", "
                   << "\"" << prefix << "collision_detected\": " << (collision_detected ? "true" : "false");
            }

            ss << "}";
            msg.data = ss.str();
        }

        publisher_->publish(msg);
    }

    /**
     * @brief parseJsonDoubleArray
     *   Minimal approach: expects a string like "[0.1,2.3,4.5]"
     *   We'll strip brackets, then split on commas, then std::stod each piece.
     *   If you need robust JSON, use a library.
     */
    std::vector<double> HMIServiceNode::parseJsonDoubleArray(const std::string &json_str)
    {
        // strip leading/trailing whitespace
        auto start = json_str.find_first_not_of(" \t\r\n");
        auto end = json_str.find_last_not_of(" \t\r\n");
        if (start == std::string::npos || end == std::string::npos)
        {
            throw std::runtime_error("Empty array string");
        }
        std::string s = json_str.substr(start, end - start + 1);

        // Expect it starts with '[' and ends with ']'
        if (s.front() != '[' || s.back() != ']')
        {
            throw std::runtime_error("parseJsonDoubleArray: no surrounding brackets");
        }
        // remove the brackets
        s = s.substr(1, s.size() - 2);

        // now split on commas
        std::vector<double> result;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, ','))
        {
            // trim
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

    /**
     * @brief parseJsonPose
     *   Minimal approach: expects e.g.
     *     {"x":1.0,"y":2.0,"z":3.0,"roll":0.0,"pitch":1.57,"yaw":0.0}
     *   We'll do simple substring searches. For robust usage => use a JSON library.
     */
    geometry_msgs::msg::Pose HMIServiceNode::parseJsonPose(const std::string &json_str)
    {
        // This is just a quick hack: we find e.g. x:..., y:..., z:..., roll:..., etc
        // For robust code => use a real parser
        auto findValue = [&](const std::string &label) -> double
        {
            // e.g. label="\"x\":" or "x":
            auto pos = json_str.find(label);
            if (pos == std::string::npos)
            {
                throw std::runtime_error("Missing field: " + label + " in pose");
            }
            // find next colon
            auto colon_pos = json_str.find(":", pos + label.size());
            if (colon_pos == std::string::npos)
            {
                throw std::runtime_error("Missing colon after " + label);
            }
            // read until comma or end brace
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
            // trim
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
