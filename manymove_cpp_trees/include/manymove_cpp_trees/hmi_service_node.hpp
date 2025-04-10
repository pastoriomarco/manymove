#ifndef MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
#define MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/blackboard.h>
#include <std_msgs/msg/string.hpp>
#include <manymove_msgs/srv/set_blackboard_values.hpp>
#include "manymove_cpp_trees/tree_helper.hpp" // for createPoseRPY
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>

namespace manymove_cpp_trees
{
    // Structure representing a blackboard key.
    struct BlackboardEntry
    {
        std::string key;
        std::string value_type; // e.g. "bool", "double", "string", "pose", etc.
    };

    /**
     * @brief A node that provides a single "update_blackboard" service
     *        (using manymove_msgs::srv::SetBlackboardValues) and publishes
     *        the status of keys every 250ms.
     *
     * All data is handled as JSON. The publishBlackboardStatus() method iterates
     * over a vector of BlackboardEntry structures to build the JSON message.
     */
    class HMIServiceNode : public rclcpp::Node
    {
    public:
        // Constructor: pass the complete vector of keys.
        explicit HMIServiceNode(const std::string &node_name,
                                BT::Blackboard::Ptr blackboard,
                                std::vector<BlackboardEntry> keys = {});

    private:
        BT::Blackboard::Ptr blackboard_;
        std::vector<BlackboardEntry> keys_;

        // The update_blackboard service.
        rclcpp::Service<manymove_msgs::srv::SetBlackboardValues>::SharedPtr update_blackboard_srv_;

        // Publisher for blackboard status.
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        // Timer to publish status every 250ms.
        rclcpp::TimerBase::SharedPtr status_timer_;

        // Service callback.
        void handleUpdateBlackboard(
            const std::shared_ptr<manymove_msgs::srv::SetBlackboardValues::Request> request,
            std::shared_ptr<manymove_msgs::srv::SetBlackboardValues::Response> response);

        std::string serializePoseRPY(const geometry_msgs::msg::Pose &pose);

        // Timer callback.
        void publishBlackboardStatus();

        // Minimal JSON parse for "double_array" (e.g. "[0.01, 0.01, 0.25]").
        std::vector<double> parseJsonDoubleArray(const std::string &json_str);

        // Minimal JSON parse for pose (e.g. {"x":0.1,"y":0.2,"z":0.3,"roll":1.57,"pitch":0.0,"yaw":0.0}).
        geometry_msgs::msg::Pose parseJsonPose(const std::string &json_str);
    };

    template <class T>
    void defineVariableKey(const rclcpp::Node::SharedPtr &node_ptr,
                               BT::Blackboard::Ptr blackboard,
                               std::vector<manymove_cpp_trees::BlackboardEntry> &keys,
                               const std::string &key_name,
                               const std::string &key_type,
                               T &value)
    {
        // Set the blackboard key with initial value
        blackboard->set(key_name, value);

        // These keys need to be published for the HMI, adding them here:
        keys.push_back({key_name, key_type});

        RCLCPP_INFO(node_ptr->get_logger(),
                    "%s key added to 'blackboard_status' service.",
                    key_name.c_str());
    }

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
