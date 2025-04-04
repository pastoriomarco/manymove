#ifndef MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
#define MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/blackboard.h>
#include <std_msgs/msg/string.hpp>
#include <manymove_msgs/srv/set_blackboard_values.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>

namespace manymove_cpp_trees
{

    /**
     * @brief A node that provides a single "update_blackboard" service
     *        (using manymove_msgs::srv::SetBlackboardValues)
     *        and publishes status of keys (like stop_execution, reset, collision_detected)
     *        for each robot_prefix, every 250ms.
     *
     *  All data is handled as JSON (both service input and publisher output).
     */
    class HMIServiceNode : public rclcpp::Node
    {
    public:
        explicit HMIServiceNode(const std::string &node_name,
                                BT::Blackboard::Ptr blackboard,
                                std::vector<std::string> robot_prefix = {});

    private:
        BT::Blackboard::Ptr blackboard_;
        std::vector<std::string> robot_prefix_;

        // The single service
        rclcpp::Service<manymove_msgs::srv::SetBlackboardValues>::SharedPtr update_blackboard_srv_;

        // Publisher for blackboard status
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        // Timer to publish status every 250ms
        rclcpp::TimerBase::SharedPtr status_timer_;

    private:
        // Service callback
        void handleUpdateBlackboard(
            const std::shared_ptr<manymove_msgs::srv::SetBlackboardValues::Request> request,
            std::shared_ptr<manymove_msgs::srv::SetBlackboardValues::Response> response);

        // Timer callback
        void publishBlackboardStatus();

        // Minimal JSON parse for "double_array" => e.g. "[0.01, 0.01, 0.25]"
        std::vector<double> parseJsonDoubleArray(const std::string &json_str);

        // Minimal JSON parse for pose => e.g. {"x":0.1,"y":0.2,"z":0.3,"roll":1.57,"pitch":0.0,"yaw":0.0}
        geometry_msgs::msg::Pose parseJsonPose(const std::string &json_str);
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
