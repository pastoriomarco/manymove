#ifndef MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
#define MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <behaviortree_cpp_v3/blackboard.h>
#include <std_msgs/msg/string.hpp> 

namespace manymove_cpp_trees
{
    /**
     * @brief A node that provides HMI services and publishes the status of key blackboard values.
     *
     * It provides three services (start_execution, stop_execution, reset_program) and publishes every 250ms
     * the status of "execution_resumed", "stop_execution", and "abort_mission".
     */
    class HMIServiceNode : public rclcpp::Node
    {
    public:
        explicit HMIServiceNode(const std::string &node_name, BT::Blackboard::Ptr blackboard, std::string robot_prefix = "");

    private:
        BT::Blackboard::Ptr blackboard_;
        std::string robot_prefix_;

        // Service servers
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_execution_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_execution_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_program_srv_;

        // Publisher for blackboard status
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        // Timer to publish status every 250ms
        rclcpp::TimerBase::SharedPtr status_timer_;

        // Service callbacks
        void handle_start_execution(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void handle_stop_execution(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void handle_reset_program(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> response);

        // Timer callback: publishes the status of certain blackboard keys.
        void publishBlackboardStatus();
    };
}

#endif // MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
