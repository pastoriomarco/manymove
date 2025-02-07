#pragma once

#include <memory>
#include <string>
#include <map>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Action Definitions
#include "manymove_msgs/action/set_output.hpp"
#include "manymove_msgs/action/get_input.hpp"
#include "manymove_msgs/action/check_robot_state.hpp"
#include "manymove_msgs/action/reset_robot_state.hpp"

// Service Definitions
#include "xarm_msgs/srv/set_digital_io.hpp"
#include "xarm_msgs/srv/get_digital_io.hpp"
#include "xarm_msgs/srv/set_int16.hpp"
#include "xarm_msgs/srv/set_int16_by_id.hpp"
#include "xarm_msgs/srv/call.hpp"

// Message Definitions
#include "xarm_msgs/msg/robot_msg.hpp"

namespace manymove_signals
{
    using SetOutput = manymove_msgs::action::SetOutput;
    using GetInput = manymove_msgs::action::GetInput;
    using ResetRobotState = manymove_msgs::action::ResetRobotState;
    using CheckRobotState = manymove_msgs::action::CheckRobotState;

    class SignalsNode : public rclcpp::Node
    {
    public:
        SignalsNode();

    private:
        // Member Variables
        std::string robot_model_;
        std::string robot_prefix_;

        // Service Clients
        std::map<std::string, rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr> set_digital_io_clients_;
        std::map<std::string, rclcpp::Client<xarm_msgs::srv::GetDigitalIO>::SharedPtr> get_digital_io_clients_;
        std::map<std::string, rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr> set_int16_clients_;

        rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr clean_error_client_;
        rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedPtr motion_enable_client_;

        // Action Servers
        rclcpp_action::Server<SetOutput>::SharedPtr set_output_server_;
        rclcpp_action::Server<GetInput>::SharedPtr get_input_server_;
        rclcpp_action::Server<ResetRobotState>::SharedPtr reset_robot_state_server_;
        rclcpp_action::Server<CheckRobotState>::SharedPtr check_robot_state_server_;

        // Callback Groups
        rclcpp::CallbackGroup::SharedPtr action_callback_group_;
        rclcpp::CallbackGroup::SharedPtr service_callback_group_;
        rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

        // Subscription
        rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_subscription_;

        // Robot State
        xarm_msgs::msg::RobotMsg::SharedPtr current_robot_state_;
        std::mutex robot_state_mutex_;

        // Initialization Methods
        void initialize_service_clients();
        bool wait_for_services();
        void initialize_action_servers();
        void initialize_robot_state_subscription();

        // Callback Methods
        void robot_state_callback(const xarm_msgs::msg::RobotMsg::SharedPtr msg);

        // SetOutput Action Callbacks
        rclcpp_action::GoalResponse handle_set_output_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const SetOutput::Goal> goal);
        rclcpp_action::CancelResponse handle_set_output_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetOutput>> goal_handle);
        void execute_set_output(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetOutput>> goal_handle);

        // GetInput Action Callbacks
        rclcpp_action::GoalResponse handle_get_input_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const GetInput::Goal> goal);
        rclcpp_action::CancelResponse handle_get_input_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetInput>> goal_handle);
        void execute_get_input(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetInput>> goal_handle);

        // ResetRobotState Action Callbacks
        rclcpp_action::GoalResponse handle_reset_robot_state_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ResetRobotState::Goal> goal);
        rclcpp_action::CancelResponse handle_reset_robot_state_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle);
        void execute_reset_robot_state(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle);

        //  CheckRobotState Callbacks
        rclcpp_action::GoalResponse handle_check_robot_state_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CheckRobotState::Goal> goal);
        rclcpp_action::CancelResponse handle_check_robot_state_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckRobotState>> goal_handle);
        void execute_check_robot_state(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckRobotState>> goal_handle);

        // Helper functions:
        void enable_motion(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
            std::shared_ptr<ResetRobotState::Result> result);
        void set_mode(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
            std::shared_ptr<ResetRobotState::Result> result);
        void set_state(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
            std::shared_ptr<ResetRobotState::Result> result);
        void verify_reset(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
            std::shared_ptr<ResetRobotState::Result> result);
    };

} // namespace manymove_signals
