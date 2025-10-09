// action_server.hpp
#ifndef MANYMOVE_PLANNER_ACTION_SERVER_HPP
#define MANYMOVE_PLANNER_ACTION_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include "manymove_msgs/action/plan_manipulator.hpp"
#include "manymove_msgs/action/move_manipulator.hpp"
#include "manymove_msgs/action/unload_traj_controller.hpp"
#include "manymove_msgs/action/load_traj_controller.hpp"
#include "planner_interface.hpp"
#include <moveit_msgs/msg/constraints.hpp>
#include "manymove_planner/compat/moveit_includes_compat.hpp"

enum class MoveExecutionState
{
    IDLE,
    PLANNING,
    EXECUTING,
    COMPLETED,
    CANCELLED
};

class ManipulatorActionServer
{
public:
    explicit ManipulatorActionServer(
        const rclcpp::Node::SharedPtr &node,
        const std::shared_ptr<PlannerInterface> &planner,
        const std::string &planner_prefix = "");

private:
    // Internal types
    using MoveManipulator = manymove_msgs::action::MoveManipulator;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;

    using UnloadTrajController = manymove_msgs::action::UnloadTrajController;
    using GoalHandleUnloadTrajController = rclcpp_action::ServerGoalHandle<UnloadTrajController>;

    using LoadTrajController = manymove_msgs::action::LoadTrajController;
    using GoalHandleLoadTrajController = rclcpp_action::ServerGoalHandle<LoadTrajController>;

    // ROS Node and Planner Interface
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PlannerInterface> planner_;
    std::string planner_prefix_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;
    rclcpp::CallbackGroup::SharedPtr param_callback_group_;

    // Action Servers
    rclcpp_action::Server<MoveManipulator>::SharedPtr move_manipulator_server_;
    rclcpp_action::Server<UnloadTrajController>::SharedPtr unload_traj_controller_server_;
    rclcpp_action::Server<LoadTrajController>::SharedPtr load_traj_controller_server_;

    // Service Clients
    rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client_;

    // Joint States
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    std::mutex joint_states_mutex_;
    std::unordered_map<std::string, double> current_joint_positions_;

    std::mutex move_state_mutex_;
    MoveExecutionState move_state_{MoveExecutionState::IDLE};

    moveit_msgs::msg::RobotTrajectory executing_traj_;
    rclcpp::Time executing_start_time_;
    manymove_msgs::msg::MoveManipulatorGoal move_manipulator_goal_;

    // MoveManipulator Callbacks
    rclcpp_action::GoalResponse handle_move_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveManipulator::Goal>);
    rclcpp_action::CancelResponse handle_move_cancel(const std::shared_ptr<GoalHandleMoveManipulator>);
    void handle_move_accepted(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle);
    void execute_move(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle);

    // UnloadTrajController Callbacks
    rclcpp_action::GoalResponse handle_unload_traj_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const UnloadTrajController::Goal>);
    rclcpp_action::CancelResponse handle_unload_traj_cancel(const std::shared_ptr<GoalHandleUnloadTrajController>);
    void handle_unload_traj_accepted(const std::shared_ptr<GoalHandleUnloadTrajController>);
    void execute_unload_traj_controller(const std::shared_ptr<GoalHandleUnloadTrajController> &);

    // LoadTrajController Callbacks
    rclcpp_action::GoalResponse handle_load_traj_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const LoadTrajController::Goal>);
    rclcpp_action::CancelResponse handle_load_traj_cancel(const std::shared_ptr<GoalHandleLoadTrajController>);
    void handle_load_traj_accepted(const std::shared_ptr<GoalHandleLoadTrajController>);
    void execute_load_traj_controller(const std::shared_ptr<GoalHandleLoadTrajController> &);

    // Async controller management helpers
    void unloadControllerAsync(const std::string &, std::function<void()>, std::function<void(const std::string &)>);
    void loadControllerAsync(const std::string &, std::function<void()>, std::function<void(const std::string &)>);
    void activateControllerAsync(const std::string &, std::function<void()>, std::function<void(const std::string &)>);
    void deactivateControllerAsync(const std::string &, std::function<void()>, std::function<void(const std::string &)>);
    void configureControllerAsync(const std::string &, std::function<void()>, std::function<void(const std::string &)>);

    // Helper functions
    bool executeTrajectoryWithCollisionChecks(
        const std::shared_ptr<GoalHandleMoveManipulator> &goal_handle,
        const moveit_msgs::msg::RobotTrajectory &traj,
        std::string &abort_reason);
};

#endif // MANYMOVE_PLANNER_ACTION_SERVER_HPP
