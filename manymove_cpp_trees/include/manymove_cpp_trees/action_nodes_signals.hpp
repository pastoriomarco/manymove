#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_SIGNALS_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_SIGNALS_HPP

#include "manymove_cpp_trees/move.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include "manymove_planner/action/plan_manipulator.hpp"
#include "manymove_planner/action/execute_trajectory.hpp"

#include "manymove_object_manager/action/add_collision_object.hpp"
#include "manymove_object_manager/action/remove_collision_object.hpp"
#include "manymove_object_manager/action/attach_detach_object.hpp"
#include "manymove_object_manager/action/check_object_exists.hpp"
#include "manymove_object_manager/action/get_object_pose.hpp"

#include "manymove_signals/action/set_output.hpp"
#include "manymove_signals/action/get_input.hpp"
#include "manymove_signals/action/check_robot_state.hpp"
#include "manymove_signals/action/reset_robot_state.hpp"
#include "manymove_planner/action/unload_traj_controller.hpp"
#include "manymove_planner/action/load_traj_controller.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>

namespace manymove_cpp_trees
{
    /**
     * @class SetOutputAction
     * @brief Sends a goal to the "set_output" action server (manymove_signals::action::SetOutput).
     */
    class SetOutputAction : public BT::StatefulActionNode
    {
    public:
        using SetOutput = manymove_signals::action::SetOutput;
        using GoalHandleSetOutput = rclcpp_action::ClientGoalHandle<SetOutput>;

        SetOutputAction(const std::string &name,
                        const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("io_type", "IO type: 'tool' or 'controller'"),
                BT::InputPort<int>("ionum", "Which IO channel number"),
                BT::InputPort<int>("value", "Desired output value (0 or 1)"),
                BT::InputPort<std::string>("robot_prefix", "Optional robot namespace prefix, e.g. 'R_' or 'L_'."),
                BT::OutputPort<bool>("success", "Whether set_output succeeded"),
            };
        }

    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void goalResponseCallback(std::shared_ptr<GoalHandleSetOutput> goal_handle);
        void resultCallback(const GoalHandleSetOutput::WrappedResult &result);

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<SetOutput>::SharedPtr action_client_;

        bool goal_sent_;
        bool result_received_;

        std::string io_type_;
        int ionum_;
        int value_;

        SetOutput::Result action_result_;
    };

    /**
     * @class GetInputAction
     * @brief Reads a digital input from "get_input" action server (manymove_signals::action::GetInput).
     */
    class GetInputAction : public BT::StatefulActionNode
    {
    public:
        using GetInput = manymove_signals::action::GetInput;
        using GoalHandleGetInput = rclcpp_action::ClientGoalHandle<GetInput>;

        GetInputAction(const std::string &name,
                       const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("io_type", "IO type: 'tool' or 'controller'"),
                BT::InputPort<int>("ionum", "Which IO channel to read"),
                BT::InputPort<std::string>("robot_prefix", "Optional robot namespace prefix, e.g. 'R_' or 'L_'."),
                BT::OutputPort<int>("value", "Read value from the input (0 or 1)"),
                BT::OutputPort<bool>("success", "Whether get_input succeeded"),
            };
        }

    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void goalResponseCallback(std::shared_ptr<GoalHandleGetInput> goal_handle);
        void resultCallback(const GoalHandleGetInput::WrappedResult &result);

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<GetInput>::SharedPtr action_client_;

        bool goal_sent_;
        bool result_received_;

        std::string io_type_;
        int ionum_;
        int value_;

        GetInput::Result action_result_;
    };

    /**
     * @class CheckRobotStateAction
     * @brief Check the robot's current state from "check_robot_state" action.
     */
    class CheckRobotStateAction : public BT::StatefulActionNode
    {
    public:
        using CheckRobotState = manymove_signals::action::CheckRobotState;
        using GoalHandleCheckRobotState = rclcpp_action::ClientGoalHandle<CheckRobotState>;

        CheckRobotStateAction(const std::string &name,
                              const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("robot_prefix", "Optional robot namespace prefix, e.g. 'R_' or 'L_'."),
                BT::OutputPort<bool>("ready", "True if robot is ready"),
                BT::OutputPort<int>("err", "Current error code"),
                BT::OutputPort<int>("mode", "Robot mode"),
                BT::OutputPort<int>("state", "Robot state"),
                BT::OutputPort<std::string>("message", "Status message")};
        }

    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void goalResponseCallback(std::shared_ptr<GoalHandleCheckRobotState> goal_handle);
        void resultCallback(const GoalHandleCheckRobotState::WrappedResult &result);

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<CheckRobotState>::SharedPtr action_client_;

        bool goal_sent_;
        bool result_received_;

        CheckRobotState::Result action_result_;
    };

    /**
     * @class ResetRobotStateAction
     * @brief Send a goal to "reset_robot_state" (manymove_signals::action::ResetRobotState).
     */
    class ResetRobotStateAction : public BT::StatefulActionNode
    {
    public:
        using ResetRobotState = manymove_signals::action::ResetRobotState;
        using GoalHandleResetRobotState = rclcpp_action::ClientGoalHandle<ResetRobotState>;

        using UnloadTrajController = manymove_planner::action::UnloadTrajController;
        using GoalHandleUnloadTrajController = rclcpp_action::ClientGoalHandle<UnloadTrajController>;

        using LoadTrajController = manymove_planner::action::LoadTrajController;
        using GoalHandleLoadTrajController = rclcpp_action::ClientGoalHandle<LoadTrajController>;

        ResetRobotStateAction(const std::string &name,
                              const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("robot_prefix", "Optional robot namespace prefix, e.g. 'R_' or 'L_'."),
                BT::InputPort<std::string>("robot_model", "Name of the robot model, e.g. 'lite6'."),
                BT::OutputPort<bool>("success", "True if robot reset is successful"),
            };
        }

    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void goalResponseCallback(std::shared_ptr<GoalHandleResetRobotState> goal_handle);
        void resultCallback(const GoalHandleResetRobotState::WrappedResult &result);

        void goalResponseCallbackUnloadTraj(std::shared_ptr<GoalHandleUnloadTrajController> goal_handle);
        void resultCallbackUnloadTraj(const GoalHandleUnloadTrajController::WrappedResult &result);

        void goalResponseCallbackLoadTraj(std::shared_ptr<GoalHandleLoadTrajController> goal_handle);
        void resultCallbackLoadTraj(const GoalHandleLoadTrajController::WrappedResult &result);

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ResetRobotState>::SharedPtr action_client_;
        rclcpp_action::Client<UnloadTrajController>::SharedPtr unload_traj_client_;
        rclcpp_action::Client<LoadTrajController>::SharedPtr load_traj_client_;

        bool goal_sent_;
        bool result_received_;
        bool unload_traj_success_;
        bool load_traj_success_;

        bool unload_goal_sent_;
        bool reset_goal_sent_;
        bool load_goal_sent_;

        std::string computed_controller_name_;

        ResetRobotState::Result action_result_;
    };

    

} // namespace manymove_cpp_trees

#endif
