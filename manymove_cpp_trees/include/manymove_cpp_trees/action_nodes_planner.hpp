#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_PLANNER_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_PLANNER_HPP

#include "manymove_cpp_trees/move.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <manymove_planner/action/plan_manipulator.hpp>
#include <manymove_planner/action/execute_trajectory.hpp>

#include <manymove_object_manager/action/add_collision_object.hpp>
#include <manymove_object_manager/action/remove_collision_object.hpp>
#include <manymove_object_manager/action/attach_detach_object.hpp>
#include <manymove_object_manager/action/check_object_exists.hpp>
#include <manymove_object_manager/action/get_object_pose.hpp>

#include "manymove_signals/action/set_output.hpp"
#include "manymove_signals/action/get_input.hpp"
#include "manymove_signals/action/check_robot_state.hpp"
#include "manymove_signals/action/reset_robot_state.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>

namespace manymove_cpp_trees
{

    /**
     * @class PlanningAction
     * @brief A stateful BT node that sends a planning request to the plan_manipulator action server.
     *
     * We use StatefulActionNode to illustrate best practices:
     *   - onStart() => send goal
     *   - onRunning() => check result
     *   - onHalted() => cancel if needed
     */
    class PlanningAction : public BT::StatefulActionNode
    {
    public:
        using PlanManipulator = manymove_planner::action::PlanManipulator;
        using GoalHandlePlanManipulator = rclcpp_action::ClientGoalHandle<PlanManipulator>;

        /**
         * @brief Constructor for the PlanningAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        PlanningAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required/optional ports for this node.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("move_id", "Unique identifier for the move"),
                BT::InputPort<std::string>("pose_key", "Blackboard key to store the retrieved pose"),
                BT::InputPort<std::string>("robot_prefix", "Optional robot namespace prefix, e.g. 'R_' or 'L_'."),
                BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::OutputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::OutputPort<bool>("planning_validity", "Indicates if planning was successful"),
            };
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandlePlanManipulator> goal_handle);
        void resultCallback(const GoalHandlePlanManipulator::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<PlanManipulator>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        PlanManipulator::Result action_result_;
        std::string move_id_;  ///< Unique move identifier
        std::string pose_key_; ///< Blackboard key for the dynamic pose
    };

    /**
     * @class ExecuteTrajectory
     * @brief A stateful BT node that requests trajectory execution from an action server,
     *        implementing a polling approach if needed.
     *
     * Using StatefulActionNode:
     *   - onStart() => check if data is ready (or poll).
     *   - onRunning() => send goal if data is ready, then wait for result.
     *   - onHalted() => cancel if needed.
     */
    class ExecuteTrajectory : public BT::StatefulActionNode
    {
    public:
        using ExecuteTrajectoryAction = manymove_planner::action::ExecuteTrajectory;
        using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectoryAction>;

        ExecuteTrajectory(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::InputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::InputPort<std::string>("robot_prefix", "Optional robot namespace prefix, e.g. 'R_' or 'L_'."),
                BT::InputPort<bool>("planning_validity", "Indicates if planning was successful"),
                BT::InputPort<bool>("collision_detected", "If a collision is detected, the execution fails"),
                BT::InputPort<bool>("execution_resumed", "Signals if the execution have been resumed"),
            };
        }

    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        // Poll data from blackboard
        bool dataReady();
        void sendGoal();

        // Callbacks
        void goalResponseCallback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle);
        void resultCallback(const GoalHandleExecuteTrajectory::WrappedResult &result);
        void feedbackCallback(
            std::shared_ptr<GoalHandleExecuteTrajectory> /*goal_handle*/,
            const std::shared_ptr<const ExecuteTrajectoryAction::Feedback> feedback);

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ExecuteTrajectoryAction>::SharedPtr action_client_;
        rclcpp_action::Client<ExecuteTrajectoryAction>::SharedPtr stop_client_;

        bool goal_sent_;
        bool result_received_;

        // Polled data
        bool planning_valid_;
        std::string move_id_;
        moveit_msgs::msg::RobotTrajectory traj_;

        ExecuteTrajectoryAction::Result action_result_;

        // Timestamps for polling / timeout
        std::chrono::steady_clock::time_point wait_start_time_;
        bool is_data_ready_;

        // Pointer to the blackboard
        BT::Blackboard::Ptr blackboard_;
    };

    /**
     * @class ResetTrajectories
     * @brief A synchronous BT node that resets trajectories and their validity in the blackboard.
     *
     * It takes a comma-separated list of move_ids and for each, sets 'trajectory_{id}' to empty
     * and 'validity_{id}' to false in the blackboard.
     */
    class ResetTrajectories : public BT::SyncActionNode
    {
    public:
        /**
         * @brief Constructor for the ResetTrajectories node.
         * @param name The name of the BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        ResetTrajectories(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required/optional ports for this node.
         */
        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("move_ids", "Comma-separated list of move IDs to reset")};
        }

        /**
         * @brief Tick function that performs the reset actions.
         */
        BT::NodeStatus tick() override;

    private:
        // ROS2 node
        rclcpp::Node::SharedPtr node_;
    };

    /**
     * @class StopMotionAction
     * @brief A stateful BT node that sends a stop command to the stop_motion action server.
     *
     * This node sends an empty trajectory or a predefined stop trajectory to ensure the robot halts safely.
     */
    class StopMotionAction : public BT::StatefulActionNode
    {
    public:
        using ExecuteTrajectoryAction = manymove_planner::action::ExecuteTrajectory;
        using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectoryAction>;

        /**
         * @brief Constructor for the StopMotionAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        StopMotionAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required/optional ports for this node.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("robot_prefix", "Optional robot namespace prefix, e.g., 'R_' or 'L_'."),
                BT::InputPort<double>("deceleration_time", 0.5, "Time in seconds to decelerate to a stop")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle);
        void resultCallback(const GoalHandleExecuteTrajectory::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ExecuteTrajectoryAction>::SharedPtr stop_client_;

        // Internal state
        bool stop_goal_sent_;
        bool stop_result_received_;
        bool stop_success_;

        // Parameters
        std::string robot_prefix_;
        double deceleration_time_;
    };

} // namespace manymove_cpp_trees

#endif
