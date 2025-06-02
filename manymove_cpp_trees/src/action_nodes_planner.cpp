#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

#include <memory>
#include <stdexcept>

namespace manymove_cpp_trees
{

    MoveManipulatorAction::MoveManipulatorAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Get the ROS node from the blackboard.
        if (!config.blackboard)
        {
            throw BT::RuntimeError("MoveManipulatorAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("MoveManipulatorAction: 'node' not found in blackboard.");
        }

        // Read the robot_prefix
        if (!getInput<std::string>("robot_prefix", robot_prefix_))
        {
            throw BT::RuntimeError("MoveManipulatorAction: 'robot_prefix' not found in blackboard.");
        }

        // Create the client for the MoveManipulator action server
        std::string move_server = robot_prefix_ + "move_manipulator";
        action_client_ = rclcpp_action::create_client<MoveManipulator>(node_, move_server);

        RCLCPP_INFO(node_->get_logger(), "[MoveManipulatorAction] Waiting for move_manipulator server: %s", move_server.c_str());
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("MoveManipulatorAction: move_manipulator server not available.");
        }
    }

    BT::NodeStatus MoveManipulatorAction::onStart()
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[MoveManipulatorAction] [%s]: onStart() called.",
                     name().c_str());

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = MoveManipulator::Result();

        // Read the robot_prefix
        if (!getInput<std::string>("robot_prefix", robot_prefix_))
        {
            throw BT::RuntimeError("MoveManipulatorAction: 'robot_prefix' not found in blackboard.");
        }

        // this should never be true on start, but let's leave it here for safety
        bool collision_detected;
        if (!getInput<bool>("collision_detected", collision_detected))
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "[MoveManipulatorAction] [%s]: 'collision_detected' not set => failing",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (collision_detected)
        {
            // reset the collision_detected value
            config().blackboard->set(robot_prefix_ + "collision_detected", false);
            config().blackboard->set(robot_prefix_ + "stop_execution", true);

            return BT::NodeStatus::FAILURE;
        }

        // Read move_id.
        if (!getInput<std::string>("move_id", move_id_))
        {
            RCLCPP_ERROR(node_->get_logger(), "[MoveManipulatorAction] No move_id inputPort");
            return BT::NodeStatus::FAILURE;
        }

        bool stop_execution;
        if (!getInput<bool>("stop_execution", stop_execution))
        {
            throw BT::RuntimeError("MoveManipulatorAction: '" + robot_prefix_ + "stop_execution' not found in blackboard.");
        }
        if (stop_execution)
        {
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus MoveManipulatorAction::onRunning()
    {
        // Retrieve the stored Move from the blackboard.
        std::string move_key = "move_" + move_id_;
        std::shared_ptr<Move> move_ptr;
        if (!config().blackboard->get(move_key, move_ptr))
        {
            RCLCPP_ERROR(node_->get_logger(), "[MoveManipulatorAction] Cannot find key [%s] in blackboard", move_key.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Retrieve invalidate_traj_on_exec
        bool invalidate_traj_on_exec;
        if (!getInput<bool>("invalidate_traj_on_exec", invalidate_traj_on_exec))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "MoveManipulatorAction [%s]: missing InputPort [invalidate_traj_on_exec].",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (!goal_sent_)
        {
            std::string input_pose_key;
            if (getInput<std::string>("pose_key", input_pose_key))
            {
                move_ptr->pose_key = input_pose_key;
                RCLCPP_INFO(node_->get_logger(), "[MoveManipulatorAction] Using provided pose_key: %s", input_pose_key.c_str());
            }

            // If the move is "pose" or "cartesian", retrieve the dynamic pose.
            geometry_msgs::msg::Pose dynamic_pose;
            if (move_ptr->type == "pose" || move_ptr->type == "cartesian")
            {
                if (!config().blackboard->get(move_ptr->pose_key, dynamic_pose))
                {
                    RCLCPP_ERROR(node_->get_logger(), "[MoveManipulatorAction] Failed to retrieve pose from blackboard key '%s'", move_ptr->pose_key.c_str());
                    return BT::NodeStatus::FAILURE;
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "[MoveManipulatorAction] Retrieved dynamic pose from '%s'", move_ptr->pose_key.c_str());
                }
            }

            // Read existing trajectory.
            moveit_msgs::msg::RobotTrajectory existing_trajectory;
            if (!getInput<moveit_msgs::msg::RobotTrajectory>("trajectory", existing_trajectory))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[MoveManipulatorAction] [%s]: missing InputPort [trajectory].",
                             name().c_str());
                return BT::NodeStatus::FAILURE;
            }

            // Build the goal.
            MoveManipulator::Goal goal_msg;
            manymove_msgs::msg::MoveManipulatorGoal mmg = move_ptr->to_move_manipulator_goal();
            if (move_ptr->type == "pose" || move_ptr->type == "cartesian")
                mmg.pose_target = dynamic_pose;
            goal_msg.plan_request = mmg;
            goal_msg.existing_trajectory = existing_trajectory;

            // Send the goal.
            auto send_opts = rclcpp_action::Client<MoveManipulator>::SendGoalOptions();
            send_opts.goal_response_callback = std::bind(&MoveManipulatorAction::goalResponseCallback, this, std::placeholders::_1);
            send_opts.feedback_callback = std::bind(&MoveManipulatorAction::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
            send_opts.result_callback = std::bind(&MoveManipulatorAction::resultCallback, this, std::placeholders::_1);

            action_client_->async_send_goal(goal_msg, send_opts);
            goal_sent_ = true;
        }

        // If the action result has been received, return SUCCESS or FAILURE accordingly.
        if (result_received_)
        {
            if (action_result_.success)
            {
                if (invalidate_traj_on_exec)
                {
                    config().blackboard->set("trajectory_" + move_id_, moveit_msgs::msg::RobotTrajectory());
                }
                else
                {
                    config().blackboard->set("trajectory_" + move_id_, action_result_.final_trajectory);
                }
                RCLCPP_INFO(node_->get_logger(), "[MoveManipulatorAction] success => returning SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                config().blackboard->set("trajectory_" + move_id_, moveit_msgs::msg::RobotTrajectory());
                RCLCPP_ERROR(node_->get_logger(), "[MoveManipulatorAction] failed => returning FAILURE");
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void MoveManipulatorAction::onHalted()
    {
        // Cancel the current goal if in progress.
        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;

        // Invalidate trajectory on halt
        config().blackboard->set("trajectory_" + move_id_, moveit_msgs::msg::RobotTrajectory());


    }

    void MoveManipulatorAction::goalResponseCallback(std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "[MoveManipulatorAction] Goal REJECTED by server.");
            result_received_ = true;
            action_result_.success = false;
            action_result_.message = "Rejected";
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "[MoveManipulatorAction] Goal ACCEPTED by server; waiting for result.");
        }
    }

    void MoveManipulatorAction::resultCallback(const GoalHandleMoveManipulator::WrappedResult &wrapped_result)
    {
        bool invalidate_traj_on_exec;
        getInput<bool>("invalidate_traj_on_exec", invalidate_traj_on_exec);
        if (invalidate_traj_on_exec)
        {
            // Invalidate the trajectory: set the planning validity key to false and clear the trajectory.
            config().blackboard->set("trajectory_" + move_id_, moveit_msgs::msg::RobotTrajectory());
        }

        result_received_ = true;
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            action_result_ = *(wrapped_result.result);
            config().blackboard->set("trajectory_" + move_id_, action_result_.final_trajectory);
        }
        else
        {
            action_result_.success = false;
            action_result_.message = "Failure: result code=" + std::to_string(static_cast<int>(wrapped_result.code));

            // Execution failed, invalidate the trajectory
            config().blackboard->set("trajectory_" + move_id_, moveit_msgs::msg::RobotTrajectory());
        }
    }

    void MoveManipulatorAction::feedbackCallback(std::shared_ptr<GoalHandleMoveManipulator> /*goal_handle*/,
                                                 const std::shared_ptr<const MoveManipulator::Feedback> feedback)
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[MoveManipulatorAction] feedback => progress=%.2f, in_collision=%s",
                     feedback->progress, feedback->in_collision ? "true" : "false");
        if (feedback->in_collision)
        {
            // Set collision_detected on the blackboard and cancel the goal.
            config().blackboard->set(robot_prefix_ + "collision_detected", true);

            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: Collision detected. Setting 'collision_detected' to true on blackboard.",
                        name().c_str());
        }
    }

    ResetTrajectories::ResetTrajectories(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("ResetTrajectories: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("ResetTrajectories: 'node' not found in blackboard.");
        }

        RCLCPP_INFO(node_->get_logger(),
                    "ResetTrajectories [%s]: Constructed with node [%s].",
                    name.c_str(), node_->get_fully_qualified_name());
    }

    BT::NodeStatus ResetTrajectories::tick()
    {
        // Get move_ids from input port
        std::string move_ids_str;
        if (!getInput<std::string>("move_ids", move_ids_str))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ResetTrajectories [%s]: missing InputPort [move_ids].",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Split move_ids_str by comma
        std::vector<std::string> move_ids;
        std::stringstream ss(move_ids_str);
        std::string id;
        while (std::getline(ss, id, ','))
        {
            // Trim whitespace
            id.erase(0, id.find_first_not_of(" \t"));
            id.erase(id.find_last_not_of(" \t") + 1);
            if (!id.empty())
            {
                move_ids.push_back(id);
            }
        }

        if (move_ids.empty())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "ResetTrajectories [%s]: No move_ids provided to reset.",
                        name().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        // Perform reset for each move_id
        for (const auto &move_id_str : move_ids)
        {
            try
            {
                int move_id = std::stoi(move_id_str);

                // Reset trajectory_{id} to empty
                moveit_msgs::msg::RobotTrajectory empty_traj;
                std::string traj_key = "trajectory_" + move_id_str;
                config().blackboard->set(traj_key, empty_traj);

                // Reset validity_{id} to false
                std::string validity_key = "validity_" + move_id_str;
                config().blackboard->set(validity_key, false);

                RCLCPP_DEBUG(node_->get_logger(),
                             "ResetTrajectories [%s]: Reset move_id=%d => %s cleared, %s set to false.",
                             name().c_str(), move_id, traj_key.c_str(), validity_key.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "ResetTrajectories [%s]: Invalid move_id '%s'. Exception: %s",
                             name().c_str(), move_id_str.c_str(), e.what());
                // Continue resetting other IDs
            }
        }

        return BT::NodeStatus::SUCCESS;
    }

} // namespace manymove_cpp_trees
