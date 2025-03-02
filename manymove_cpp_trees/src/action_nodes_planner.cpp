#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

#include <memory>
#include <stdexcept>

namespace manymove_cpp_trees
{

    PlanningAction::PlanningAction(const std::string &name,
                                   const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("PlanningAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("PlanningAction: 'node' not found in blackboard.");
        }

        // Retrive the robot_prefix
        std::string prefix;
        if (!getInput<std::string>("robot_prefix", prefix))
        {
            // if no robot_prefix provided, default to empty
            prefix = "";
        }
        std::string server_name = prefix + "plan_manipulator";

        // Initialize the action client
        RCLCPP_INFO(node_->get_logger(),
                    "PlanningAction [%s]: waiting up to 5s for '%s' server...",
                    name.c_str(), server_name.c_str());
        action_client_ = rclcpp_action::create_client<PlanManipulator>(node_, server_name);

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("PlanningAction: server " + server_name + " not available after 5s.");
        }
        RCLCPP_INFO(node_->get_logger(),
                    "PlanningAction [%s]: Connected to server [%s].",
                    name.c_str(), server_name.c_str());
    }

    BT::NodeStatus PlanningAction::onStart()
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "PlanningAction [%s]: onStart() called.", name().c_str());

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = PlanManipulator::Result();

        // Retrieve move_id from input port
        if (!getInput<std::string>("move_id", move_id_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: missing InputPort [move_id].",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Build blackboard key => "move_{move_id}"
        std::string move_key = "move_" + move_id_;
        std::shared_ptr<Move> move_ptr;
        if (!config().blackboard->get(move_key, move_ptr))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: can't find key [%s] in blackboard.",
                         name().c_str(), move_key.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Retrieve the input "robot_prefix" for the current plan
        std::string input_robot_prefix;
        if (!getInput<std::string>("robot_prefix", input_robot_prefix))
        {
            input_robot_prefix = "";
        }

        // This is the prefix actually stored in the move
        std::string stored_move_prefix = move_ptr->robot_prefix;

        // Check if move prefix exists:
        if ((!stored_move_prefix.empty()) && (stored_move_prefix != input_robot_prefix))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Move robot prefix not found or prefix mismatch: Aborting plan.");
            return BT::NodeStatus::FAILURE;
        }

        // **Begin: Set start_joint_values based on previous move**
        try
        {
            bool sequential_from_previous;
            if (!getInput<bool>("sequential_from_previous", sequential_from_previous))
            {

                RCLCPP_ERROR(node_->get_logger(),
                             "PlanningAction [%s]: Key sequential_from_previous not found", name().c_str());

                return BT::NodeStatus::FAILURE;
            }

            int current_move_id_int = std::stoi(move_id_);
            if (sequential_from_previous)
            {
                int previous_move_id_int = current_move_id_int - 1;

                // 1) Check validity_{prevID}
                std::string validity_key = "validity_" + std::to_string(previous_move_id_int);
                bool prev_validity = false;

                if (config().blackboard->get(validity_key, prev_validity) && prev_validity)
                {
                    // 2) Check "move_{prevID}" to ensure same robot prefix
                    std::string prev_move_key = "move_" + std::to_string(previous_move_id_int);
                    std::shared_ptr<Move> prev_move_ptr;
                    if (config().blackboard->get(prev_move_key, prev_move_ptr))
                    {
                        // If the prefixes match, THEN copy
                        if (prev_move_ptr->robot_prefix == move_ptr->robot_prefix)
                        {
                            // 3) Check if trajectory_{prevID} is non-empty
                            std::string trajectory_key = "trajectory_" + std::to_string(previous_move_id_int);
                            moveit_msgs::msg::RobotTrajectory previous_traj;
                            if (config().blackboard->get(trajectory_key, previous_traj) &&
                                !previous_traj.joint_trajectory.points.empty())
                            {
                                // Copy last waypoint
                                const auto &last_point = previous_traj.joint_trajectory.points.back();
                                move_ptr->start_joint_values = last_point.positions;

                                RCLCPP_INFO(node_->get_logger(),
                                            "PlanningAction [%s]: Copied last positions from move_%d => start_joint_values. (prefix=%s)",
                                            name().c_str(), previous_move_id_int,
                                            move_ptr->robot_prefix.c_str());
                            }
                            else
                            {
                                // no trajectory or empty => skip
                                move_ptr->start_joint_values.clear();
                                RCLCPP_WARN(node_->get_logger(),
                                            "PlanningAction [%s]: Previous trajectory '%s' missing or empty. Using empty start_joint_values.",
                                            name().c_str(), trajectory_key.c_str());
                            }
                        }
                        else
                        {
                            // Different robot => skip
                            move_ptr->start_joint_values.clear();
                            RCLCPP_INFO(node_->get_logger(),
                                        "PlanningAction [%s]: Not copying from move_%d => different prefix (%s vs %s).",
                                        name().c_str(), previous_move_id_int,
                                        prev_move_ptr->robot_prefix.c_str(),
                                        move_ptr->robot_prefix.c_str());
                        }
                    }
                    else
                    {
                        // can't find the previous move struct => skip
                        move_ptr->start_joint_values.clear();
                    }
                }
                else
                {
                    // invalid => skip
                    move_ptr->start_joint_values.clear();
                }
            }
            else
            {
                // first move => no previous
                move_ptr->start_joint_values.clear();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: Error parsing move_id '%s': %s. Using empty joint state.",
                         name().c_str(), move_id_.c_str(), e.what());
            move_ptr->start_joint_values.clear();
        }
        // **End: Set start_joint_values based on previous move**

        geometry_msgs::msg::Pose dynamic_pose;
        manymove_msgs::msg::MoveManipulatorGoal move_goal;

        // Assign Move to goal
        move_goal = move_ptr->to_move_manipulator_goal();

        // **Begin: Retrieve dynamic pose using pose_key**
        if (move_ptr->type == "pose" || move_ptr->type == "cartesian")
        {
            pose_key_ = move_ptr->pose_key;
            if (!config().blackboard->get(pose_key_, dynamic_pose))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "PlanningAction [%s]: Failed to retrieve pose from blackboard key '%s'.",
                             name().c_str(), pose_key_.c_str());
                return BT::NodeStatus::FAILURE;
            }

            // Log the retrieved pose
            RCLCPP_DEBUG(node_->get_logger(),
                         "PlanningAction [%s]: Retrieved pose from '%s' - Position (%.3f, %.3f, %.3f), Orientation (%.3f, %.3f, %.3f, %.3f)",
                         name().c_str(), pose_key_.c_str(),
                         dynamic_pose.position.x, dynamic_pose.position.y, dynamic_pose.position.z,
                         dynamic_pose.orientation.x, dynamic_pose.orientation.y, dynamic_pose.orientation.z, dynamic_pose.orientation.w);

            // Assign the dynamic pose to the goal
            move_goal.pose_target = dynamic_pose;

            RCLCPP_INFO(node_->get_logger(),
                        "PlanningAction [%s]: Final move_goal.pose_target set to Position (%.3f, %.3f, %.3f), Orientation (%.3f, %.3f, %.3f, %.3f)",
                        name().c_str(),
                        move_goal.pose_target.position.x, move_goal.pose_target.position.y, move_goal.pose_target.position.z,
                        move_goal.pose_target.orientation.x, move_goal.pose_target.orientation.y, move_goal.pose_target.orientation.z, move_goal.pose_target.orientation.w);
        }
        // **End: Retrieve dynamic pose using pose_key**

        RCLCPP_INFO(node_->get_logger(),
                    "PlanningAction [%s]: sending plan goal => move_id=%s",
                    name().c_str(), move_id_.c_str());

        // Assign goal message to send
        PlanManipulator::Goal goal_msg;
        goal_msg.goal = move_goal;

        // Send goal
        auto send_goal_options = rclcpp_action::Client<PlanManipulator>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&PlanningAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&PlanningAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus PlanningAction::onRunning()
    {
        if (result_received_)
        {
            if (action_result_.success)
            {
                // Set outputs
                setOutput("planned_move_id", move_id_);
                setOutput("trajectory", action_result_.trajectory);
                setOutput("planning_validity", true);

                RCLCPP_INFO(node_->get_logger(),
                            "PlanningAction [%s]: SUCCESS => planning_validity=true",
                            name().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                // --- PLANNING FAILED ---
                setOutput("planning_validity", false);
                // --- STOPPING EXECUTION ---
                config().blackboard->set("stop_execution", true);

                RCLCPP_ERROR(node_->get_logger(),
                             "PlanningAction [%s]: FAIL => planning_validity=false",
                             name().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // still waiting for action result
        return BT::NodeStatus::RUNNING;
    }

    void PlanningAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "PlanningAction [%s]: onHalted => cancel goal if needed.",
                    name().c_str());

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;
    }

    void PlanningAction::goalResponseCallback(std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: Goal REJECTED by server.",
                         name().c_str());
            PlanManipulator::Result fail;
            fail.success = false;
            action_result_ = fail;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "PlanningAction [%s]: Goal ACCEPTED by server.",
                        name().c_str());
        }
    }

    void PlanningAction::resultCallback(const GoalHandlePlanManipulator::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "PlanningAction [%s]: Plan action => SUCCEEDED.",
                        name().c_str());
            action_result_ = *(wrapped_result.result);
            result_received_ = true;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: Plan action => FAILED code=%d",
                         name().c_str(), static_cast<int>(wrapped_result.code));
            PlanManipulator::Result fail;
            fail.success = false;
            action_result_ = fail;
            result_received_ = true;
        }
    }

    ExecuteTrajectory::ExecuteTrajectory(const std::string &name,
                                         const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false),
          is_data_ready_(false)
    {
        if (!config.blackboard)
        {
            throw BT::RuntimeError(
                "ExecuteTrajectory constructor: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError(
                "ExecuteTrajectory constructor: 'node' not found in blackboard.");
        }

        std::string prefix;
        if (!getInput<std::string>("robot_prefix", prefix))
        {
            prefix = "";
        }

        // build server names with prefix
        std::string exec_server = prefix + "execute_manipulator_traj";
        std::string stop_server = prefix + "stop_motion";

        action_client_ = rclcpp_action::create_client<ExecuteTrajectoryAction>(node_, exec_server);
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: waiting 5s for '%s' server...",
                    name.c_str(), exec_server.c_str());
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("ExecuteTrajectory: server " + exec_server + " not available after 5s.");
        }

        /**
         * The stop_motion action servers takes as input any traj and just stops the motion of the manipulator
         * by overriding the current trajectory execution by traj_controller with the current position,
         * zero velocity, and deceleration time. The robot will try to "spring back" to the position it was
         * when the stop command is issued within the deceleration time. The higher the time, the smoother
         * the stop, but the higher the move lenght to decelerate and come back to the stop point.
         * At current time, the deceleration_time is hardcoded in the manymove_planner in execute_stop() function
         * inside the action_server implementation.
         */
        stop_client_ = rclcpp_action::create_client<ExecuteTrajectoryAction>(node_, stop_server);
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: waiting 5s for '%s' server...",
                    name.c_str(), stop_server.c_str());
        if (!stop_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("ExecuteTrajectory: server " + stop_server + " not available after 5s.");
        }

        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Constructed with node [%s], "
                    "connected to servers [%s] and [%s].",
                    name.c_str(), node_->get_fully_qualified_name(),
                    exec_server.c_str(), stop_server.c_str());
    }

    BT::NodeStatus ExecuteTrajectory::onStart()
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "ExecuteTrajectory [%s]: onStart() called.",
                     name().c_str());

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = ExecuteTrajectoryAction::Result();
        is_data_ready_ = false;

        bool collision_detected;
        if (!getInput<bool>("collision_detected", collision_detected))
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "ExecuteTrajectory [%s]: 'collision_detected' not set => failing",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (collision_detected)
        {
            // reset the collision_detected value
            config().blackboard->set("collision_detected", false);

            return BT::NodeStatus::FAILURE;
        }

        // Start polling
        wait_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(node_->get_logger(),
                     "ExecuteTrajectory [%s]: Polling for 'planned_move_id', 'trajectory', 'planning_validity'...",
                     name().c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ExecuteTrajectory::onRunning()
    {

        // Retrieve execution_resumed
        bool execution_resumed;
        if (!getInput<bool>("execution_resumed", execution_resumed))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "StopMotionAction [%s]: missing InputPort [execution_resumed].",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Retrieve invalidate_traj_on_exec
        bool invalidate_traj_on_exec;
        if (!getInput<bool>("invalidate_traj_on_exec", invalidate_traj_on_exec))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "StopMotionAction [%s]: missing InputPort [invalidate_traj_on_exec].",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Retrieve robot_prefix
        std::string robot_prefix_;
        if (!getInput<std::string>("robot_prefix", robot_prefix_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "StopMotionAction [%s]: missing InputPort [robot_prefix].",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Check if execution_resumed is true getting the blackboard key value.
        if (execution_resumed)
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "ExecuteTrajectory [%s]: execution_resumed is true.", name().c_str());

            // If we already sent a goal and are waiting for a result, cancel the goal immediately.
            if (goal_sent_ && !result_received_)
            {
                RCLCPP_DEBUG(node_->get_logger(),
                             "ExecuteTrajectory [%s]: Cancelling current goal due to execution_resumed.", name().c_str());
                action_client_->async_cancel_all_goals();
            }

            // Invalidate the trajectory: set the planning validity key to false and clear the trajectory.
            setOutput("trajectory", moveit_msgs::msg::RobotTrajectory());
            setOutput("planning_validity", false);

            // Clear the execution_resumed key so that it does not keep triggering.
            config().blackboard->set(robot_prefix_ + "execution_resumed", false);

            RCLCPP_DEBUG(node_->get_logger(),
                         "ExecuteTrajectory [%s]: Forcing failure due to execution_resumed; key cleared.",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Normal polling for plan data.
        if (!is_data_ready_)
        {
            if (!dataReady())
            {
                if (!invalidate_traj_on_exec)
                {
                    // If we don't invalidate traj on exex, the data should always be ready but on the first move!
                    // IF it's not, we need to plan a new traj, so we fail to pass the ball to the fallback planning node.
                    return BT::NodeStatus::FAILURE;
                }

                auto elapsed = std::chrono::steady_clock::now() - wait_start_time_;
                /// TODO: the planning should fail before this timeout, must check what actually happens if it doesn't
                int max_time = 64;
                if (elapsed > std::chrono::seconds(max_time))
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "ExecuteTrajectory [%s]: Timed out (%is) waiting for plan data.",
                                 name().c_str(), max_time);
                    return BT::NodeStatus::FAILURE;
                }
                else
                {
                    RCLCPP_DEBUG(node_->get_logger(),
                                 "ExecuteTrajectory [%s]: Still polling => RUNNING",
                                 name().c_str());
                    return BT::NodeStatus::RUNNING;
                }
            }
            else
            {
                sendGoal();
                is_data_ready_ = true;
                return BT::NodeStatus::RUNNING;
            }
        }

        if (result_received_)
        {
            if (invalidate_traj_on_exec)
            {
                // Invalidate the trajectory: set the planning validity key to false and clear the trajectory.
                setOutput("trajectory", moveit_msgs::msg::RobotTrajectory());
                setOutput("planning_validity", false);
            }

            if (action_result_.success)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "ExecuteTrajectory [%s]: Execution SUCCEEDED => SUCCESS.",
                            name().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "ExecuteTrajectory [%s]: Execution FAILED => FAILURE.",
                             name().c_str());

                // Invalidate the trajectory after execution if it failed, regardless of the value of invalidate_traj_on_exec.
                setOutput("trajectory", moveit_msgs::msg::RobotTrajectory());
                setOutput("planning_validity", false);

                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING; // Still waiting for final result.
    }

    void ExecuteTrajectory::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "ExecuteTrajectory [%s]: onHalted => cancel goal if needed.",
                    name().c_str());

        // Cancel any in-progress "execute_manipulator_traj" goal
        // if (goal_sent_ && !result_received_)
        // {
        //     action_client_->async_cancel_all_goals();
        // }
        goal_sent_ = false;
        result_received_ = false;
        is_data_ready_ = false;

        // Invalidate trajectory on halt
        setOutput("trajectory", moveit_msgs::msg::RobotTrajectory());
        setOutput("planning_validity", false);

        // *** Call STOP_MOTION server ***
        if (!stop_client_)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "ExecuteTrajectory [%s]: stop_motion_client_ is not initialized. Cannot stop motion.",
                        name().c_str());
            return;
        }

        // Create an empty goal (we won't send a real trajectory)
        manymove_msgs::action::ExecuteTrajectory::Goal stop_goal;
        // e.g. stop_goal.trajectory is empty

        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Sending STOP goal to 'stop_motion' server...",
                    name().c_str());

        auto send_goal_options = rclcpp_action::Client<manymove_msgs::action::ExecuteTrajectory>::SendGoalOptions();

        // Add result callback to see if STOP completed
        send_goal_options.result_callback =
            [this](const typename rclcpp_action::ClientGoalHandle<
                   manymove_msgs::action::ExecuteTrajectory>::WrappedResult &wrapped_result)
        {
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(node_->get_logger(), "StopMotion => SUCCEEDED: %s",
                            wrapped_result.result->message.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "StopMotion => code=%d => message: %s",
                            static_cast<int>(wrapped_result.code),
                            wrapped_result.result ? wrapped_result.result->message.c_str() : "");
            }
        };

        // Send goal asynchronously
        stop_client_->async_send_goal(stop_goal, send_goal_options);
    }

    bool ExecuteTrajectory::dataReady()
    {
        // Attempt to read planned_move_id
        std::string pmid;
        if (!getInput<std::string>("planned_move_id", pmid) || pmid.empty())
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "ExecuteTrajectory [%s]: 'planned_move_id' not set => poll again",
                         name().c_str());
            return false;
        }
        move_id_ = pmid;

        // Attempt to read planning_validity
        bool validity = false;
        if (!getInput<bool>("planning_validity", validity) || !validity)
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "ExecuteTrajectory [%s]: planning_validity=false => poll again",
                         name().c_str());
            return false;
        }
        planning_valid_ = validity;

        // Attempt to read trajectory
        moveit_msgs::msg::RobotTrajectory t;
        if (!getInput<moveit_msgs::msg::RobotTrajectory>("trajectory", t) ||
            t.joint_trajectory.points.empty())
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "ExecuteTrajectory [%s]: 'trajectory' missing/empty => poll again",
                         name().c_str());
            return false;
        }
        traj_ = t;

        // If all are present, we are ready
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Data is ready => planned_move_id=%s, valid=%s, %zu pts",
                    name().c_str(), move_id_.c_str(), (planning_valid_ ? "true" : "false"),
                    traj_.joint_trajectory.points.size());
        return true;
    }

    void ExecuteTrajectory::sendGoal()
    {
        if (goal_sent_)
        {
            return;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Sending ExecuteTrajectory goal => move_id=%s",
                    name().c_str(), move_id_.c_str());

        ExecuteTrajectoryAction::Goal goal_msg;
        goal_msg.trajectory = traj_;

        auto opts = rclcpp_action::Client<ExecuteTrajectoryAction>::SendGoalOptions();
        opts.goal_response_callback =
            std::bind(&ExecuteTrajectory::goalResponseCallback, this, std::placeholders::_1);
        opts.result_callback =
            std::bind(&ExecuteTrajectory::resultCallback, this, std::placeholders::_1);

        opts.feedback_callback =
            std::bind(&ExecuteTrajectory::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

        action_client_->async_send_goal(goal_msg, opts);
        goal_sent_ = true;
    }

    void ExecuteTrajectory::goalResponseCallback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ExecuteTrajectory [%s]: Goal REJECTED by server.",
                         name().c_str());
            ExecuteTrajectoryAction::Result fail;
            fail.success = false;
            action_result_ = fail;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: Goal ACCEPTED by server => waiting for result.",
                        name().c_str());
        }
    }

    void ExecuteTrajectory::resultCallback(const GoalHandleExecuteTrajectory::WrappedResult &wrapped_result)
    {
        bool invalidate_traj_on_exec;
        getInput<bool>("invalidate_traj_on_exec", invalidate_traj_on_exec);
        if (invalidate_traj_on_exec)
        {
            // Invalidate the trajectory: set the planning validity key to false and clear the trajectory.
            setOutput("trajectory", moveit_msgs::msg::RobotTrajectory());
            setOutput("planning_validity", false);
        }

        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: Execution => SUCCEEDED.",
                        name().c_str());
            action_result_ = *(wrapped_result.result);
            result_received_ = true;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ExecuteTrajectory [%s]: Execution => code=%d => FAIL.",
                         name().c_str(), static_cast<int>(wrapped_result.code));
            ExecuteTrajectoryAction::Result fail;
            fail.success = false;
            fail.message = "Execution failed.";
            action_result_ = fail;
            result_received_ = true;

            // Execution failed, invalidate the trajectory
            setOutput("trajectory", moveit_msgs::msg::RobotTrajectory());
            setOutput("planning_validity", false);
        }
    }

    void ExecuteTrajectory::feedbackCallback(
        std::shared_ptr<GoalHandleExecuteTrajectory> /*goal_handle*/,
        const std::shared_ptr<const ExecuteTrajectoryAction::Feedback> feedback)
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "ExecuteTrajectory [%s]: Received feedback: progress = %f, in_collision = %s",
                     name().c_str(),
                     feedback->progress,
                     feedback->in_collision ? "true" : "false");

        // If the feedback indicates a collision, set the blackboard key "stop_execution" to true.
        if (feedback->in_collision)
        {
            // Set collision_detected to true
            config().blackboard->set("collision_detected", true);

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
        for (const auto &mid_str : move_ids)
        {
            try
            {
                int mid = std::stoi(mid_str);

                // Reset trajectory_{id} to empty
                moveit_msgs::msg::RobotTrajectory empty_traj;
                std::string traj_key = "trajectory_" + mid_str;
                config().blackboard->set(traj_key, empty_traj);

                // Reset validity_{id} to false
                std::string validity_key = "validity_" + mid_str;
                config().blackboard->set(validity_key, false);

                RCLCPP_DEBUG(node_->get_logger(),
                             "ResetTrajectories [%s]: Reset move_id=%d => %s cleared, %s set to false.",
                             name().c_str(), mid, traj_key.c_str(), validity_key.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "ResetTrajectories [%s]: Invalid move_id '%s'. Exception: %s",
                             name().c_str(), mid_str.c_str(), e.what());
                // Continue resetting other IDs
            }
        }

        return BT::NodeStatus::SUCCESS;
    }

    // ------------------------------------------------------------------
    // StopMotionAction
    // ------------------------------------------------------------------

    StopMotionAction::StopMotionAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          stop_goal_sent_(false),
          stop_result_received_(false),
          stop_success_(false),
          deceleration_time_(0.5) // Default value
    {
        // Retrieve the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("StopMotionAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("StopMotionAction: 'node' not found in blackboard.");
        }

        // Retrieve robot_prefix
        if (!getInput<std::string>("robot_prefix", robot_prefix_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "StopMotionAction [%s]: missing InputPort [robot_prefix].",
                         robot_prefix_.c_str());
            throw BT::RuntimeError("StopMotionAction: missing robot_prefix InputPort.");
        }

        std::string stop_server_name = robot_prefix_ + "stop_motion";

        // Create action client for stop_motion
        stop_client_ = rclcpp_action::create_client<ExecuteTrajectoryAction>(node_, stop_server_name);

        RCLCPP_INFO(node_->get_logger(),
                    "StopMotionAction [%s]: waiting up to 5s for server '%s'...",
                    name.c_str(), stop_server_name.c_str());

        if (!stop_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("StopMotionAction: server '" + stop_server_name + "' not available after 5s.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "StopMotionAction [%s]: Connected to server '%s'.",
                        name.c_str(), stop_server_name.c_str());
        }

        // Retrieve deceleration_time if provided
        getInput<double>("deceleration_time", deceleration_time_);
    }

    BT::NodeStatus StopMotionAction::onStart()
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "StopMotionAction [%s]: onStart() called.",
                     name().c_str());

        stop_goal_sent_ = false;
        stop_result_received_ = false;
        stop_success_ = false;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus StopMotionAction::onRunning()
    {
        if (!stop_goal_sent_)
        {
            // Build a stop goal
            ExecuteTrajectoryAction::Goal stop_goal;
            /**
             * Given the implementation for StopMotion.action in manymove_planner, we send an empty
             * trajectory and the planner will automatically construct a stop motion trajectory to
             * send to the joint trajectory controller. The new trajectory will preempt the current
             * trajectory being executed, and its relative action will fail. Any trajectory we send
             * to the StopMotion action server will be ignored anyway and the stop trajectory will be
             * constructed as implemented in manymove_planner.
             */
            // Send the goal
            auto send_goal_options = rclcpp_action::Client<ExecuteTrajectoryAction>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&StopMotionAction::goalResponseCallback, this, std::placeholders::_1);
            send_goal_options.result_callback =
                std::bind(&StopMotionAction::resultCallback, this, std::placeholders::_1);

            stop_client_->async_send_goal(stop_goal, send_goal_options);
            stop_goal_sent_ = true;

            RCLCPP_INFO(node_->get_logger(),
                        "StopMotionAction [%s]: STOP goal sent. Waiting for result...",
                        name().c_str());
        }

        if (stop_result_received_)
        {
            if (stop_success_)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "StopMotionAction [%s]: STOP succeeded => SUCCESS.",
                            name().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "StopMotionAction [%s]: STOP failed => FAILURE.",
                             name().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // Still waiting for the result
        return BT::NodeStatus::RUNNING;
    }

    void StopMotionAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "StopMotionAction [%s]: onHalted => cancel goal if needed.",
                    name().c_str());

        if (stop_goal_sent_ && !stop_result_received_)
        {
            stop_client_->async_cancel_all_goals();
            RCLCPP_WARN(node_->get_logger(),
                        "StopMotionAction [%s]: STOP goal canceled.",
                        name().c_str());
        }

        stop_goal_sent_ = false;
        stop_result_received_ = false;
        stop_success_ = false;
    }

    void StopMotionAction::goalResponseCallback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "StopMotionAction [%s]: STOP goal was rejected by the server.",
                         name().c_str());
            stop_success_ = false;
            stop_result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "StopMotionAction [%s]: STOP goal was accepted by the server.",
                        name().c_str());
        }
    }

    void StopMotionAction::resultCallback(const GoalHandleExecuteTrajectory::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "StopMotionAction [%s]: STOP command succeeded. Message: %s",
                        name().c_str(),
                        wrapped_result.result->message.c_str());
            stop_success_ = true;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "StopMotionAction [%s]: STOP command failed with code %d. Message: %s",
                         name().c_str(),
                         static_cast<int>(wrapped_result.code),
                         wrapped_result.result ? wrapped_result.result->message.c_str() : "No message");
            stop_success_ = false;
        }

        stop_result_received_ = true;
    }

} // namespace manymove_cpp_trees
