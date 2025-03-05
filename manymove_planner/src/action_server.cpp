#include "manymove_planner/planner_interface.hpp"
#include "manymove_msgs/action/plan_manipulator.hpp"
#include "manymove_msgs/action/execute_trajectory.hpp"
#include "manymove_msgs/action/unload_traj_controller.hpp"
#include "manymove_msgs/action/load_traj_controller.hpp"

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>

class ManipulatorActionServer
{
public:
    using PlanManipulator = manymove_msgs::action::PlanManipulator;
    using GoalHandlePlanManipulator = rclcpp_action::ServerGoalHandle<PlanManipulator>;

    using ExecuteTrajectory = manymove_msgs::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    using UnloadTrajController = manymove_msgs::action::UnloadTrajController;
    using GoalHandleUnloadTrajController = rclcpp_action::ServerGoalHandle<UnloadTrajController>;

    using LoadTrajController = manymove_msgs::action::LoadTrajController;
    using GoalHandleLoadTrajController = rclcpp_action::ServerGoalHandle<LoadTrajController>;

    ManipulatorActionServer(const rclcpp::Node::SharedPtr &node,
                                const std::shared_ptr<PlannerInterface> &planner,
                                const std::string &planner_prefix = "")
        : node_(node), planner_(planner), planner_prefix_(planner_prefix)
    {
        // Create Reentrant Callback Groups
        action_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        param_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Initialize Service Clients
        unload_controller_client_ = node_->create_client<controller_manager_msgs::srv::UnloadController>(
            "/controller_manager/unload_controller",
            rmw_qos_profile_services_default,
            param_callback_group_);

        load_controller_client_ = node_->create_client<controller_manager_msgs::srv::LoadController>(
            "/controller_manager/load_controller",
            rmw_qos_profile_services_default,
            param_callback_group_);

        switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller",
            rmw_qos_profile_services_default,
            param_callback_group_);

        configure_controller_client_ = node_->create_client<controller_manager_msgs::srv::ConfigureController>(
            "/controller_manager/configure_controller",
            rmw_qos_profile_services_default,
            param_callback_group_);

        // **Wait for Services to Be Available**
        bool all_services_available = true;

        if (!unload_controller_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '/controller_manager/unload_controller' not available.");
            all_services_available = false;
        }

        if (!load_controller_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '/controller_manager/load_controller' not available.");
            all_services_available = false;
        }

        if (!switch_controller_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '/controller_manager/switch_controller' not available.");
            all_services_available = false;
        }

        if (!configure_controller_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '/controller_manager/configure_controller' not available.");
            all_services_available = false;
        }

        if (!all_services_available)
        {
            RCLCPP_ERROR(node_->get_logger(), "Not all required services are available. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Subscribe to /joint_states to let ExecuteTrajectory handle the collision check feedback
        joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(), // or rclcpp::SystemDefaultsQoS(), etc.
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(joint_states_mutex_);
                // Update map of joint_name → position
                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    // Guard against out-of-range indexing if position array is shorter than name array
                    if (i < msg->position.size())
                    {
                        current_joint_positions_[msg->name[i]] = msg->position[i];
                    }
                }
            });

        plan_action_server_ = rclcpp_action::create_server<PlanManipulator>(
            node_,
            planner_prefix_ + "plan_manipulator",
            std::bind(&ManipulatorActionServer::handle_plan_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ManipulatorActionServer::handle_plan_cancel, this, std::placeholders::_1),
            std::bind(&ManipulatorActionServer::handle_plan_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        execute_action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            node_,
            planner_prefix_ + "execute_manipulator_traj",
            std::bind(&ManipulatorActionServer::handle_execute_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ManipulatorActionServer::handle_execute_cancel, this, std::placeholders::_1),
            std::bind(&ManipulatorActionServer::handle_execute_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        stop_motion_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            node_,
            planner_prefix_ + "stop_motion",
            std::bind(&ManipulatorActionServer::handle_stop_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ManipulatorActionServer::handle_stop_cancel, this, std::placeholders::_1),
            std::bind(&ManipulatorActionServer::handle_stop_accept, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        unload_traj_controller_server_ = rclcpp_action::create_server<UnloadTrajController>(
            node_,
            planner_prefix_ + "unload_trajectory_controller",
            std::bind(&ManipulatorActionServer::handle_unload_traj_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ManipulatorActionServer::handle_unload_traj_cancel, this, std::placeholders::_1),
            std::bind(&ManipulatorActionServer::handle_unload_traj_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        load_traj_controller_server_ = rclcpp_action::create_server<LoadTrajController>(
            node_,
            planner_prefix_ + "load_trajectory_controller",
            std::bind(&ManipulatorActionServer::handle_load_traj_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ManipulatorActionServer::handle_load_traj_cancel, this, std::placeholders::_1),
            std::bind(&ManipulatorActionServer::handle_load_traj_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PlannerInterface> planner_;
    std::string planner_prefix_;

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;
    rclcpp::CallbackGroup::SharedPtr param_callback_group_;

    // Action Servers
    rclcpp_action::Server<PlanManipulator>::SharedPtr plan_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr execute_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr stop_motion_server_;
    rclcpp_action::Server<UnloadTrajController>::SharedPtr unload_traj_controller_server_;
    rclcpp_action::Server<LoadTrajController>::SharedPtr load_traj_controller_server_;

    // Service Clients
    rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client_;

    // Joint States Subscriber to let the ExecuteTrajectory function handle the collision check feedback
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    std::mutex joint_states_mutex_;
    std::unordered_map<std::string, double> current_joint_positions_;

    // -------------------------------------
    // PlanManipulator callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_plan_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        [[maybe_unused]] std::shared_ptr<const PlanManipulator::Goal> goal_msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Received PlanManipulator action goal request");
        // Optionally validate goal_msg->goal ...
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_plan_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel PlanManipulator goal");
        // For a pure planning action, you might accept or reject cancellation
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_plan_accepted(
        const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        // Spin off a new thread to do planning so we don't block the executor
        std::thread{std::bind(&ManipulatorActionServer::execute_plan_goal, this, goal_handle)}.detach();
    }

    void execute_plan_goal(const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        auto result = std::make_shared<PlanManipulator::Result>();
        const auto &goal_msg = goal_handle->get_goal();

        // 1) Plan the trajectory
        manymove_msgs::action::PlanManipulator::Goal internal_goal;
        internal_goal.goal = goal_msg->goal;

        auto [success, trajectory] = planner_->plan(internal_goal);
        if (!success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 2) Call the applyTimeParameterization
        auto [param_ok, timed_trajectory] =
            planner_->applyTimeParameterization(trajectory, goal_msg->goal.config);
        if (!param_ok)
        {
            RCLCPP_ERROR(node_->get_logger(), "Time param failed");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 3) Set the final result
        result->success = true;
        result->trajectory = timed_trajectory;
        goal_handle->succeed(result);
    }

    // -------------------------------------
    // ExecuteTrajectory callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_execute_goal(
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const manymove_msgs::action::ExecuteTrajectory::Goal> goal)
    {
        if (goal->trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Received an empty trajectory");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(node_->get_logger(), "Received ExecuteTrajectory goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_execute_cancel(
        const std::shared_ptr<GoalHandleExecuteTrajectory> /*goal_handle*/)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel trajectory execution");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_execute_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        // Run in a separate thread.
        std::thread{[this, goal_handle]()
                    {
                        RCLCPP_INFO(node_->get_logger(), "Executing trajectory");
                        auto result = std::make_shared<manymove_msgs::action::ExecuteTrajectory::Result>();
                        const auto &goal_msg = goal_handle->get_goal();
                        moveit_msgs::msg::RobotTrajectory traj = goal_msg->trajectory;
                        const auto &points = traj.joint_trajectory.points;

                        if (points.empty())
                        {
                            RCLCPP_ERROR(node_->get_logger(), "Empty trajectory received");
                            result->success = false;
                            result->message = "Empty trajectory";
                            goal_handle->abort(result);
                            return;
                        }

                        std::vector<double> current_joint_state;
                        {
                            // Lock joint_states_mutex_ to safely read current_joint_positions_
                            std::lock_guard<std::mutex> lock(joint_states_mutex_);
                            // Extract joint values in the order of the trajectory's joint_names
                            for (const auto &joint_name : traj.joint_trajectory.joint_names)
                            {
                                auto it = current_joint_positions_.find(joint_name);
                                if (it != current_joint_positions_.end())
                                {
                                    current_joint_state.push_back(it->second);
                                }
                                else
                                {
                                    RCLCPP_ERROR(node_->get_logger(), "Current joint state for %s not found", joint_name.c_str());
                                    result->success = false;
                                    result->message = "Incomplete current joint state";
                                    goal_handle->abort(result);
                                    return;
                                }
                            }
                        }

                        double tolerance = 0.05; // tolerance in radians
                        if (!planner_->isTrajectoryStartValid(traj, current_joint_state, tolerance))
                        {
                            RCLCPP_ERROR(node_->get_logger(), "Trajectory start is not within tolerance of current joint state");
                            result->success = false;
                            result->message = "Trajectory start mismatch";
                            goal_handle->abort(result);
                            return;
                        }

                        // Full trajectory state validity check BEFORE execution.
                        for (size_t i = 0; i < points.size(); ++i)
                        {
                            if (!planner_->isJointStateValid(points[i].positions))
                            {
                                RCLCPP_ERROR(node_->get_logger(), "Trajectory point %zu is invalid", i);
                                result->success = false;
                                result->message = "Trajectory execution aborted: invalid state at point " + std::to_string(i);
                                goal_handle->abort(result);
                                return;
                            }
                        }

                        // Prepare a promise and a flag to track any collision during execution.
                        auto result_promise = std::make_shared<std::promise<bool>>();
                        std::future<bool> result_future = result_promise->get_future();
                        std::atomic<bool> collision_detected(false);

                        // Set up FollowJointTrajectory send goal options.
                        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions options;

                        // Feedback callback: check upcoming waypoints.
                        options.feedback_callback =
                            [this, points, goal_handle, &collision_detected, last_idx = size_t(0)](auto /*unused_goal_handle*/,
                                                                                                   const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> &feedback) mutable
                        {
                            // 1) Get the robot's current actual joint positions from the feedback
                            const auto &actual_positions = feedback->actual.positions;

                            if (actual_positions.empty())
                            {
                                RCLCPP_ERROR(node_->get_logger(),
                                             "[ActionServer] Feedback has no positions. Aborting execution.");

                                collision_detected = true; // or some other flag
                                // publish feedback so the client knows we are aborting
                                auto exec_feedback = std::make_shared<manymove_msgs::action::ExecuteTrajectory::Feedback>();
                                exec_feedback->progress = -1.0f;
                                exec_feedback->in_collision = true;
                                goal_handle->publish_feedback(exec_feedback);

                                // Then simply return. Your result_callback will see "collision_detected=true" and abort.
                                return;
                            }

                            // 2) Find the new closest index by scanning from 'last_idx' forward
                            //    until the distance starts to increase again.
                            size_t start_idx = last_idx;
                            double last_dist = std::numeric_limits<double>::infinity();
                            size_t closest_idx = start_idx;

                            for (size_t i = start_idx; i < points.size(); ++i)
                            {
                                // Compute distance between feedback->actual.positions and points[i].positions
                                double sum_sq = 0.0;
                                for (size_t j = 0;
                                     j < actual_positions.size() && j < points[i].positions.size();
                                     ++j)
                                {
                                    double diff = actual_positions[j] - points[i].positions[j];
                                    sum_sq += diff * diff;
                                }
                                double dist = std::sqrt(sum_sq);

                                if (dist <= last_dist)
                                {
                                    // We are still getting “closer”
                                    last_dist = dist;
                                    closest_idx = i;
                                }
                                else
                                {
                                    // The distance is now larger than the previous iteration,
                                    // so the previous i was the best match. Stop searching.
                                    break;
                                }
                            }

                            // Update the last_idx for next time
                            last_idx = closest_idx;

                            // 3) Decide how far forward you want to collision-check.
                            // Checking to the end of the traj:
                            size_t check_limit = points.size() - 1;
                            // As an alternative, we can check only the next 10 points to reduce CPU usage:
                            // size_t check_limit = std::min(closest_idx + 10, points.size() - 1);

                            // 4) Check collisions in the next few waypoints
                            for (size_t i = closest_idx + 1; i <= check_limit; ++i)
                            {
                                if (!planner_->isJointStateValid(points[i].positions))
                                {
                                    RCLCPP_WARN(node_->get_logger(),
                                                "[ActionServer] Future waypoint %zu is in collision", i);
                                    collision_detected = true;
                                    break;
                                }
                            }

                            // 5) Publish action feedback
                            auto exec_feedback =
                                std::make_shared<manymove_msgs::action::ExecuteTrajectory::Feedback>();
                            // We no longer rely on controller time, so we can store e.g. the index or distance
                            exec_feedback->progress = static_cast<float>(closest_idx);
                            exec_feedback->in_collision = collision_detected.load();
                            goal_handle->publish_feedback(exec_feedback);
                        };

                        // Result callback.
                        options.result_callback =
                            [this, result_promise, &collision_detected](const auto &wrapped_result)
                        {
                            bool success = false;
                            if (collision_detected)
                            {
                                RCLCPP_ERROR(node_->get_logger(), "[ActionServer] Aborting execution due to collision");
                                success = false;
                            }
                            else
                            {
                                switch (wrapped_result.code)
                                {
                                case rclcpp_action::ResultCode::SUCCEEDED:
                                    RCLCPP_INFO(node_->get_logger(), "[ActionServer] FollowJointTrajectory succeeded");
                                    success = true;
                                    break;
                                case rclcpp_action::ResultCode::ABORTED:
                                    RCLCPP_ERROR(node_->get_logger(), "[ActionServer] FollowJointTrajectory aborted");
                                    success = false;
                                    break;
                                case rclcpp_action::ResultCode::CANCELED:
                                    RCLCPP_WARN(node_->get_logger(), "[ActionServer] FollowJointTrajectory canceled");
                                    success = false;
                                    break;
                                default:
                                    RCLCPP_ERROR(node_->get_logger(), "[ActionServer] Unknown result code");
                                    success = false;
                                    break;
                                }
                            }
                            result_promise->set_value(success);
                        };

                        // Prepare the FollowJointTrajectory goal.
                        control_msgs::action::FollowJointTrajectory::Goal exec_goal;
                        exec_goal.trajectory = traj.joint_trajectory;

                        // Get the action client from the planner.
                        auto client = planner_->getFollowJointTrajClient();
                        if (!client)
                        {
                            RCLCPP_ERROR(node_->get_logger(), "[ActionServer] FollowJointTrajectory client is null");
                            result->success = false;
                            result->message = "Internal error: No action client";
                            goal_handle->abort(result);
                            return;
                        }

                        // Send the goal.
                        auto goal_handle_future = client->async_send_goal(exec_goal, options);
                        if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
                        {
                            RCLCPP_ERROR(node_->get_logger(), "[ActionServer] Timed out while sending the goal");
                            result->success = false;
                            result->message = "Timeout obtaining goal handle";
                            goal_handle->abort(result);
                            return;
                        }
                        auto client_goal_handle = goal_handle_future.get();
                        if (!client_goal_handle)
                        {
                            RCLCPP_ERROR(node_->get_logger(), "[ActionServer] Goal was rejected by the controller");
                            result->success = false;
                            result->message = "Goal rejected";
                            goal_handle->abort(result);
                            return;
                        }

                        RCLCPP_INFO(node_->get_logger(), "[ActionServer] Waiting for FollowJointTrajectory result...");
                        auto res_future = client->async_get_result(client_goal_handle);
                        if (res_future.wait_for(std::chrono::seconds(300)) != std::future_status::ready)
                        {
                            RCLCPP_ERROR(node_->get_logger(), "[ActionServer] Trajectory execution timed out");
                            result->success = false;
                            result->message = "Execution timeout";
                            goal_handle->abort(result);
                            return;
                        }

                        // Retrieve the final status from the result callback.
                        bool final_status = result_future.get();
                        if (final_status)
                        {
                            RCLCPP_INFO(node_->get_logger(), "[ActionServer] Trajectory execution succeeded");
                            result->success = true;
                            result->message = "Trajectory executed successfully";
                            goal_handle->succeed(result);
                        }
                        else
                        {
                            RCLCPP_ERROR(node_->get_logger(), "[ActionServer] Trajectory execution failed (collision or controller error)");
                            result->success = false;
                            result->message = "Execution failed (collision or controller error)";
                            goal_handle->abort(result);
                        }
                    }}
            .detach();
    }

    // -------------------------------------
    // StopMotion callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_stop_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        [[maybe_unused]] std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Received request to STOP");
        // Always accept the request:
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_stop_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Received request to cancel STOP motion");
        // Typically we just accept the cancel, though there's not much sense in "canceling a stop."
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_stop_accept(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        // Execute in a separate thread so as not to block the executor
        std::thread{std::bind(&ManipulatorActionServer::execute_stop, this, goal_handle)}.detach();
    }

    void execute_stop(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Executing STOP motion...");
        auto result = std::make_shared<ExecuteTrajectory::Result>();

        // 1) Check if canceled before we start
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(node_->get_logger(), "[ExecuteTrajectory] STOP goal canceled before execution.");
            result->success = false;
            result->message = "Canceled before execution";
            goal_handle->canceled(result);
            return;
        }

        // 2) Actually send the short stop trajectory
        double dec_time = 1.0;
        bool ok = planner_->sendControlledStop(dec_time);

        // 3) If canceled mid-stop
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(node_->get_logger(), "[ExecuteTrajectory] STOP goal canceled mid‐execution.");
            result->success = false;
            result->message = "Canceled mid‐execution";
            goal_handle->canceled(result);
            return;
        }

        // 4) Finalize
        if (ok)
        {
            RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] STOP completed successfully.");
            result->success = true;
            result->message = "Stop complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "[ExecuteTrajectory] STOP motion failed or was aborted by the controller.");
            result->success = false;
            result->message = "Stop motion failed";
            goal_handle->abort(result);
        }
    }

    // -------------------------------------
    // UnloadTrajController callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_unload_traj_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const UnloadTrajController::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received UnloadTrajController goal request for controller: %s",
                    goal->controller_name.c_str());

        if (goal->controller_name.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Controller name is empty. Rejecting.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_unload_traj_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleUnloadTrajController> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received request to CANCEL UnloadTrajController goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_unload_traj_accepted(
        const std::shared_ptr<GoalHandleUnloadTrajController> goal_handle)
    {
        std::thread{std::bind(&ManipulatorActionServer::execute_unload_traj_controller, this, goal_handle)}.detach();
    }

    void execute_unload_traj_controller(
        const std::shared_ptr<GoalHandleUnloadTrajController> &goal_handle)
    {
        auto result = std::make_shared<UnloadTrajController::Result>();
        std::string controller_name = goal_handle->get_goal()->controller_name;

        // 1) Deactivate the controller
        deactivateControllerAsync(
            controller_name,
            [this, goal_handle, result, controller_name]()
            {
                // 2) Unload the controller
                unloadControllerAsync(
                    controller_name,
                    [this, goal_handle, result, controller_name]()
                    {
                        result->success = true;
                        result->message = "Controller unloaded successfully.";
                        goal_handle->succeed(result);
                    },
                    [goal_handle, result, controller_name](const std::string &err)
                    {
                        result->success = false;
                        result->message = "Unload error: " + err;
                        goal_handle->abort(result);
                    });
            },
            [goal_handle, result, controller_name](const std::string &err)
            {
                result->success = false;
                result->message = "Deactivate error: " + err;
                goal_handle->abort(result);
            });
    }

    // -------------------------------------
    // LoadTrajController callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_load_traj_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const LoadTrajController::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received LoadTrajController goal request for controller: %s",
                    goal->controller_name.c_str());

        if (goal->controller_name.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Controller name is empty. Rejecting.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_load_traj_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleLoadTrajController> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received request to CANCEL LoadTrajController goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_load_traj_accepted(
        const std::shared_ptr<GoalHandleLoadTrajController> goal_handle)
    {
        std::thread{std::bind(&ManipulatorActionServer::execute_load_traj_controller, this, goal_handle)}.detach();
    }

    void execute_load_traj_controller(
        const std::shared_ptr<GoalHandleLoadTrajController> &goal_handle)
    {
        auto result = std::make_shared<LoadTrajController::Result>();
        std::string controller_name = goal_handle->get_goal()->controller_name;

        // 1) Load the controller
        loadControllerAsync(
            controller_name,
            [this, goal_handle, result, controller_name]()
            {
                // 2) Configure the controller
                configureControllerAsync(
                    controller_name,
                    [this, goal_handle, result, controller_name]()
                    {
                        // 3) Activate the controller
                        activateControllerAsync(
                            controller_name,
                            [this, goal_handle, result, controller_name]()
                            {
                                result->success = true;
                                result->message = "Controller loaded and activated successfully.";
                                goal_handle->succeed(result);
                            },
                            [goal_handle, result, controller_name](const std::string &err)
                            {
                                result->success = false;
                                result->message = "Activate error: " + err;
                                goal_handle->abort(result);
                            });
                    },
                    [goal_handle, result, controller_name](const std::string &err)
                    {
                        result->success = false;
                        result->message = "Configure error: " + err;
                        goal_handle->abort(result);
                    });
            },
            [goal_handle, result, controller_name](const std::string &err)
            {
                result->success = false;
                result->message = "Load error: " + err;
                goal_handle->abort(result);
            });
    }

    // -------------------------------------
    // Helpers:
    // -------------------------------------

    void unloadControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[unloadControllerAsync] Called for controller: '%s'",
                    controller_name.c_str());

        if (!unload_controller_client_->service_is_ready())
        {
            std::string msg = "[unloadControllerAsync] Service '/controller_manager/unload_controller' unavailable";
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
        request->name = controller_name;

        unload_controller_client_->async_send_request(request,
                                                      [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedFuture future_response)
                                                      {
                                                          auto response = future_response.get();
                                                          if (!response)
                                                          {
                                                              std::string msg = "[unloadControllerAsync] Null response for " + controller_name;
                                                              RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                              on_error(msg);
                                                              return;
                                                          }

                                                          if (response->ok)
                                                          {
                                                              RCLCPP_INFO(node_->get_logger(), "[unloadControllerAsync] SUCCESS: Unloaded '%s'", controller_name.c_str());
                                                              on_success();
                                                          }
                                                          else
                                                          {
                                                              std::string msg = "[unloadControllerAsync] Failed to unload " + controller_name;
                                                              RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                              on_error(msg);
                                                          }
                                                      });

        RCLCPP_INFO(node_->get_logger(), "[unloadControllerAsync] Request sent, awaiting response...");
    }

    void loadControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[loadControllerAsync] Called for controller: '%s'",
                    controller_name.c_str());

        if (!load_controller_client_->service_is_ready())
        {
            std::string msg = "[loadControllerAsync] Service '/controller_manager/load_controller' unavailable for: " + controller_name;
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
        request->name = controller_name;

        load_controller_client_->async_send_request(request,
                                                    [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedFuture future_response)
                                                    {
                                                        auto response = future_response.get();
                                                        if (!response)
                                                        {
                                                            std::string msg = "[loadControllerAsync] Null response for " + controller_name;
                                                            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                            on_error(msg);
                                                            return;
                                                        }

                                                        if (response->ok)
                                                        {
                                                            RCLCPP_INFO(node_->get_logger(), "[loadControllerAsync] SUCCESS: Loaded '%s'", controller_name.c_str());
                                                            on_success();
                                                        }
                                                        else
                                                        {
                                                            std::string msg = "[loadControllerAsync] Failed to load " + controller_name;
                                                            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                            on_error(msg);
                                                        }
                                                    });

        RCLCPP_INFO(node_->get_logger(), "[loadControllerAsync] Request sent, awaiting response...");
    }

    void activateControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[activateControllerAsync] Called for controller: '%s'",
                    controller_name.c_str());

        if (!switch_controller_client_->service_is_ready())
        {
            std::string msg = "[activateControllerAsync] Service '/controller_manager/switch_controller' unavailable";
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers.push_back(controller_name);
        request->deactivate_controllers.clear();
        request->strictness = request->STRICT;
        request->start_asap = false;
        request->timeout.sec = 0;
        request->timeout.nanosec = 0;

        switch_controller_client_->async_send_request(request,
                                                      [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response)
                                                      {
                                                          RCLCPP_INFO(node_->get_logger(), "[activateControllerAsync] Received response for '%s'.", controller_name.c_str());

                                                          auto response = future_response.get();
                                                          if (!response)
                                                          {
                                                              std::string msg = "[activateControllerAsync] Null response for " + controller_name;
                                                              RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                              on_error(msg);
                                                              return;
                                                          }

                                                          if (response->ok)
                                                          {
                                                              RCLCPP_INFO(node_->get_logger(), "[activateControllerAsync] SUCCESS: Activated '%s'", controller_name.c_str());
                                                              on_success();
                                                          }
                                                          else
                                                          {
                                                              std::string msg = "[activateControllerAsync] Failed to activate " + controller_name;
                                                              RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                              on_error(msg);
                                                          }
                                                      });

        RCLCPP_INFO(node_->get_logger(), "[activateControllerAsync] Request sent, awaiting response...");
    }

    void deactivateControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[deactivateControllerAsync] Called for controller: '%s'",
                    controller_name.c_str());

        // Check if the service is ready
        if (!switch_controller_client_->service_is_ready())
        {
            std::string msg = "[deactivateControllerAsync] Service '/controller_manager/switch_controller' unavailable";
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        // Prepare the SwitchController request
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers.clear();                      // We are not starting any new controllers
        request->deactivate_controllers.push_back(controller_name); // We want to deactivate/stop this controller
        request->strictness = request->STRICT;                      // STRICT means all requested switches must succeed
        request->start_asap = false;                                // You can set this to true or false depending on your usage
        request->timeout.sec = 0;                                   // Timeout for the switch
        request->timeout.nanosec = 0;

        // Send asynchronous request
        switch_controller_client_->async_send_request(
            request,
            [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[deactivateControllerAsync] Received response for '%s'.",
                            controller_name.c_str());

                auto response = future_response.get();
                if (!response)
                {
                    std::string msg = "[deactivateControllerAsync] Null response for " + controller_name;
                    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                    on_error(msg);
                    return;
                }

                if (response->ok)
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "[deactivateControllerAsync] SUCCESS: Deactivated '%s'",
                                controller_name.c_str());
                    on_success();
                }
                else
                {
                    std::string msg = "[deactivateControllerAsync] Failed to deactivate " + controller_name;
                    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                    on_error(msg);
                }
            });

        RCLCPP_INFO(node_->get_logger(),
                    "[deactivateControllerAsync] Request sent, awaiting response...");
    }

    void configureControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(), "[configureControllerAsync] Configuring '%s'", controller_name.c_str());

        if (!configure_controller_client_->service_is_ready())
        {
            std::string msg = "[configureControllerAsync] Service unavailable for " + controller_name;
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
        request->name = controller_name;

        configure_controller_client_->async_send_request(
            request,
            [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedFuture future)
            {
                auto response = future.get();
                if (!response)
                {
                    std::string msg = "[configureControllerAsync] Null response for " + controller_name;
                    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                    on_error(msg);
                    return;
                }

                if (response->ok)
                {
                    RCLCPP_INFO(node_->get_logger(), "[configureControllerAsync] Successfully configured '%s'", controller_name.c_str());
                    on_success();
                }
                else
                {
                    std::string msg = "[configureControllerAsync] Failed to configure " + controller_name;
                    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                    on_error(msg);
                }
            });
    }
};
