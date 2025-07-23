#include "manymove_planner/action_server.hpp"

ManipulatorActionServer::ManipulatorActionServer(
    const rclcpp::Node::SharedPtr &node,
    const std::shared_ptr<PlannerInterface> &planner,
    const std::string &planner_prefix)
    : node_(node), planner_(planner), planner_prefix_(planner_prefix)
{
    action_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    param_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    unload_controller_client_ = node_->create_client<controller_manager_msgs::srv::UnloadController>(
        "/controller_manager/unload_controller", rmw_qos_profile_services_default, param_callback_group_);

    load_controller_client_ = node_->create_client<controller_manager_msgs::srv::LoadController>(
        "/controller_manager/load_controller", rmw_qos_profile_services_default, param_callback_group_);

    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller", rmw_qos_profile_services_default, param_callback_group_);

    configure_controller_client_ = node_->create_client<controller_manager_msgs::srv::ConfigureController>(
        "/controller_manager/configure_controller", rmw_qos_profile_services_default, param_callback_group_);

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
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(joint_states_mutex_);
            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
            {
                current_joint_positions_[msg->name[i]] = msg->position[i];
            }
        });

    move_manipulator_server_ = rclcpp_action::create_server<MoveManipulator>(
        node_,
        planner_prefix_ + "move_manipulator", // e.g. "/move_manipulator" if prefix is empty
        std::bind(&ManipulatorActionServer::handle_move_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ManipulatorActionServer::handle_move_cancel, this, std::placeholders::_1),
        std::bind(&ManipulatorActionServer::handle_move_accepted, this, std::placeholders::_1),
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

// -------------------------------------
// MoveManipulator callbacks
// -------------------------------------

rclcpp_action::GoalResponse ManipulatorActionServer::handle_move_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    [[maybe_unused]] std::shared_ptr<const MoveManipulator::Goal> goal_msg)
{
    RCLCPP_INFO(node_->get_logger(), "[MoveManipulator] Received new goal");

    {
        std::lock_guard<std::mutex> lock(move_state_mutex_);
        move_state_ = MoveExecutionState::PLANNING;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulatorActionServer::handle_move_cancel(
    [[maybe_unused]] const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "[MoveManipulator] Received cancel request");

    {
        std::lock_guard<std::mutex> lock(move_state_mutex_);
        if (move_state_ == MoveExecutionState::EXECUTING)
        {
            const double elapsed =
                (node_->now() - executing_start_time_).seconds();

            RCLCPP_INFO(node_->get_logger(),
                        "[MoveManipulator] Cancel while EXECUTING → "
                        "controlled stop (elapsed %.3f s).",
                        elapsed);

            planner_->sendControlledStop(move_manipulator_goal_.config,
                                         executing_traj_,
                                         elapsed);
        }
        else
        {
            // Cancel while PLANNING or IDLE => do nothing to the controller
            RCLCPP_INFO(node_->get_logger(), "[MoveManipulator] Cancel while PLANNING/IDLE => no stop needed");
        }
    }

    return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorActionServer::handle_move_accepted(
    const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
{
    // Spin off a thread to do the actual logic
    std::thread{std::bind(&ManipulatorActionServer::execute_move, this, goal_handle)}.detach();
}

void ManipulatorActionServer::execute_move(
    const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "[MoveManipulator] Executing plan-or-reuse logic ...");

    auto result = std::make_shared<manymove_msgs::action::MoveManipulator::Result>();

    {
        std::lock_guard<std::mutex> lock(move_state_mutex_);
        move_state_ = MoveExecutionState::PLANNING;
    }

    const auto &goal = goal_handle->get_goal();
    moveit_msgs::msg::RobotTrajectory final_traj;
    bool have_valid_traj = false;

    // 1) Check if the user provided an existing trajectory
    if (!goal->existing_trajectory.joint_trajectory.points.empty())
    {
        std::vector<double> current_joints;
        {
            std::lock_guard<std::mutex> lock(joint_states_mutex_);
            for (const auto &jn : goal->existing_trajectory.joint_trajectory.joint_names)
            {
                current_joints.push_back(current_joint_positions_[jn]);
            }
        }
        bool starts_ok = planner_->isTrajectoryStartValid(goal->existing_trajectory, goal->plan_request, current_joints);

        bool all_ok = true;
        // const auto &pts = goal->existing_trajectory.joint_trajectory.points;
        if (!planner_->isTrajectoryValid(goal->existing_trajectory.joint_trajectory, moveit_msgs::msg::Constraints()))
        {
            RCLCPP_WARN(node_->get_logger(), "[MoveManipulator] existing_trajectory fails.");
            all_ok = false;
        }

        if (starts_ok && all_ok)
        {
            have_valid_traj = true;
            final_traj = goal->existing_trajectory;
            RCLCPP_INFO(node_->get_logger(), "[MoveManipulator] existing_trajectory is valid => will try to execute it");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "[MoveManipulator] existing_trajectory is not valid => planning anew");
        }
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "[MoveManipulator] existing_trajectory EMPTY");
    }

    // 2) If not, plan a new trajectory
    if (!have_valid_traj)
    {
        manymove_msgs::action::PlanManipulator::Goal plan_goal;
        // Copy the plan request from the move goal.
        plan_goal.goal = goal->plan_request;

        auto [plan_ok, planned_traj] = planner_->plan(plan_goal);
        if (!plan_ok)
        {
            RCLCPP_ERROR(node_->get_logger(), "[MoveManipulator] plan() failed => ABORT");
            result->success = false;
            result->message = "Planning failed";
            goal_handle->abort(result);
            return;
        }

        // // If the planner chooses the shortest path, and not the fastest, we need to time parametrize it:
        // auto [param_ok, timed] = planner_->applyTimeParameterization(planned_traj, goal->plan_request.config);
        // if (!param_ok)
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "[MoveManipulator] time param failed => ABORT");
        //     result->success = false;
        //     result->message = "Time parameterization failed";
        //     goal_handle->abort(result);
        //     return;
        // }
        // final_traj = timed;

        // With time parametrization active in the planner, the path returned is already good to go:
        final_traj = planned_traj;
    }

    if (goal_handle->is_canceling())
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[MoveManipulator] Canceled while still planning => no motion was sent");
        result->success = false;
        result->message = "Canceled during planning";
        goal_handle->canceled(result);

        {
            std::lock_guard<std::mutex> lock(move_state_mutex_);
            move_state_ = MoveExecutionState::CANCELLED;
        }
        return;
    }

    // 3) Execute the trajectory with real-time collision checking and partial feedback.
    std::string abort_reason;
    bool exec_ok = executeTrajectoryWithCollisionChecks(
        goal_handle, final_traj, abort_reason);

    if (!exec_ok)
    {
        RCLCPP_ERROR(node_->get_logger(), "[MoveManipulator] Execution failed => %s", abort_reason.c_str());
        result->success = false;
        result->message = abort_reason;

        {
            std::lock_guard<std::mutex> lock(move_state_mutex_);
            move_state_ = MoveExecutionState::CANCELLED;
        }

        goal_handle->abort(result);
        return;
    }

    // 4) Succeed
    RCLCPP_INFO(node_->get_logger(), "[MoveManipulator] Done. Execution success!");
    result->success = true;
    result->message = "Execution success";
    result->final_trajectory = final_traj;

    {
        std::lock_guard<std::mutex> lock(move_state_mutex_);
        move_state_ = MoveExecutionState::COMPLETED;
    }

    goal_handle->succeed(result);
}

// -------------------------------------
// UnloadTrajController callbacks
// -------------------------------------

rclcpp_action::GoalResponse ManipulatorActionServer::handle_unload_traj_goal(
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

rclcpp_action::CancelResponse ManipulatorActionServer::handle_unload_traj_cancel(
    [[maybe_unused]] const std::shared_ptr<GoalHandleUnloadTrajController> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(),
                "Received request to CANCEL UnloadTrajController goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorActionServer::handle_unload_traj_accepted(
    const std::shared_ptr<GoalHandleUnloadTrajController> goal_handle)
{
    std::thread{std::bind(&ManipulatorActionServer::execute_unload_traj_controller, this, goal_handle)}.detach();
}

void ManipulatorActionServer::execute_unload_traj_controller(
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

rclcpp_action::GoalResponse ManipulatorActionServer::handle_load_traj_goal(
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

rclcpp_action::CancelResponse ManipulatorActionServer::handle_load_traj_cancel(
    [[maybe_unused]] const std::shared_ptr<GoalHandleLoadTrajController> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(),
                "Received request to CANCEL LoadTrajController goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorActionServer::handle_load_traj_accepted(
    const std::shared_ptr<GoalHandleLoadTrajController> goal_handle)
{
    std::thread{std::bind(&ManipulatorActionServer::execute_load_traj_controller, this, goal_handle)}.detach();
}

void ManipulatorActionServer::execute_load_traj_controller(
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

void ManipulatorActionServer::unloadControllerAsync(
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

void ManipulatorActionServer::loadControllerAsync(
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

void ManipulatorActionServer::activateControllerAsync(
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

void ManipulatorActionServer::deactivateControllerAsync(
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

void ManipulatorActionServer::configureControllerAsync(
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

bool ManipulatorActionServer::executeTrajectoryWithCollisionChecks(
    const std::shared_ptr<GoalHandleMoveManipulator> &goal_handle,
    const moveit_msgs::msg::RobotTrajectory &traj,
    std::string &abort_reason)
{
    // 1) Preliminary checks: is the trajectory empty?
    const auto &points = traj.joint_trajectory.points;
    if (points.empty())
    {
        abort_reason = "Empty trajectory";
        return false;
    }

    // Gather current joint state
    std::vector<double> current_joint_state;
    {
        std::lock_guard<std::mutex> lock(joint_states_mutex_);
        for (const auto &joint_name : traj.joint_trajectory.joint_names)
        {
            auto it = current_joint_positions_.find(joint_name);
            if (it == current_joint_positions_.end())
            {
                abort_reason = "Incomplete joint state for " + joint_name;
                return false;
            }
            current_joint_state.push_back(it->second);
        }
    }

    if (!planner_->isTrajectoryStartValid(traj, goal_handle->get_goal()->plan_request, current_joint_state))
    {
        abort_reason = "Trajectory start mismatch with current state";
        return false;
    }

    // 2) Validate the trajectory's end configuration.
    // Note: use get_goal() to access the move request.
    if (!planner_->isTrajectoryEndValid(traj, goal_handle->get_goal()->plan_request))
    {
        abort_reason = "Trajectory end mismatch with target";
        return false;
    }

    // 3) Check the full trajectory validity (collision checking)
    if (!planner_->isTrajectoryValid(traj.joint_trajectory,
                                     moveit_msgs::msg::Constraints()))
    {
        abort_reason = "Invalid state or collision detected.";
        return false;
    }

    // 4) Set up a promise/future so we know if the final execution is successful
    auto result_promise = std::make_shared<std::promise<bool>>();
    std::future<bool> result_future = result_promise->get_future();
    std::atomic<bool> collision_detected(false);

    // 5) Build the FollowJointTrajectory goal
    control_msgs::action::FollowJointTrajectory::Goal fjt_goal;
    fjt_goal.trajectory = traj.joint_trajectory;

    // 6) Prepare the SendGoalOptions with feedback and result callbacks.
    auto follow_joint_traj_client = planner_->getFollowJointTrajClient();
    if (!follow_joint_traj_client)
    {
        abort_reason = "No FollowJointTrajectory action client available";
        return false;
    }

    const rclcpp::Time start_time = node_->now();

    {
        std::lock_guard<std::mutex> lock(move_state_mutex_);
        move_state_ = MoveExecutionState::EXECUTING;
        executing_start_time_ = start_time;
        executing_traj_ = traj;
        move_manipulator_goal_ = goal_handle->get_goal()->plan_request;
    }

    double total_time_s = rclcpp::Duration(traj.joint_trajectory.points.back().time_from_start).seconds();

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opts;

    // Feedback callback: perform collision check on the *future* part of the path
    opts.feedback_callback = [this, &collision_detected, goal_handle, traj = traj.joint_trajectory, start_time, total_time_s](auto /*unused_handle*/,
                                                                                                                              const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> &feedback) mutable
    {
        RCLCPP_DEBUG(node_->get_logger(), "Partial execution (MoveItCppPlanner): time_from_start %.2f",
                     rclcpp::Duration(feedback->actual.time_from_start).seconds());

        if (!feedback || feedback->actual.positions.empty())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[executeTrajectoryWithCollisionChecks] Empty feedback => abort");
            collision_detected.store(true);
        }

        // elapsed time since start
        double elapsed_s = (node_->now() - start_time).seconds();

        RCLCPP_DEBUG(node_->get_logger(),
                     "[executeTrajectoryWithCollisionChecks] Collision check at t=%.3f of %.3f", elapsed_s, total_time_s);

        // collision-check the *remaining* trajectory automatically by time
        if (!planner_->isTrajectoryValid(traj,
                                         moveit_msgs::msg::Constraints(),
                                         elapsed_s))
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[executeTrajectoryWithCollisionChecks] Collision predicted at t=%.3f of %.3f", elapsed_s, total_time_s);
            collision_detected.store(true);
        }

        // publish progress in [0..1]
        auto fb = std::make_shared<manymove_msgs::action::MoveManipulator::Feedback>();
        fb->progress = static_cast<float>(std::min(elapsed_s / total_time_s, 1.0));
        fb->in_collision = collision_detected.load();
        goal_handle->publish_feedback(fb);
    };

    // Result callback: set the promise value based on execution result
    opts.result_callback = [this, result_promise, &collision_detected](const auto &wrapped_result)
    {
        bool success = (!collision_detected.load()) &&
                       (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED);
        result_promise->set_value(success);
    };

    // 7) Send the trajectory execution goal
    auto goal_handle_future = follow_joint_traj_client->async_send_goal(fjt_goal, opts);
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        abort_reason = "Timeout sending FollowJointTrajectory goal";
        return false;
    }
    auto fjt_goal_handle = goal_handle_future.get();
    if (!fjt_goal_handle)
    {
        abort_reason = "FollowJointTrajectory goal rejected by server";
        return false;
    }

    // 8) Wait for and process the result
    auto res_future = follow_joint_traj_client->async_get_result(fjt_goal_handle);

    double timeout_s = std::max(2.0 * total_time_s, 5.0);

    if (res_future.wait_for(
            std::chrono::duration<double>(timeout_s)) != std::future_status::ready)
    {
        abort_reason = "Timeout (" + std::to_string(timeout_s) + " s) waiting for FollowJointTrajectory result";
        return false;
    }

    auto wrapped_result = res_future.get();
    bool final_status = result_future.get();
    if (!final_status)
    {
        if (collision_detected.load())
        {
            abort_reason = "Collision detected during trajectory execution";
        }
        else
        {
            abort_reason = "FollowJointTrajectory failed (" +
                           std::to_string(wrapped_result.result->error_code) + ")";
            if (!wrapped_result.result->error_string.empty())
            {
                abort_reason += ": " + wrapped_result.result->error_string;
            }
        }
        return false;
    }

    // Success
    return true;
}
