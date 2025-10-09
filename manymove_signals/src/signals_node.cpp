#include "manymove_signals/signals.hpp"

namespace
{
template <typename NodeT, typename ServiceT>
auto createClientCompat(NodeT *node,
                        const std::string &service_name,
                        rclcpp::CallbackGroup::SharedPtr group,
                        int)
    -> decltype(node->template create_client<ServiceT>(service_name, rclcpp::ServicesQoS(), group))
{
    return node->template create_client<ServiceT>(service_name, rclcpp::ServicesQoS(), group);
}

template <typename NodeT, typename ServiceT>
auto createClientCompat(NodeT *node,
                        const std::string &service_name,
                        rclcpp::CallbackGroup::SharedPtr group,
                        long)
    -> decltype(node->template create_client<ServiceT>(service_name, rmw_qos_profile_services_default, group))
{
    return node->template create_client<ServiceT>(service_name, rmw_qos_profile_services_default, group);
}

template <typename NodeT, typename ServiceT>
auto createClient(NodeT *node,
                  const std::string &service_name,
                  rclcpp::CallbackGroup::SharedPtr group)
    -> decltype(createClientCompat<NodeT, ServiceT>(node, service_name, group, 0))
{
    return createClientCompat<NodeT, ServiceT>(node, service_name, group, 0);
}
} // namespace

namespace manymove_signals
{

    SignalsNode::SignalsNode()
        : Node("manymove_signals_node")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("robot_model", "lite6"); // defaults to lite6; TODO: add parameters for prefix
        this->get_parameter("robot_model", robot_model_);

        this->declare_parameter<std::string>("robot_prefix", ""); 
        this->get_parameter("robot_prefix", robot_prefix_);

        // Define Callback Groups
        action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        subscription_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Initialize service clients for IO operations and ResetRobotState
        initialize_service_clients();

        // Wait for all service clients to be available
        if (!wait_for_services())
        {
            RCLCPP_ERROR(this->get_logger(), "Not all required services are available. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Initialize Action Servers with the action callback group
        initialize_action_servers();

        // Initialize Subscription for Robot States
        initialize_robot_state_subscription();

        RCLCPP_INFO(this->get_logger(), "manymove_signals_node has been started.");
    }

    // Initialize all necessary service clients based on robot model
    void SignalsNode::initialize_service_clients()
    {
        std::string namespace_prefix;
        if (robot_model_ == "lite6" || robot_model_ == "uf850")
        {
            namespace_prefix = "/" + robot_prefix_ + "ufactory";
            RCLCPP_INFO(this->get_logger(), namespace_prefix.c_str());
        }
        else if (robot_model_ == "xarm")
        {
            namespace_prefix = "/" + robot_prefix_ + "xarm";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported robot model: %s", robot_model_.c_str());
            throw std::runtime_error("Unsupported robot model");
        }

        // Tool GPIO
        set_digital_io_clients_["tool"] = createClient<SignalsNode, xarm_msgs::srv::SetDigitalIO>(
            this,
            namespace_prefix + "/set_tgpio_digital",
            service_callback_group_);
        get_digital_io_clients_["tool"] = createClient<SignalsNode, xarm_msgs::srv::GetDigitalIO>(
            this,
            namespace_prefix + "/get_tgpio_digital",
            service_callback_group_);

        // Controller GPIO
        set_digital_io_clients_["controller"] = createClient<SignalsNode, xarm_msgs::srv::SetDigitalIO>(
            this,
            namespace_prefix + "/set_cgpio_digital",
            service_callback_group_);
        get_digital_io_clients_["controller"] = createClient<SignalsNode, xarm_msgs::srv::GetDigitalIO>(
            this,
            namespace_prefix + "/get_cgpio_digital",
            service_callback_group_);

        // Initialize SetInt16 clients for mode and state
        set_int16_clients_["mode"] = createClient<SignalsNode, xarm_msgs::srv::SetInt16>(
            this,
            namespace_prefix + "/set_mode",
            service_callback_group_);
        set_int16_clients_["state"] = createClient<SignalsNode, xarm_msgs::srv::SetInt16>(
            this,
            namespace_prefix + "/set_state",
            service_callback_group_);

        // Initialize additional service clients for ResetRobotState
        clean_error_client_ = createClient<SignalsNode, xarm_msgs::srv::Call>(
            this,
            namespace_prefix + "/clean_error",
            service_callback_group_);

        motion_enable_client_ = createClient<SignalsNode, xarm_msgs::srv::SetInt16ById>(
            this,
            namespace_prefix + "/motion_enable",
            service_callback_group_);
    }

    // Wait for all service clients to be available
    bool SignalsNode::wait_for_services()
    {
        bool all_services_available = true;

        // Wait for SetDigitalIO services
        for (const auto &[io_type, client] : set_digital_io_clients_)
        {
            if (!client->wait_for_service(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(this->get_logger(), "SetDigitalIO service for IO type '%s' not available.", io_type.c_str());
                all_services_available = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "SetDigitalIO service for IO type '%s' is available.", io_type.c_str());
            }
        }

        // Wait for GetDigitalIO services
        for (const auto &[io_type, client] : get_digital_io_clients_)
        {
            if (!client->wait_for_service(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(this->get_logger(), "GetDigitalIO service for IO type '%s' not available.", io_type.c_str());
                all_services_available = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "GetDigitalIO service for IO type '%s' is available.", io_type.c_str());
            }
        }

        // Wait for SetInt16 services (mode and state)
        for (const auto &[param, client] : set_int16_clients_)
        {
            if (!client->wait_for_service(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(this->get_logger(), "SetInt16 service for '%s' not available.", param.c_str());
                all_services_available = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "SetInt16 service for '%s' is available.", param.c_str());
            }
        }

        // Wait for additional ResetRobotState services
        if (!clean_error_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "CleanError service not available.");
            all_services_available = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "CleanError service is available.");
        }

        if (!motion_enable_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "MotionEnable service not available.");
            all_services_available = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "MotionEnable service is available.");
        }

        return all_services_available;
    }

    // Initialize all action servers
    void SignalsNode::initialize_action_servers()
    {
        auto serverOptions = rcl_action_server_get_default_options();
        serverOptions.result_service_qos = rmw_qos_profile_services_default;

        // Action Server for SetOutput
        set_output_server_ = rclcpp_action::create_server<SetOutput>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            robot_prefix_ + "set_output",
            // Goal Callback
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const SetOutput::Goal> goal) -> rclcpp_action::GoalResponse
            {
                return this->handle_set_output_goal(uuid, goal);
            },
            // Cancel Callback
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<SetOutput>> goal_handle) -> rclcpp_action::CancelResponse
            {
                return this->handle_set_output_cancel(goal_handle);
            },
            // Accepted Callback
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetOutput>> goal_handle) -> void
            {
                this->execute_set_output(goal_handle);
            },
            serverOptions,
            action_callback_group_);

        // Action Server for GetInput
        get_input_server_ = rclcpp_action::create_server<GetInput>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            robot_prefix_ + "get_input",
            // Goal Callback
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const GetInput::Goal> goal) -> rclcpp_action::GoalResponse
            {
                return this->handle_get_input_goal(uuid, goal);
            },
            // Cancel Callback
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<GetInput>> goal_handle) -> rclcpp_action::CancelResponse
            {
                return this->handle_get_input_cancel(goal_handle);
            },
            // Accepted Callback
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetInput>> goal_handle) -> void
            {
                this->execute_get_input(goal_handle);
            },
            serverOptions,
            action_callback_group_);

        // Action Server for ResetRobotState
        reset_robot_state_server_ = rclcpp_action::create_server<ResetRobotState>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            robot_prefix_ + "reset_robot_state",
            // Goal Callback
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const ResetRobotState::Goal> goal) -> rclcpp_action::GoalResponse
            {
                return this->handle_reset_robot_state_goal(uuid, goal);
            },
            // Cancel Callback
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle) -> rclcpp_action::CancelResponse
            {
                return this->handle_reset_robot_state_cancel(goal_handle);
            },
            // Accepted Callback
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle) -> void
            {
                this->execute_reset_robot_state(goal_handle);
            },
            serverOptions,
            action_callback_group_);

        check_robot_state_server_ = rclcpp_action::create_server<CheckRobotState>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            robot_prefix_ + "check_robot_state",
            // Goal Callback
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const CheckRobotState::Goal> goal)
            {
                return this->handle_check_robot_state_goal(uuid, goal);
            },
            // Cancel Callback
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckRobotState>> goal_handle)
            {
                return this->handle_check_robot_state_cancel(goal_handle);
            },
            // Accepted Callback
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckRobotState>> goal_handle)
            {
                this->execute_check_robot_state(goal_handle);
            },
            serverOptions,
            action_callback_group_);
    }

    // Initialize Subscription for Robot States
    void SignalsNode::initialize_robot_state_subscription()
    {
        std::string robot_states_topic;
        if (robot_model_ == "lite6" || robot_model_ == "uf850")
        {
            robot_states_topic = "/" + robot_prefix_ + "ufactory/robot_states";
        }
        else if (robot_model_ == "xarm")
        {
            robot_states_topic = "/" + robot_prefix_ + "xarm/robot_states";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported robot model: %s", robot_model_.c_str());
            throw std::runtime_error("Unsupported robot model");
        }

        rclcpp::SubscriptionOptions options;
        options.callback_group = subscription_callback_group_;

        robot_state_subscription_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
            robot_states_topic,
            10,
            std::bind(&SignalsNode::robot_state_callback, this, std::placeholders::_1),
            options);

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s", robot_states_topic.c_str());
    }

    // Robot state callback
    void SignalsNode::robot_state_callback(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        current_robot_state_ = msg;
    }

    // ----------- SetOutput Action Callbacks -----------

    rclcpp_action::GoalResponse SignalsNode::handle_set_output_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const SetOutput::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received SetOutput goal: io_type=%s, ionum=%d, value=%d",
                    goal->io_type.c_str(), goal->ionum, goal->value);
        // Validate io_type
        if (goal->io_type != "tool" && goal->io_type != "controller")
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid io_type: %s. Must be 'tool' or 'controller'.", goal->io_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // Optionally, validate ionum based on io_type
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SignalsNode::handle_set_output_cancel(
        [[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetOutput>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel SetOutput goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SignalsNode::execute_set_output(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetOutput>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing SetOutput goal");
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<SetOutput::Result>();

        // Create the service request
        auto request = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
        request->ionum = goal->ionum;
        request->value = goal->value;

        // Determine the service client based on io_type
        auto client_it = set_digital_io_clients_.find(goal->io_type);
        if (client_it == set_digital_io_clients_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid io_type: %s", goal->io_type.c_str());
            result->success = false;
            result->message = "Invalid io_type";
            goal_handle->abort(result);
            return;
        }
        auto client = client_it->second;

        // Call the service asynchronously and handle the response in a callback
        client->async_send_request(request,
                                   [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedFuture future_response)
                                   {
                                       auto response = future_response.get();
                                       if (response->ret == 0)
                                       {
                                           RCLCPP_INFO(this->get_logger(), "SetOutput succeeded for io_type=%s, ionum=%d",
                                                       goal_handle->get_goal()->io_type.c_str(), goal_handle->get_goal()->ionum);
                                           result->success = true;
                                           result->message = "SetOutput succeeded";
                                           goal_handle->succeed(result);
                                       }
                                       else
                                       {
                                           RCLCPP_ERROR(this->get_logger(), "SetOutput failed with ret=%d for io_type=%s, ionum=%d",
                                                        response->ret, goal_handle->get_goal()->io_type.c_str(), goal_handle->get_goal()->ionum);
                                           result->success = false;
                                           result->message = "SetOutput failed with ret=" + std::to_string(response->ret);
                                           goal_handle->abort(result);
                                       }
                                   });
    }

    // ----------- GetInput Action Callbacks -----------

    rclcpp_action::GoalResponse SignalsNode::handle_get_input_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const GetInput::Goal> goal)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received GetInput goal: io_type=%s, ionum=%d",
                    goal->io_type.c_str(), goal->ionum);
        // Validate io_type
        if (goal->io_type != "tool" && goal->io_type != "controller")
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid io_type: %s. Must be 'tool' or 'controller'.", goal->io_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // TODO: validate ionum based on io_type?
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SignalsNode::handle_get_input_cancel(
        [[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetInput>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel GetInput goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SignalsNode::execute_get_input(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetInput>> goal_handle)
    {
        RCLCPP_DEBUG(this->get_logger(), "Executing GetInput goal");
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<GetInput::Result>();

        // Create the service request (empty)
        auto request = std::make_shared<xarm_msgs::srv::GetDigitalIO::Request>();

        // Determine the service client based on io_type
        auto client_it = get_digital_io_clients_.find(goal->io_type);
        if (client_it == get_digital_io_clients_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid io_type: %s", goal->io_type.c_str());
            result->success = false;
            result->message = "Invalid io_type";
            goal_handle->abort(result);
            return;
        }
        auto client = client_it->second;

        // Call the service asynchronously and handle the response in a callback
        client->async_send_request(request,
                                   [this, goal_handle, goal, result](rclcpp::Client<xarm_msgs::srv::GetDigitalIO>::SharedFuture future_response)
                                   {
                                       auto response = future_response.get();
                                       if (response->ret == 0)
                                       {
                                           // Validate ionum
                                           if (goal->ionum < 0 || goal->ionum >= static_cast<int16_t>(response->digitals.size()))
                                           {
                                               RCLCPP_ERROR(this->get_logger(), "GetInput failed: ionum=%d out of range", goal->ionum);
                                               result->success = false;
                                               result->message = "ionum out of range";
                                               goal_handle->abort(result);
                                               return;
                                           }

                                           // Extract the value from the digitals array
                                           int16_t raw_value = response->digitals[goal->ionum];
                                           int16_t value;

                                           // Ufactory robots' inputs are ON when set their value is 0, which is te opposite uf usual I/O signals
                                           // Inversion logic: 0 -> 1 (ON), else 0 (OFF)
                                           value = (raw_value == 0) ? 1 : 0;

                                           RCLCPP_DEBUG(this->get_logger(), "GetInput succeeded: io_type=%s, ionum=%d, value=%d",
                                                       goal->io_type.c_str(), goal->ionum, value);
                                           result->success = true;
                                           result->value = value;
                                           result->message = "GetInput succeeded";
                                           goal_handle->succeed(result);
                                       }
                                       else
                                       {
                                           RCLCPP_ERROR(this->get_logger(), "GetInput failed with ret=%d", response->ret);
                                           result->success = false;
                                           result->message = "GetInput failed with ret=" + std::to_string(response->ret);
                                           goal_handle->abort(result);
                                       }
                                   });
    }

    // ----------- ResetRobotState Action Callbacks -----------

    rclcpp_action::GoalResponse SignalsNode::handle_reset_robot_state_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ResetRobotState::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received ResetRobotState goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SignalsNode::handle_reset_robot_state_cancel(
        [[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel ResetRobotState goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SignalsNode::execute_reset_robot_state(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing ResetRobotState goal");
        auto result = std::make_shared<ResetRobotState::Result>();

        // Step 1: Clean Error
        auto clean_error_req = std::make_shared<xarm_msgs::srv::Call::Request>();

        clean_error_client_->async_send_request(clean_error_req,
                                                [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::Call>::SharedFuture future_response)
                                                {
                                                    auto response = future_response.get();
                                                    if (response->ret != 0)
                                                    {
                                                        RCLCPP_ERROR(this->get_logger(), "CleanError failed with ret=%d", response->ret);
                                                        result->success = false;
                                                        result->message = "CleanError failed with ret=" + std::to_string(response->ret);
                                                        goal_handle->abort(result);
                                                    }
                                                    else
                                                    {
                                                        RCLCPP_INFO(this->get_logger(), "CleanError succeeded");
                                                        // Proceed to Motion Enable
                                                        this->enable_motion(goal_handle, result);
                                                    }
                                                });
    }

    void SignalsNode::enable_motion(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
        std::shared_ptr<ResetRobotState::Result> result)
    {
        auto motion_enable_req = std::make_shared<xarm_msgs::srv::SetInt16ById::Request>();
        motion_enable_req->id = 8;
        motion_enable_req->data = 1;

        motion_enable_client_->async_send_request(motion_enable_req,
                                                  [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedFuture future_response)
                                                  {
                                                      auto response = future_response.get();
                                                      if (response->ret != 0)
                                                      {
                                                          RCLCPP_ERROR(this->get_logger(), "MotionEnable failed with ret=%d", response->ret);
                                                          result->success = false;
                                                          result->message = "MotionEnable failed with ret=" + std::to_string(response->ret);
                                                          goal_handle->abort(result);
                                                      }
                                                      else
                                                      {
                                                          RCLCPP_INFO(this->get_logger(), "MotionEnable succeeded");
                                                          // Proceed to Set Mode
                                                          this->set_mode(goal_handle, result);
                                                      }
                                                  });
    }

    void SignalsNode::set_mode(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
        std::shared_ptr<ResetRobotState::Result> result)
    {
        auto set_mode_req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
        // With xarm_planner mode would be 0, but with xarm_moveit_config and manymove_planner packages the mode to use is 1.
        set_mode_req->data = 1;

        set_int16_clients_["mode"]->async_send_request(set_mode_req,
                                                       [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedFuture future_response)
                                                       {
                                                           auto response = future_response.get();
                                                           if (response->ret != 0)
                                                           {
                                                               RCLCPP_ERROR(this->get_logger(), "SetMode failed with ret=%d", response->ret);
                                                               result->success = false;
                                                               result->message = "SetMode failed with ret=" + std::to_string(response->ret);
                                                               goal_handle->abort(result);
                                                           }
                                                           else
                                                           {
                                                               RCLCPP_INFO(this->get_logger(), "SetMode succeeded");
                                                               // Proceed to Set State
                                                               this->set_state(goal_handle, result);
                                                           }
                                                       });
    }

    void SignalsNode::set_state(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
        std::shared_ptr<ResetRobotState::Result> result)
    {
        auto set_state_req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
        set_state_req->data = 0;

        set_int16_clients_["state"]->async_send_request(set_state_req,
                                                        [this, goal_handle, result](rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedFuture future_response)
                                                        {
                                                            auto response = future_response.get();
                                                            if (response->ret != 0)
                                                            {
                                                                RCLCPP_ERROR(this->get_logger(), "SetState failed with ret=%d", response->ret);
                                                                result->success = false;
                                                                result->message = "SetState failed with ret=" + std::to_string(response->ret);
                                                                goal_handle->abort(result);
                                                            }
                                                            else
                                                            {
                                                                RCLCPP_INFO(this->get_logger(), "SetState succeeded");
                                                                // Proceed to Verification
                                                                this->verify_reset(goal_handle, result);
                                                            }
                                                        });
    }

    void SignalsNode::verify_reset(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetRobotState>> goal_handle,
        std::shared_ptr<ResetRobotState::Result> result)
    {
        // Define the number of attempts and interval
        const int max_attempts = 5;
        const std::chrono::milliseconds interval(200);

        for (int attempt = 1; attempt <= max_attempts; ++attempt)
        {
            {
                std::lock_guard<std::mutex> lock(robot_state_mutex_);
                if (!current_robot_state_)
                {
                    RCLCPP_ERROR(this->get_logger(), "No robot state information available for verification.");
                    result->success = false;
                    result->message = "No robot state information available";
                    goal_handle->abort(result);
                    return;
                }

                // Check success criteria
                bool success = true;
                std::string failure_reason;

                if (current_robot_state_->err != 0)
                {
                    success = false;
                    failure_reason += "err != 0; ";
                }
                if (current_robot_state_->mode != 1)
                {
                    success = false;
                    failure_reason += "mode != 1; ";
                }
                if (current_robot_state_->state != 2)
                {
                    success = false;
                    failure_reason += "state != 2; ";
                }
                if (current_robot_state_->mt_brake == 0)
                {
                    success = false;
                    failure_reason += "mt_brake == 0; ";
                }
                if (current_robot_state_->mt_able == 0)
                {
                    success = false;
                    failure_reason += "mt_able == 0; ";
                }

                if (success)
                {
                    RCLCPP_INFO(this->get_logger(), "ResetRobotState succeeded");
                    result->success = true;
                    result->message = "ResetRobotState succeeded";
                    goal_handle->succeed(result);
                    return;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Attempt %d: ResetRobotState verification failed: %s", attempt, failure_reason.c_str());
                }
            }

            // Wait for the specified interval before the next attempt
            std::this_thread::sleep_for(interval);
        }

        // After all attempts, if not successful, abort the action
        RCLCPP_ERROR(this->get_logger(), "ResetRobotState verification failed after %d attempts.", max_attempts);
        result->success = false;
        result->message = "ResetRobotState verification failed after " + std::to_string(max_attempts) + " attempts.";
        goal_handle->abort(result);
    }

    // ----------- CheckRobotState Action Callbacks -----------

    rclcpp_action::GoalResponse SignalsNode::handle_check_robot_state_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        [[maybe_unused]] std::shared_ptr<const CheckRobotState::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received CheckRobotState goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SignalsNode::handle_check_robot_state_cancel(
        [[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckRobotState>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel CheckRobotState goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SignalsNode::execute_check_robot_state(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckRobotState>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing CheckRobotState goal");

        auto result = std::make_shared<CheckRobotState::Result>();

        // Access current_robot_state_
        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            if (!current_robot_state_)
            {
                RCLCPP_ERROR(this->get_logger(), "No robot state information available");
                result->ready = false;
                result->err = -1;
                result->mode = -1;
                result->state = -1;
                result->message = "No robot state available";
                goal_handle->succeed(result);
                return;
            }

            // Extract robot state
            result->err = current_robot_state_->err;
            result->mode = current_robot_state_->mode;
            result->state = current_robot_state_->state;

            // Determine readiness
            result->ready = (result->err == 0 && result->mode == 1 && result->state <= 2);

            // Create a message
            result->message = result->ready ? "Robot is ready" : "Robot is not ready";
        }

        RCLCPP_INFO(this->get_logger(), "CheckRobotState result: ready=%d, err=%d, mode=%d, state=%d, message=%s",
                    result->ready, result->err, result->mode, result->state, result->message.c_str());

        goal_handle->succeed(result);
    }

} // namespace manymove_signals

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Use a MultiThreadedExecutor to handle multiple callback groups concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<manymove_signals::SignalsNode>();
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);

    rclcpp::shutdown();
    return 0;
}
