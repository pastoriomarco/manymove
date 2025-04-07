#include "manymove_planner/move_group_planner.hpp"

MoveGroupPlanner::MoveGroupPlanner(
    const rclcpp::Node::SharedPtr &node,
    const std::string &planning_group,
    const std::string &base_frame,
    const std::string &tcp_frame,
    const std::string &traj_controller)
    : node_(node), logger_(node->get_logger()),
      planning_group_(planning_group),
      base_frame_(base_frame),
      tcp_frame_(tcp_frame),
      traj_controller_(traj_controller)
{
    // Initialize MoveGroupInterface with the shared node
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
    move_group_interface_->setPlanningTime(0.5);

    RCLCPP_INFO(logger_, "MoveGroupPlanner initialized with group: %s", planning_group_.c_str());

    // Create a PlanningSceneMonitor

    // "robot_description" is typically the parameter name for the robot’s URDF
    // If you have a different param or an SRDF param, adjust accordingly
    const std::string robot_description_param = "robot_description";

    // Construct the monitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, robot_description_param);

    if (!planning_scene_monitor_->getPlanningScene())
    {
        RCLCPP_ERROR(logger_, "PlanningSceneMonitor did not load a valid robot model");
        // handle error if needed
    }
    else
    {
        // Optionally request the full scene once from the service
        // planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");

        // Start listening to diffs / updates from move_group
        planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");

        // Start listening to the robot state topic
        planning_scene_monitor_->startStateMonitor();

        // To get world geometry updates (collision objects) via /collision_object, etc.
        planning_scene_monitor_->startWorldGeometryMonitor();

        RCLCPP_INFO(logger_, "PlanningSceneMonitor started: listening to /monitored_planning_scene");
    }

    // Initialize FollowJointTrajectory action client
    follow_joint_traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_, "/" + traj_controller_ + "/follow_joint_trajectory");
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action server not available after waiting");
    }
    else
    {
        RCLCPP_INFO(logger_, "FollowJointTrajectory action server available");
    }

    // /joint_states subscriber to map positions and velocities
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        std::bind(&MoveGroupPlanner::jointStateCallback, this, std::placeholders::_1));

    // For safety, initialize the maps as empty
    current_positions_.clear();
    current_velocities_.clear();
}

// functions to let ExecuteTrajectory in action_server.cpp handle the collision check feedback
rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr MoveGroupPlanner::getFollowJointTrajClient() const
{
    return follow_joint_traj_client_;
}

double MoveGroupPlanner::computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const
{
    const auto &joint_trajectory = trajectory.joint_trajectory;
    double total_length = 0.0;

    // Compute joint-space path length
    auto computeJointPathLength = [&](const trajectory_msgs::msg::JointTrajectory &traj)
    {
        double length = 0.0;
        for (size_t i = 1; i < traj.points.size(); ++i)
        {
            double segment_length = 0.0;
            for (size_t j = 0; j < traj.points[i].positions.size(); ++j)
            {
                double diff = traj.points[i].positions[j] - traj.points[i - 1].positions[j];
                segment_length += diff * diff;
            }
            length += std::sqrt(segment_length);
        }
        return length;
    };

    // Compute Cartesian path length
    auto computeCartesianPathLength = [&]()
    {
        if (trajectory.multi_dof_joint_trajectory.joint_names.empty())
            return 0.0;

        double length = 0.0;
        for (size_t i = 1; i < trajectory.multi_dof_joint_trajectory.points.size(); ++i)
        {
            const auto &prev_point = trajectory.multi_dof_joint_trajectory.points[i - 1];
            const auto &curr_point = trajectory.multi_dof_joint_trajectory.points[i];

            if (prev_point.transforms.empty() || curr_point.transforms.empty())
                continue;

            const auto &prev_transform = prev_point.transforms[0];
            const auto &curr_transform = curr_point.transforms[0];

            Eigen::Vector3d prev_pos(prev_transform.translation.x, prev_transform.translation.y, prev_transform.translation.z);
            Eigen::Vector3d curr_pos(curr_transform.translation.x, curr_transform.translation.y, curr_transform.translation.z);

            double dist = (curr_pos - prev_pos).norm();
            length += dist;
        }
        return length;
    };

    double joint_length = computeJointPathLength(joint_trajectory);
    double cart_length = computeCartesianPathLength();

    total_length = (2 * cart_length) + (joint_length / 2.0);
    return total_length;
}

double MoveGroupPlanner::computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const
{
    // Compute max Cartesian speed by analyzing consecutive waypoints
    if (trajectory->getWayPointCount() < 2)
        return 0.0;
    double max_speed = 0.0;
    for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
    {
        Eigen::Isometry3d prev_pose = trajectory->getWayPoint(i - 1).getGlobalLinkTransform(tcp_frame_);
        Eigen::Isometry3d curr_pose = trajectory->getWayPoint(i).getGlobalLinkTransform(tcp_frame_);

        double dist = (curr_pose.translation() - prev_pose.translation()).norm();
        double dt = trajectory->getWayPointDurationFromPrevious(i);
        double speed = dist / dt;
        if (speed > max_speed)
            max_speed = speed;
    }
    return max_speed;
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveGroupPlanner::applyTimeParameterization(
    const moveit_msgs::msg::RobotTrajectory &input_traj,
    const manymove_msgs::msg::MovementConfig &config)
{
    // 1) Basic checks
    if (input_traj.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(logger_, "Cannot parameterize an empty trajectory (MoveGroupPlanner).");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    auto robot_model = move_group_interface_->getRobotModel();
    if (!robot_model)
    {
        RCLCPP_ERROR(logger_, "Robot model is null (MoveGroupPlanner).");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    // 2) Convert input to a RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr robot_traj_ptr =
        std::make_shared<robot_trajectory::RobotTrajectory>(
            robot_model, planning_group_);

    // Get a "current state" from MoveGroup:
    auto current_state_ptr = move_group_interface_->getCurrentState(5.0 /*seconds*/);
    if (!current_state_ptr)
    {
        RCLCPP_ERROR(logger_, "No current robot state received after 5s; aborting.");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    const moveit::core::RobotState &current_state = *current_state_ptr;
    robot_traj_ptr->setRobotTrajectoryMsg(current_state, input_traj);

    // 3) TOTG or iterative param, similar loop as above
    double velocity_scaling_factor = config.velocity_scaling_factor;
    double acceleration_scaling_factor = config.acceleration_scaling_factor;
    const int max_iterations = 32;

    for (int iteration = 0; iteration < max_iterations; ++iteration)
    {
        // Reset durations
        for (size_t i = 1; i < robot_traj_ptr->getWayPointCount(); i++)
        {
            robot_traj_ptr->setWayPointDurationFromPrevious(i, 0.0);
        }

        bool time_param_success = false;
        if (config.smoothing_type == "time_optimal")
        {
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*robot_traj_ptr,
                                                              velocity_scaling_factor,
                                                              acceleration_scaling_factor);
        }
        else if (config.smoothing_type == "ruckig")
        {
            // Ruckig-based smoothing
            time_param_success = trajectory_processing::RuckigSmoothing::applySmoothing(
                *robot_traj_ptr, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else
        {
            // Default fallback to time_optimal
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*robot_traj_ptr,
                                                              velocity_scaling_factor,
                                                              acceleration_scaling_factor);
        }

        if (!time_param_success)
        {
            // Attempt fallback if TOTG or rockig fails
            RCLCPP_ERROR(logger_, "Failed to compute time stamps with '%s'",
                         config.smoothing_type.c_str());
            RCLCPP_WARN(logger_, "Fallback to time-optimal smoothing...");

            // Reset durations
            for (size_t i = 1; i < robot_traj_ptr->getWayPointCount(); i++)
            {
                robot_traj_ptr->setWayPointDurationFromPrevious(i, 0.0);
            }

            trajectory_processing::TimeOptimalTrajectoryGeneration fallback_param;
            bool fallback_ok = fallback_param.computeTimeStamps(*robot_traj_ptr,
                                                                velocity_scaling_factor,
                                                                acceleration_scaling_factor);
            if (!fallback_ok)
            {
                RCLCPP_ERROR(logger_, "Fallback time-optimal smoothing also failed.");
                return {false, moveit_msgs::msg::RobotTrajectory()};
            }
        }

        // Check cartesian speed if needed
        double max_speed = computeMaxCartesianSpeed(robot_traj_ptr);
        if (max_speed <= config.max_cartesian_speed)
        {
            // All good, we've param'd the trajectory
            moveit_msgs::msg::RobotTrajectory output_traj;
            robot_traj_ptr->getRobotTrajectoryMsg(output_traj);
            return {true, output_traj};
        }
        else
        {
            // Need to reduce velocity scaling factor
            RCLCPP_WARN(logger_, "Adjusting cartesian speed from %.2f to <= %.2f. Reducing velocity scale...",
                        max_speed, config.max_cartesian_speed);

            double scale = (config.max_cartesian_speed * 0.99) / max_speed;
            velocity_scaling_factor *= scale;

            // If it's too small, we abort
            if (velocity_scaling_factor < 0.001 || acceleration_scaling_factor < 0.001)
            {
                RCLCPP_ERROR(logger_, "Scaling factors too small to limit Cartesian speed.");
                return {false, moveit_msgs::msg::RobotTrajectory()};
            }
        }
    }

    RCLCPP_ERROR(logger_, "[MoveGroupPlanner] Could not limit speed after iteration loops.");
    return {false, moveit_msgs::msg::RobotTrajectory()};
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveGroupPlanner::plan(const manymove_msgs::action::PlanManipulator::Goal &goal_msg)
{
    std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;

    // Handle start state
    if (!goal_msg.goal.start_joint_values.empty())
    {
        moveit::core::RobotState start_state(*move_group_interface_->getCurrentState());
        const moveit::core::JointModelGroup *joint_model_group = start_state.getJointModelGroup(move_group_interface_->getName());

        start_state.setJointGroupPositions(joint_model_group, goal_msg.goal.start_joint_values);
        move_group_interface_->setStartState(start_state);
    }
    else
    {
        move_group_interface_->setStartStateToCurrentState();
    }

    const auto &cfg = goal_msg.goal.config;

    // In MoveGroupInterface, pipeline selection is typically done by setting the "planner_id".
    // e.g. "ompl/AnytimePRM", "chomp/CHOMP", "pilz_industrial_motion_planner/LIN", etc.
    // Here we split them for coherence with MoveItCPP version:
    if (!cfg.planning_pipeline.empty() && !cfg.planner_id.empty())
    {
        // e.g. "chomp/CHOMP" or "ompl/RRTConnect" or "pilz_industrial_motion_planner/LIN"
        move_group_interface_->setPlanningPipelineId(cfg.planning_pipeline);
        move_group_interface_->setPlannerId(cfg.planning_pipeline + "/" + cfg.planner_id);
    }
    // If the user uses only movegroup version and needs compatibility, he can set the full string in planner_id only:
    else if (!cfg.planner_id.empty())
    {
        // If user only passes "ompl/PRM" in planner_id, just set that:
        move_group_interface_->setPlannerId(cfg.planner_id);
    }

    // Planning time
    if (cfg.planning_time > 0.0)
        move_group_interface_->setPlanningTime(cfg.planning_time);

    // lanning attempts
    if (cfg.planning_attempts > 0)
        move_group_interface_->setNumPlanningAttempts(cfg.planning_attempts);

    // Velocity/acceleration scaling
    if (cfg.velocity_scaling_factor > 0.0)
        move_group_interface_->setMaxVelocityScalingFactor(cfg.velocity_scaling_factor);

    if (cfg.acceleration_scaling_factor > 0.0)
        move_group_interface_->setMaxAccelerationScalingFactor(cfg.acceleration_scaling_factor);

    // Set movement targets
    if ((goal_msg.goal.movement_type == "pose") || (goal_msg.goal.movement_type == "joint") || (goal_msg.goal.movement_type == "named"))
    {
        if (goal_msg.goal.movement_type == "pose")
        {
            move_group_interface_->setPoseTarget(goal_msg.goal.pose_target, tcp_frame_);
        }
        else if (goal_msg.goal.movement_type == "joint")
        {
            move_group_interface_->setJointValueTarget(goal_msg.goal.joint_values);
        }
        else if (goal_msg.goal.movement_type == "named")
        {
            move_group_interface_->setNamedTarget(goal_msg.goal.named_target);
        }

        // Plan multiple trajectories
        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit && static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                double length = computePathLength(plan.trajectory_);
                trajectories.emplace_back(plan.trajectory_, length);
            }
            else
            {
                RCLCPP_WARN(logger_, "%s target planning attempt %d failed.",
                            goal_msg.goal.movement_type.c_str(), attempts + 1);
            }
            attempts++;
        }
    }
    else if (goal_msg.goal.movement_type == "cartesian")
    {
        // Cartesian movement
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(goal_msg.goal.pose_target);

        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit && static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            RCLCPP_DEBUG_STREAM(logger_, "Cartesian path planning attempt with step size " << goal_msg.goal.config.step_size << ", jump threshold " << goal_msg.goal.config.jump_threshold);
            double fraction = move_group_interface_->computeCartesianPath(
                waypoints, goal_msg.goal.config.step_size, goal_msg.goal.config.jump_threshold, plan.trajectory_);

            if (fraction >= 1.0)
            {
                double length = computePathLength(plan.trajectory_);
                trajectories.emplace_back(plan.trajectory_, length);
            }
            else
            {
                RCLCPP_WARN(logger_, "Cartesian path planning attempt %d failed (%.2f%% achieved)", attempts + 1, fraction * 100.0);
            }
            attempts++;
        }
    }
    else
    {
        RCLCPP_ERROR(logger_, "Unknown movement_type: %s", goal_msg.goal.movement_type.c_str());
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger_, "No valid trajectory found for movement_type: %s", goal_msg.goal.movement_type.c_str());
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    // Select the shortest trajectory
    auto shortest = std::min_element(trajectories.begin(), trajectories.end(),
                                     [](const auto &a, const auto &b)
                                     { return a.second < b.second; });

    return {true, shortest->first};
}

bool MoveGroupPlanner::executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    // (1) Basic checks
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(logger_, "Received an empty trajectory. Execution aborted.");
        return false;
    }
    if (!follow_joint_traj_client_)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action client not initialized!");
        return false;
    }

    // (2) Wait for the action server
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action server not available after waiting.");
        return false;
    }

    // (3) Create the goal
    control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = trajectory.joint_trajectory;

    RCLCPP_INFO(logger_, "Sending FollowJointTrajectory goal (MoveGroupPlanner) ...");

    // (4) Prepare the promise/future to track success or failure
    auto result_promise = std::make_shared<std::promise<bool>>();
    std::future<bool> result_future = result_promise->get_future();

    // (5) Define SendGoalOptions with optional feedback and result callbacks
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions options;

    // (a) feedback callback
    options.feedback_callback =
        [this](auto /*unused_goal_handle*/,
               const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
    {
        RCLCPP_DEBUG(logger_, "Partial execution, time_from_start: %.2f",
                     rclcpp::Duration(feedback->actual.time_from_start).seconds());
    };

    // (b) Result callback
    options.result_callback =
        [this, result_promise](const auto &wrapped_result)
    {
        bool success = false;
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(logger_, "FollowJointTrajectory succeeded (MoveGroupPlanner).");
            success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger_, "FollowJointTrajectory was aborted.");
            success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(logger_, "FollowJointTrajectory was canceled.");
            success = false;
            break;
        default:
            RCLCPP_ERROR(logger_, "Unknown result from FollowJointTrajectory.");
            success = false;
            break;
        }
        result_promise->set_value(success);
    };

    // (6) Send the goal asynchronously
    auto goal_handle_future = follow_joint_traj_client_->async_send_goal(goal_msg, options);

    // (7) Wait for the goal handle
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Failed to get goal handle from FollowJointTrajectory within 5 seconds.");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory goal was rejected by the server.");
        return false;
    }

    // (8) Wait for the result
    RCLCPP_INFO(logger_, "Waiting for FollowJointTrajectory result (MoveGroupPlanner) ...");
    auto status = result_future.wait_for(std::chrono::seconds(300));
    if (status != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution timed out (MoveGroupPlanner).");
        return false;
    }

    bool exec_success = result_future.get();
    if (!exec_success)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution failed (MoveGroupPlanner).");
        return false;
    }

    RCLCPP_INFO(logger_, "Trajectory execution succeeded (MoveGroupPlanner).");
    return true;
}

bool MoveGroupPlanner::areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const
{
    if (j1.size() != j2.size())
    {
        return false;
    }

    for (size_t i = 0; i < j1.size(); i++)
    {
        if (std::abs(j1[i] - j2[i]) > tolerance)
        {
            return false;
        }
    }

    return true;
}

bool MoveGroupPlanner::sendControlledStop(double deceleration_time)
{
    RCLCPP_INFO(logger_, "Constructing a single-point 'controlled stop' trajectory (%.2fs).",
                deceleration_time);

    // 1) Make sure the FollowJointTrajectory action server is up
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(logger_, "Cannot send stop trajectory, FollowJointTrajectory server is not available.");
        return false;
    }

    // 2) Retrieve current joint positions
    std::vector<double> positions = move_group_interface_->getCurrentJointValues();
    if (positions.empty())
    {
        RCLCPP_ERROR(logger_, "Failed to retrieve current joint values.");
        return false;
    }

    // Only Point: current position, zero velocity: it will "spring-back" to the position it was when the command is issued.
    control_msgs::action::FollowJointTrajectory::Goal stop_goal;
    stop_goal.trajectory.joint_names = move_group_interface_->getJointNames();

    // We set velocities to zero
    std::vector<double> velocities(positions.size(), 0.0);

    // 3) Build a SINGLE-point trajectory
    trajectory_msgs::msg::JointTrajectoryPoint stop_point;
    stop_point.positions = positions;
    stop_point.velocities = velocities; // Assume 0 if not available
    stop_point.accelerations.resize(positions.size(), 0.0);

    // Use deceleration_time to define how long we give the controller to ramp to zero velocity.
    // A larger deceleration_time will produce a smoother (but slower) stop.
    stop_point.time_from_start = rclcpp::Duration::from_seconds(deceleration_time);

    stop_goal.trajectory.points.push_back(stop_point);

    RCLCPP_INFO(logger_, "Sending single-point stop trajectory [time_from_start=%.2fs].", deceleration_time);

    // 4) Send the goal
    auto send_goal_future = follow_joint_traj_client_->async_send_goal(stop_goal);
    if (send_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Timeout while sending stop trajectory goal.");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "Stop trajectory goal was rejected by the controller.");
        return false;
    }

    // 5) Wait for result to confirm execution
    auto result_future = follow_joint_traj_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Controlled stop goal did not finish before timeout.");
        return false;
    }

    auto wrapped_result = result_future.get();
    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger_, "Single-point controlled stop completed successfully.");
        return true;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger_, "Stop goal was aborted by the controller.");
        return false;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger_, "Stop goal was canceled by the controller.");
        return false;
    default:
        RCLCPP_ERROR(logger_, "Stop goal ended with unknown result code %d.", (int)wrapped_result.code);
        return false;
    }
}

bool MoveGroupPlanner::isStateValid(const moveit::core::RobotState *state,
                                    const moveit::core::JointModelGroup *group) const
{
    if (!planning_scene_monitor_)
    {
        RCLCPP_ERROR(logger_, "PlanningSceneMonitor is null. Cannot perform collision checking.");
        return false;
    }

    planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
    if (!locked_scene)
    {
        RCLCPP_ERROR(logger_, "LockedPlanningSceneRO is null. Cannot perform collision checking.");
        return false;
    }

    // Clone the input state
    moveit::core::RobotState temp_state(*state);

    // Retrieve the list of joint names that belong to the group (jmg)
    const std::vector<std::string> &group_joint_names = group->getVariableNames();

    {
        std::lock_guard<std::mutex> lock(js_mutex_);
        for (const auto &entry : current_positions_)
        {
            const std::string &joint_name = entry.first;
            double joint_value = entry.second;
            // Only update joints not in the planning group
            if (std::find(group_joint_names.begin(), group_joint_names.end(), joint_name) == group_joint_names.end())
            {
                temp_state.setVariablePosition(joint_name, joint_value);
            }
        }
    }
    temp_state.update(true); // update transforms

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;  // Enable contact reporting
    collision_request.max_contacts = 1; // Adjust as needed

    locked_scene->checkCollision(collision_request, collision_result, temp_state);

    if (collision_result.collision)
    {
        RCLCPP_WARN(logger_, "[MoveGroupPlanner] Collision detected in isStateValid() (group='%s').", group->getName().c_str());
        for (const auto &contact : collision_result.contacts)
        {
            RCLCPP_WARN(logger_, "Collision between: '%s' and '%s'", contact.first.first.c_str(), contact.first.second.c_str());
        }
    }

    return !collision_result.collision;
}

bool MoveGroupPlanner::isJointStateValid(const std::vector<double> &joint_positions) const
{
    // 1) Grab the RobotModel from the MoveGroupInterface
    auto robot_model = move_group_interface_->getRobotModel();
    if (!robot_model)
    {
        RCLCPP_ERROR(logger_, "Robot model is null in isJointStateValid()");
        return false;
    }

    // 2) Retrieve the group
    const moveit::core::JointModelGroup *jmg = robot_model->getJointModelGroup(planning_group_);
    if (!jmg)
    {
        RCLCPP_ERROR(logger_, "JointModelGroup '%s' not found in isJointStateValid().", planning_group_.c_str());
        return false;
    }

    // 3) Construct a temp RobotState
    moveit::core::RobotState temp_state(robot_model);
    temp_state.setToDefaultValues();
    temp_state.setJointGroupPositions(jmg, joint_positions);
    temp_state.update();

    // 4) Reuse your existing isStateValid
    return isStateValid(&temp_state, jmg);
}

bool MoveGroupPlanner::isTrajectoryStartValid(const moveit_msgs::msg::RobotTrajectory &traj,
                                              const std::vector<double> &current_joint_state,
                                              double tolerance) const
{
    if (traj.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(logger_, "Trajectory is empty. Cannot validate start.");
        return false;
    }

    const auto &first_point = traj.joint_trajectory.points.front();
    if (first_point.positions.size() != current_joint_state.size())
    {
        RCLCPP_ERROR(logger_, "Mismatch between trajectory joint positions (%zu) and current joint state (%zu).",
                     first_point.positions.size(), current_joint_state.size());
        return false;
    }

    // Check each joint's difference
    for (size_t i = 0; i < first_point.positions.size(); ++i)
    {
        if (std::fabs(first_point.positions[i] - current_joint_state[i]) > tolerance)
        {
            RCLCPP_INFO(logger_, "Joint %zu difference (%.6f) exceeds tolerance (%.6f).",
                        i, std::fabs(first_point.positions[i] - current_joint_state[i]), tolerance);
            return false;
        }
    }
    return true;
}

void MoveGroupPlanner::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Lock mutex for thread-safe access
    std::lock_guard<std::mutex> lock(js_mutex_);

    // Update position/velocity for each joint in the message
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        const std::string &joint_name = msg->name[i];

        // Safety checks (avoid out of range)
        double pos = 0.0;
        double vel = 0.0;
        if (i < msg->position.size())
            pos = msg->position[i];
        if (i < msg->velocity.size())
            vel = msg->velocity[i];

        current_positions_[joint_name] = pos;
        current_velocities_[joint_name] = vel;
    }
}
