#include "manymove_planner/move_group_planner.hpp"

MoveGroupPlanner::MoveGroupPlanner(
    const rclcpp::Node::SharedPtr &node,
    const std::string &planning_group,
    const std::string &base_frame,
    const std::string &traj_controller)
    : node_(node), logger_(node->get_logger()),
      planning_group_(planning_group),
      base_frame_(base_frame),
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

double MoveGroupPlanner::computePathLength(
    const moveit_msgs::msg::RobotTrajectory &traj,
    const manymove_msgs::msg::MovementConfig &config) const
{
    if (traj.joint_trajectory.points.size() < 2)
        return 0.0;

    // ---------- helper: joint‑space ----------
    auto jointLen = [&](const trajectory_msgs::msg::JointTrajectory &jt)
    {
        double len = 0.0;
        for (size_t i = 1; i < jt.points.size(); ++i)
        {
            double seg = 0.0;
            for (size_t j = 0; j < jt.points[i].positions.size(); ++j)
            {
                double d = jt.points[i].positions[j] - jt.points[i - 1].positions[j];
                seg += d * d;
            }
            len += std::sqrt(seg);
        }
        return len;
    };

    // ---------- helper: TCP Cartesian ----------
    auto cartLen = [&](const moveit_msgs::msg::RobotTrajectory &t)
    {
        const auto robot_model = move_group_interface_->getRobotModel();
        const auto jmg = robot_model->getJointModelGroup(planning_group_);

        moveit::core::RobotState st(robot_model);
        double len = 0.0;

        for (size_t i = 1; i < t.joint_trajectory.points.size(); ++i)
        {
            st.setJointGroupPositions(jmg, t.joint_trajectory.points[i - 1].positions);
            Eigen::Vector3d p1 = st.getGlobalLinkTransform(config.tcp_frame).translation();

            st.setJointGroupPositions(jmg, t.joint_trajectory.points[i].positions);
            Eigen::Vector3d p2 = st.getGlobalLinkTransform(config.tcp_frame).translation();

            len += (p2 - p1).norm();
        }
        return len;
    };

    const double jl = jointLen(traj.joint_trajectory);
    const double cl = cartLen(traj);

    // Tune the weight to taste (here: 4× as before)
    return jl + 4.0 * cl;
}

// Function to get a geometry_msgs::msg::Pose from a RobotState and frame
geometry_msgs::msg::Pose MoveGroupPlanner::getPoseFromRobotState(const moveit::core::RobotState &robot_state,
                                                                 const std::string &link_frame) const
{

    // Clone the state to ensure the original state isn't modified
    moveit::core::RobotState state(robot_state);

    // Update link transforms to ensure they are valid
    state.updateLinkTransforms();

    geometry_msgs::msg::Pose pose;

    // Get the transform of the frame
    const Eigen::Isometry3d &pose_eigen = state.getGlobalLinkTransform(link_frame);

    // Extract position
    pose.position.x = pose_eigen.translation().x();
    pose.position.y = pose_eigen.translation().y();
    pose.position.z = pose_eigen.translation().z();

    // Extract orientation as a quaternion
    Eigen::Quaterniond quat(pose_eigen.rotation());
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
}

// Function to compute the Euclidean distance between the start pose and the target pose
double MoveGroupPlanner::computeCartesianDistance(const geometry_msgs::msg::Pose &start_pose,
                                                  const geometry_msgs::msg::Pose &target_pose) const
{
    // Compute the Euclidean distance to the target pose
    double dx = target_pose.position.x - start_pose.position.x;
    double dy = target_pose.position.y - start_pose.position.y;
    double dz = target_pose.position.z - start_pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Function to get a pose from a trajectory and TCP frame
geometry_msgs::msg::Pose MoveGroupPlanner::getPoseFromTrajectory(const moveit_msgs::msg::RobotTrajectory &traj_msg,
                                                                 const moveit::core::RobotState &robot_state,
                                                                 const std::string &link_frame,
                                                                 bool use_last_point) const
{
    geometry_msgs::msg::Pose pose;

    // Ensure the trajectory is not empty
    if (traj_msg.joint_trajectory.points.empty())
    {
        throw std::runtime_error("Trajectory is empty, cannot extract pose.");
    }

    // Select the point to use (first or last)
    const auto &point = use_last_point ? traj_msg.joint_trajectory.points.back() : traj_msg.joint_trajectory.points.front();
    const auto &joint_names = traj_msg.joint_trajectory.joint_names;
    std::vector<double> joint_positions(point.positions.begin(), point.positions.end());

    // Clone the robot state to avoid modifying the original
    moveit::core::RobotState state(robot_state);

    // Update link transforms to ensure they are valid
    state.updateLinkTransforms();

    // Set the joint positions in the cloned state
    state.setVariablePositions(joint_names, joint_positions);

    // Get the pose of the TCP frame
    pose = getPoseFromRobotState(state, link_frame);

    return pose;
}

double MoveGroupPlanner::computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory,
                                                  const manymove_msgs::msg::MovementConfig &config) const
{
    // Compute max Cartesian speed by analyzing consecutive waypoints
    if (trajectory->getWayPointCount() < 2)
        return 0.0;
    double max_speed = 0.0;
    for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
    {
        Eigen::Isometry3d prev_pose = trajectory->getWayPoint(i - 1).getGlobalLinkTransform(config.tcp_frame);
        Eigen::Isometry3d curr_pose = trajectory->getWayPoint(i).getGlobalLinkTransform(config.tcp_frame);

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
        double max_speed = computeMaxCartesianSpeed(robot_traj_ptr, config);
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
            RCLCPP_INFO(logger_, "Adjusting cartesian speed from %.2f to <= %.2f. Reducing velocity scale...",
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
        move_group_interface_->setPlannerId(cfg.planner_id); // cfg.planning_pipeline + "/" + cfg.planner_id);
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
            move_group_interface_->setPoseTarget(goal_msg.goal.pose_target, goal_msg.goal.config.tcp_frame);
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
                // // Calculate the path lenght and choose the shortest
                // double length = computePathLength(plan.trajectory_);
                // trajectories.emplace_back(plan.trajectory_, length);

                // Time parametrize the path and choose the fastest:
                auto [ok, timed_traj] = applyTimeParameterization(plan.trajectory_, cfg);
                if (!ok)
                {
                    RCLCPP_WARN(logger_, "Time‑parameterization failed on this candidate, skipping.");
                }
                else
                {
                    // grab the very last point's time_from_start
                    const auto &pts = timed_traj.joint_trajectory.points;
                    if (!pts.empty())
                    {
                        // convert to seconds
                        double duration = rclcpp::Duration(pts.back().time_from_start).seconds();
                        trajectories.emplace_back(timed_traj, duration);
                    }
                }
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
                // // Calculate the path lenght and choose the shortest
                // double length = computePathLength(plan.trajectory_);
                // trajectories.emplace_back(plan.trajectory_, length);

                // Time parametrize the path and choose the fastest:
                if (fraction >= 1.0)
                {
                    // double length = computePathLength(plan.trajectory_);
                    // trajectories.emplace_back(plan.trajectory_, length);

                    auto [ok, timed_traj] = applyTimeParameterization(plan.trajectory_, cfg);
                    if (ok && !timed_traj.joint_trajectory.points.empty())
                    {
                        double duration =
                            rclcpp::Duration(timed_traj.joint_trajectory.points.back().time_from_start)
                                .seconds();
                        trajectories.emplace_back(timed_traj, duration);
                    }
                    else
                    {
                        RCLCPP_WARN(logger_, "Time‑param failed on Cartesian candidate.");
                    }
                }
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

bool MoveGroupPlanner::sendControlledStop(const manymove_msgs::msg::MovementConfig &move_cfg,
                                          const moveit_msgs::msg::RobotTrajectory &running_traj,
                                          double elapsed_s)
{
    RCLCPP_INFO(logger_, "Controlled stop: decel=%.2f  elapsed=%.2f.", move_cfg.deceleration_time, elapsed_s);

    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory server unavailable.");
        return false;
    }

    /* -------------------------------------------------------------
     * 1)  Decide target joint positions
     * ------------------------------------------------------------*/
    std::vector<double> target_q;

    target_q = move_group_interface_->getCurrentJointValues();
    if (target_q.empty())
    {
        RCLCPP_ERROR(logger_, "Failed to read current joint state.");
        return false;
    }

    /* -------------------------------------------------------------
     * 2)  Check if remaining time in trajectory is lower than deceleration time
     *     If yes, return success as the motion will stop naturally before deceleration
     * ------------------------------------------------------------*/
    moveit_msgs::msg::RobotTrajectory truncated_traj = running_traj;

    // Get the last point in the trajectory
    const auto &last_point = truncated_traj.joint_trajectory.points.back();
    double remaining_time = rclcpp::Duration(last_point.time_from_start).seconds() - elapsed_s;

    // If remaining time is less than the deceleration time, the motion will stop naturally
    if (remaining_time < move_cfg.deceleration_time)
    {
        RCLCPP_INFO(logger_, "Remaining time in trajectory is less than deceleration time. Stopping motion naturally.");
        return true; // Do nothing and succeed as the motion will stop naturally
    }

    /* -------------------------------------------------------------
     * 3)  Truncate the original trajectory and offset the time_from_start
     *     for each point (negative time)
     * ------------------------------------------------------------*/
    // Remove the past points up to the current time (elapsed_s)
    auto &points = truncated_traj.joint_trajectory.points;
    points.erase(std::remove_if(points.begin(), points.end(),
                                [elapsed_s](const trajectory_msgs::msg::JointTrajectoryPoint &point)
                                {
                                    return rclcpp::Duration(point.time_from_start).seconds() >= elapsed_s;
                                }),
                 points.end());

    // Offset time_from_start to be negative for all points
    for (auto &point : points)
    {
        point.time_from_start = rclcpp::Duration::from_seconds(rclcpp::Duration(point.time_from_start).seconds() - elapsed_s);
    }

    // Add the stop point to the trajectory
    trajectory_msgs::msg::JointTrajectoryPoint stop_point;
    stop_point.positions = target_q;
    stop_point.velocities.assign(target_q.size(), 0.0);
    stop_point.accelerations.assign(target_q.size(), 0.0);
    stop_point.time_from_start = rclcpp::Duration::from_seconds(move_cfg.deceleration_time);
    truncated_traj.joint_trajectory.points.push_back(stop_point);

    // Send the modified trajectory
    control_msgs::action::FollowJointTrajectory::Goal stop_goal;
    stop_goal.trajectory.joint_names = move_group_interface_->getJointNames();
    stop_goal.trajectory.points = truncated_traj.joint_trajectory.points;

    auto gh_fut = follow_joint_traj_client_->async_send_goal(stop_goal);
    if (gh_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Timeout while sending stop goal.");
        return false;
    }
    auto gh = gh_fut.get();
    if (!gh)
    {
        RCLCPP_ERROR(logger_, "Stop goal rejected by controller.");
        return false;
    }

    const double timeout_s = std::max(2.0 * move_cfg.deceleration_time, 5.0);
    auto res_fut = follow_joint_traj_client_->async_get_result(gh);

    if (res_fut.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Stop goal timed-out (%.2f s).", timeout_s);
        return false;
    }

    auto result = res_fut.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(logger_, "Controlled stop completed successfully.");

    return result.code == rclcpp_action::ResultCode::SUCCEEDED;
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
                                              const manymove_msgs::msg::MoveManipulatorGoal &move_request,
                                              const std::vector<double> &current_joint_state) const
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
        if (std::fabs(first_point.positions[i] - current_joint_state[i]) > move_request.config.rotational_precision)
        {
            RCLCPP_INFO(logger_, "Joint %zu difference (%.6f) exceeds tolerance (%.6f).",
                        i, std::fabs(first_point.positions[i] - current_joint_state[i]), move_request.config.rotational_precision);
            return false;
        }
    }
    return true;
}

bool MoveGroupPlanner::isTrajectoryEndValid(
    const moveit_msgs::msg::RobotTrajectory &traj,
    const manymove_msgs::msg::MoveManipulatorGoal &move_request) const
{
    // Check that the trajectory is not empty.
    if (traj.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(logger_, "Trajectory is empty. Cannot validate end.");
        return false;
    }

    auto const current_state = *move_group_interface_->getCurrentState();

    // For Cartesian/pose-type movements, compare the computed end pose to the target pose.
    if (move_request.movement_type == "pose" || move_request.movement_type == "cartesian")
    {
        geometry_msgs::msg::Pose traj_end_pose;
        try
        {
            // Get the pose from the last point of the trajectory.
            traj_end_pose = getPoseFromTrajectory(traj, current_state, move_request.config.tcp_frame, true);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Error extracting trajectory end pose: %s", e.what());
            return false;
        }

        double distance = computeCartesianDistance(traj_end_pose, move_request.pose_target);
        if (distance > move_request.config.linear_precision)
        {
            RCLCPP_INFO(logger_,
                        "Trajectory end pose invalid: Euclidean distance (%.6f) exceeds tolerance (%.6f)",
                        distance, move_request.config.linear_precision);
            return false;
        }
        return true;
    }
    // For joint or named target movements, compare the joint positions.
    else if (move_request.movement_type == "joint" || move_request.movement_type == "named")
    {
        const auto &last_point = traj.joint_trajectory.points.back();
        std::vector<double> target_joint_values;
        if (move_request.movement_type == "named")
        {
            // getNamedTargetValues returns a map<string, double>.
            auto named_map = move_group_interface_->getNamedTargetValues(move_request.named_target);
            // Use the trajectory's joint_names order to extract joint values.
            for (const auto &joint_name : traj.joint_trajectory.joint_names)
            {
                auto it = named_map.find(joint_name);
                if (it != named_map.end())
                {
                    target_joint_values.push_back(it->second);
                }
                else
                {
                    RCLCPP_ERROR(logger_, "Joint '%s' not found in named target values for '%s'.",
                                 joint_name.c_str(), move_request.named_target.c_str());
                    return false;
                }
            }
        }
        else
        {
            target_joint_values = move_request.joint_values;
        }

        if (last_point.positions.size() != target_joint_values.size())
        {
            RCLCPP_ERROR(logger_,
                         "Mismatch: trajectory joints (%zu) vs. target joints (%zu).",
                         last_point.positions.size(), target_joint_values.size());
            return false;
        }

        for (size_t i = 0; i < last_point.positions.size(); ++i)
        {
            double diff = std::fabs(last_point.positions[i] - target_joint_values[i]);
            if (diff > move_request.config.rotational_precision)
            {
                RCLCPP_INFO(logger_,
                            "Joint %zu difference (%.6f) exceeds tolerance (%.6f) for end validation.",
                            i, diff, move_request.config.rotational_precision);
                return false;
            }
        }
        return true;
    }
    else
    {
        // If movement_type is unrecognized, warn and allow the trajectory.
        RCLCPP_WARN(logger_,
                    "Unknown movement type '%s'; skipping end validation.",
                    move_request.movement_type.c_str());
        return true;
    }
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

bool MoveGroupPlanner::isTrajectoryValid(
    const robot_trajectory::RobotTrajectory &trajectory,
    const moveit_msgs::msg::Constraints &path_constraints,
    const double time_from_start) const
{
    robot_trajectory::RobotTrajectory sub_traj(trajectory.getRobotModel(),
                                               trajectory.getGroupName());

    // If (time_from_start > 0) only check the trajectory after that time from start
    if (time_from_start > 0.0)
    {
        // indices bracketing   time_from_start   in the original trajectory
        int before = -1, after = -1;
        double blend = 0.0;
        trajectory.findWayPointIndicesForDurationAfterStart(
            time_from_start, before, after, blend);

        if (after < static_cast<int>(trajectory.getWayPointCount()))
        {
            // copy way-points [after … end) into  sub_traj
            for (std::size_t i = static_cast<std::size_t>(after);
                 i < trajectory.getWayPointCount(); ++i)
            {
                sub_traj.addSuffixWayPoint(trajectory.getWayPoint(i),
                                           trajectory.getWayPointDurationFromPrevious(i));
            }
        }
    }
    else
    {
        // we can use a cheap deep-copy constructor here
        sub_traj = trajectory;
    }

    // Get a lock on the planning scene through the planning scene monitor.
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
    if (!lscene)
    {
        RCLCPP_ERROR(logger_, "PlanningSceneMonitor is not available in isTrajectoryValid().");
        return false;
    }

    // Delegate the validity check to the PlanningScene's isPathValid method.
    // Note that the isPathValid overload taking a robot_trajectory::RobotTrajectory,
    // constraints, group name, verbosity flag, and an optional invalid index vector
    // iterates over each waypoint and performs collision/constraint checking.
    return lscene->isPathValid(trajectory, path_constraints, planning_group_, /*verbose*/ false, /*invalid_index*/ nullptr);
}

bool MoveGroupPlanner::isTrajectoryValid(
    const trajectory_msgs::msg::JointTrajectory &joint_traj_msg,
    const moveit_msgs::msg::Constraints &path_constraints,
    const double time_from_start) const
{
    trajectory_msgs::msg::JointTrajectory jt = joint_traj_msg;

    // If (time_from_start > 0) only check the trajectory after that time from start
    if (time_from_start > 0.0)
    {
        auto first_after = std::find_if(
            jt.points.begin(), jt.points.end(),
            [time_from_start](const auto &pt)
            {
                return rclcpp::Duration(pt.time_from_start).seconds() > time_from_start;
            });

        jt.points.erase(jt.points.begin(), first_after);

        // if nothing is left, there is nothing to validate
        if (jt.points.empty())
            return true;
    }

    // 1) Lock the planning scene
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
    if (!lscene)
    {
        RCLCPP_ERROR(logger_, "Failed to lock the PlanningScene in isTrajectoryValid");
        return false;
    }

    // 2) Get the RobotModel
    auto robot_model_ptr = move_group_interface_->getRobotModel();
    if (!robot_model_ptr)
    {
        RCLCPP_ERROR(logger_, "Robot model is null in isTrajectoryValid");
        return false;
    }

    // 3) Copy the *scene’s* current RobotState (including any attached objects!)
    //    so that we preserve the attached bodies.
    moveit::core::RobotState local_state(lscene->getCurrentState());

    // 4) Override just the joint angles from your current_positions_ map
    {
        std::lock_guard<std::mutex> lock(js_mutex_);

        for (const auto &joint_name : joint_traj_msg.joint_names)
        {
            auto it = current_positions_.find(joint_name);
            if (it != current_positions_.end())
            {
                local_state.setVariablePosition(joint_name, it->second);
            }
            else
            {
                RCLCPP_WARN(logger_,
                            "Joint '%s' not found in current_positions_. Using default value instead.",
                            joint_name.c_str());
            }
        }
    }

    // Make sure transforms are up to date
    local_state.update();

    // 5) Convert your trajectory message to a RobotTrajectory
    moveit_msgs::msg::RobotTrajectory rt_msg;
    rt_msg.joint_trajectory = joint_traj_msg;

    auto robot_traj_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(
        robot_model_ptr, planning_group_);

    robot_traj_ptr->setRobotTrajectoryMsg(local_state, rt_msg);

    // 6) Finally, let the PlanningScene do the path validity check.
    //    Because 'local_state' came from the scene, it already has the attached object.
    bool valid = lscene->isPathValid(*robot_traj_ptr, path_constraints, planning_group_, /*verbose*/ false, /*invalid_index*/ nullptr);

    return valid;
}
