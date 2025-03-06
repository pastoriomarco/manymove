#include "manymove_planner/moveit_cpp_planner.hpp"

using manymove_msgs::msg::MovementConfig;
using namespace std::chrono_literals;

MoveItCppPlanner::MoveItCppPlanner(
    const rclcpp::Node::SharedPtr &node,
    const std::string &planning_group,
    const std::string &base_frame,
    const std::string &tcp_frame,
    const std::string &traj_controller,
    const std::shared_ptr<moveit_cpp::MoveItCpp> &moveit_cpp_ptr)
    : node_(node), logger_(node->get_logger()),
      planning_group_(planning_group),
      base_frame_(base_frame), tcp_frame_(tcp_frame),
      traj_controller_(traj_controller),
      moveit_cpp_ptr_(moveit_cpp_ptr)
{
    if (!moveit_cpp_ptr_)
    {
        moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
        moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();
    }

    /**
     * The following functions were used to publish the standard topics and services needed by the manymove_object_manager package.
     * Correctly configuring /config/moveit_cpp.yaml and the package launchers seems to offer the same results, while adding
     * these lines seems to compromise the package functionality in some contexts: for example, inside the current
     * NVIDIA Isaac ROS 3.1/3.2 docker container for Isaac Manipulator.
     */
    // // moveit_cpp_ptr_->getPlanningSceneMonitor()->requestPlanningSceneState("get_planning_scene");
    // moveit_cpp_ptr_->getPlanningSceneMonitor()->startSceneMonitor();
    // moveit_cpp_ptr_->getPlanningSceneMonitor()->startStateMonitor();
    // moveit_cpp_ptr_->getPlanningSceneMonitor()->startWorldGeometryMonitor("/collision_object", "/attached_collision_object", true);
    // // moveit_cpp_ptr_->getPlanningSceneMonitor()->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_GEOMETRY);
    // moveit_cpp_ptr_->getPlanningSceneMonitor()->monitorDiffs(true);

    planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_, moveit_cpp_ptr_);
    RCLCPP_INFO(logger_, "===================================================");
    RCLCPP_INFO(logger_, "MoveItCppPlanner initialized with group: %s", planning_group_.c_str());

    plan_parameters_.load(node_);

    RCLCPP_INFO(logger_, "===================================================");
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.max_acceleration_scaling_factor: " << plan_parameters_.max_acceleration_scaling_factor);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.max_velocity_scaling_factor: " << plan_parameters_.max_velocity_scaling_factor);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.planner_id: " << plan_parameters_.planner_id);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.planning_attempts: " << plan_parameters_.planning_attempts);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.planning_pipeline: " << plan_parameters_.planning_pipeline);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.planning_time: " << plan_parameters_.planning_time);
    RCLCPP_INFO(logger_, "===================================================");

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
        std::bind(&MoveItCppPlanner::jointStateCallback, this, std::placeholders::_1));

    // For safety, initialize the maps as empty
    current_positions_.clear();
    current_velocities_.clear();
}

// functions to let ExecuteTrajectory in action_server.cpp handle the collision check feedback
rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr MoveItCppPlanner::getFollowJointTrajClient() const
{
    return follow_joint_traj_client_;
}

// Compute Path Length
double MoveItCppPlanner::computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const
{
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_WARN(logger_, "Joint trajectory is empty. Path length is zero.");
        return 0.0;
    }

    // Helper to compute joint-space path length
    auto computeJointPathLength = [&](const moveit_msgs::msg::RobotTrajectory &traj) -> double
    {
        double length = 0.0;
        for (size_t i = 1; i < traj.joint_trajectory.points.size(); ++i)
        {
            const auto &prev_point = traj.joint_trajectory.points[i - 1];
            const auto &curr_point = traj.joint_trajectory.points[i];

            // Ensure joint positions are valid
            if (prev_point.positions.size() != curr_point.positions.size())
            {
                RCLCPP_ERROR(logger_, "Mismatch in joint positions size at trajectory points %zu and %zu.", i - 1, i);
                return 0.0;
            }

            double segment_length = 0.0;
            for (size_t j = 0; j < prev_point.positions.size(); ++j)
            {
                double diff = curr_point.positions[j] - prev_point.positions[j];
                segment_length += diff * diff;
            }
            length += std::sqrt(segment_length);
        }

        return length;
    };

    // Helper to compute Cartesian path length using TCP pose
    auto computeCartesianPathLength = [&](const moveit_msgs::msg::RobotTrajectory &traj) -> double
    {
        double length = 0.0;

        // Access the robot model
        auto robot_model = moveit_cpp_ptr_->getRobotModel();
        if (!robot_model)
        {
            RCLCPP_ERROR(logger_, "Robot model is null.");
            return 0.0;
        }

        // Create a robot state
        moveit::core::RobotState robot_state(robot_model);
        const auto &joint_model_group = robot_model->getJointModelGroup(planning_group_);
        if (!joint_model_group)
        {
            RCLCPP_ERROR(logger_, "Invalid joint model group.");
            return 0.0;
        }

        for (size_t i = 1; i < traj.joint_trajectory.points.size(); ++i)
        {
            // Set the previous and current joint values
            const auto &prev_point = traj.joint_trajectory.points[i - 1];
            const auto &curr_point = traj.joint_trajectory.points[i];

            robot_state.setJointGroupPositions(joint_model_group, prev_point.positions);
            const Eigen::Isometry3d prev_tcp_pose = robot_state.getGlobalLinkTransform(tcp_frame_);

            robot_state.setJointGroupPositions(joint_model_group, curr_point.positions);
            const Eigen::Isometry3d curr_tcp_pose = robot_state.getGlobalLinkTransform(tcp_frame_);

            // Calculate the Cartesian distance
            Eigen::Vector3d prev_position = prev_tcp_pose.translation();
            Eigen::Vector3d curr_position = curr_tcp_pose.translation();

            length += (curr_position - prev_position).norm();
        }

        return length;
    };

    // Compute both joint and Cartesian path lengths
    double joint_length = computeJointPathLength(trajectory);
    double cart_length = computeCartesianPathLength(trajectory);

    // Combine lengths (example: weight them as desired)
    double total_length = joint_length + (2 * cart_length);

    return total_length;
}

// Function to get a geometry_msgs::msg::Pose from a RobotState and frame
geometry_msgs::msg::Pose MoveItCppPlanner::getPoseFromRobotState(const moveit::core::RobotState &robot_state,
                                                                 const std::string &link_frame)
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
double MoveItCppPlanner::computeCartesianDistance(const geometry_msgs::msg::Pose &start_pose,
                                                  const geometry_msgs::msg::Pose &target_pose)
{
    // Compute the Euclidean distance to the target pose
    double dx = target_pose.position.x - start_pose.position.x;
    double dy = target_pose.position.y - start_pose.position.y;
    double dz = target_pose.position.z - start_pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Function to get a pose from a trajectory and TCP frame
geometry_msgs::msg::Pose MoveItCppPlanner::getPoseFromTrajectory(const moveit_msgs::msg::RobotTrajectory &traj_msg,
                                                                 const moveit::core::RobotState &robot_state,
                                                                 const std::string &link_frame,
                                                                 bool use_last_point)
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

// Compute Max Cartesian Speed
double MoveItCppPlanner::computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const
{
    if (trajectory->getWayPointCount() < 2)
        return 0.0;
    double max_speed = 0.0;
    for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
    {
        Eigen::Isometry3d prev_pose = trajectory->getWayPoint(i - 1).getGlobalLinkTransform(tcp_frame_);
        Eigen::Isometry3d curr_pose = trajectory->getWayPoint(i).getGlobalLinkTransform(tcp_frame_);
        double dist = (curr_pose.translation() - prev_pose.translation()).norm();
        double dt = trajectory->getWayPointDurationFromPrevious(i);
        if (dt > 0.0)
        {
            double speed = dist / dt;
            if (speed > max_speed)
                max_speed = speed;
        }
    }
    return max_speed;
}

bool MoveItCppPlanner::areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const
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

moveit_msgs::msg::RobotTrajectory MoveItCppPlanner::convertToMsg(const robot_trajectory::RobotTrajectory &trajectory) const
{
    moveit_msgs::msg::RobotTrajectory traj_msg;
    trajectory.getRobotTrajectoryMsg(traj_msg);
    return traj_msg;
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveItCppPlanner::plan(const manymove_msgs::action::PlanManipulator::Goal &goal_msg)
{
    std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;
    auto robot_model_ptr = moveit_cpp_ptr_->getRobotModel();
    auto robot_start_state_ptr = planning_components_->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(planning_group_);

    // Handle start state: if there is a start joint position we use that one to plan, otherwise we use the current one.
    if (!goal_msg.goal.start_joint_values.empty())
    {
        moveit::core::RobotState start_state(*moveit_cpp_ptr_->getCurrentState());
        start_state.setJointGroupPositions(joint_model_group_ptr, goal_msg.goal.start_joint_values);
        planning_components_->setStartState(start_state);
    }
    else
    {
        planning_components_->setStartStateToCurrentState();
    }

    /** Create PlanRequestParameters and set scaling factors
     * moveit_cpp::PlanningComponent::PlanRequestParameters params;
     * params.planning_pipeline = "ompl"; // or the name of your pipeline from your moveit_cpp.yaml
     * params.max_velocity_scaling_factor = goal_msg.goal.config.velocity_scaling_factor;
     * params.max_acceleration_scaling_factor = goal_msg.goal.config.acceleration_scaling_factor;
     * IMPORTANT NOTE: You can set other parameters in `params` as needed, but remember that if you
     * set even one parameter you will have to set all of them. When I last tried, setting one
     * parameter had as consequence that none of the default parameters is used anymore.
     */

    // Set movement targets
    if ((goal_msg.goal.movement_type == "joint") || (goal_msg.goal.movement_type == "named"))
    {
        bool goal_valid = true;
        if (goal_msg.goal.movement_type == "joint")
        {
            RCLCPP_DEBUG_STREAM(logger_, "Setting joint target");
            moveit::core::RobotState goal_state(robot_model_ptr);
            goal_state.setJointGroupPositions(joint_model_group_ptr, goal_msg.goal.joint_values);
            planning_components_->setGoal(goal_state);
        }
        else if (goal_msg.goal.movement_type == "named")
        {
            RCLCPP_DEBUG_STREAM(logger_, "Setting named target " << goal_msg.goal.named_target);
            planning_components_->setGoal(goal_msg.goal.named_target);

            // Retrieve the list of available named targets
            const auto available_targets = planning_components_->getNamedTargetStates();
            // Check if the specified named target is available
            if (std::find(available_targets.begin(), available_targets.end(), goal_msg.goal.named_target) != available_targets.end())
            {
                // Retrieve and print the joint values for the named target
                auto joint_values = planning_components_->getNamedTargetStateValues(goal_msg.goal.named_target);
                RCLCPP_DEBUG(logger_, "Joint values for named target '%s':", goal_msg.goal.named_target.c_str());
                for (const auto &entry : joint_values)
                {
                    RCLCPP_DEBUG(logger_, "  %s: %f", entry.first.c_str(), entry.second);
                }
            }
            else
            {
                RCLCPP_ERROR(logger_, "Named target '%s' not found among available targets.", goal_msg.goal.named_target.c_str());
                goal_valid = false;
            }
        }

        // Plan multiple trajectories using the plan(params) method
        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target &&
               goal_valid)
        {
            auto solution = planning_components_->plan();
            if (solution)
            {
                moveit_msgs::msg::RobotTrajectory traj_msg;
                solution.trajectory->getRobotTrajectoryMsg(traj_msg);

                // We compute the traj length only if the traj is valid
                double length = computePathLength(traj_msg);

                trajectories.emplace_back(traj_msg, length);
                RCLCPP_INFO_STREAM(logger_, "Calculated " << goal_msg.goal.movement_type << " traj length: " << length);
            }
            else
            {
                RCLCPP_WARN(logger_, "%s target planning attempt %d failed.",
                            goal_msg.goal.movement_type.c_str(), attempts + 1);
            }
            attempts++;
        }
    }
    else if (goal_msg.goal.movement_type == "pose")
    {

        RCLCPP_DEBUG_STREAM(logger_, "Setting pose target for " << tcp_frame_);
        // geometry_msgs::msg::PoseStamped target_pose;
        // target_pose.pose = goal_msg.goal.pose_target;
        // target_pose.header.frame_id = tcp_frame_;
        // planning_components_->setGoal(target_pose, tcp_frame_);

        // Plan multiple trajectories using the plan(params) method
        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {

            auto target_state = *robot_start_state_ptr;
            target_state.setFromIK(joint_model_group_ptr, goal_msg.goal.pose_target);
            planning_components_->setGoal(target_state);

            auto solution = planning_components_->plan();

            if (solution && solution.error_code == moveit::core::MoveItErrorCode::SUCCESS)
            {
                moveit_msgs::msg::RobotTrajectory traj_msg;
                solution.trajectory->getRobotTrajectoryMsg(traj_msg);

                // Check if traj is empty
                if (traj_msg.joint_trajectory.points.empty())
                {
                    RCLCPP_WARN(logger_, "%s target planning attempt %d failed: trajectory is empty.",
                                goal_msg.goal.movement_type.c_str(), attempts + 1);
                    break; // the traj is empty
                }

                // I'm having trouble with plans succeeding with empty or almost empty trajectories
                // The following checks are to verify that the trajectory makes sense

                // auto start_pose = getPoseFromRobotState(*robot_start_state_ptr, tcp_frame_);
                // auto traj_start_pose = getPoseFromTrajectory(traj_msg, *robot_start_state_ptr, tcp_frame_, false);
                auto traj_end_pose = getPoseFromTrajectory(traj_msg, *robot_start_state_ptr, tcp_frame_, true);

                // double min_euclidean_distance = computeCartesianDistance(start_pose, goal_msg.goal.pose_target);
                // double traj_euclidean_distance = computeCartesianDistance(traj_start_pose, traj_end_pose);
                // double starts_euclidean_distance = computeCartesianDistance(start_pose, traj_start_pose);
                double targets_euclidean_distance = computeCartesianDistance(traj_end_pose, goal_msg.goal.pose_target);

                // RCLCPP_INFO_STREAM(logger_, "Minimum theoretical eclidean distance between start pose and target pose: " << min_euclidean_distance);
                // RCLCPP_INFO_STREAM(logger_, "Euclidean distance between trajectory first point and trajectory last point: " << traj_euclidean_distance);
                // RCLCPP_INFO_STREAM(logger_, "Eclidean distance between theoretical start and calculated trajectory first point: " << starts_euclidean_distance);

                double traj_tolerance = 0.001;

                if (targets_euclidean_distance < traj_tolerance)
                {
                    // Only compute path length if the traj is valid
                    double length = computePathLength(traj_msg);

                    trajectories.emplace_back(traj_msg, length);
                    RCLCPP_DEBUG_STREAM(logger_, "Calculated pose traj length: " << length);
                    RCLCPP_DEBUG(logger_, "moveit::core::MoveItErrorCode = %d.",
                                 solution.error_code.val);
                }
                else
                {
                    RCLCPP_WARN_STREAM(logger_, "Eclidean distance between theoretical target and calculated trajectory last point: " << targets_euclidean_distance);

                    RCLCPP_WARN_STREAM(logger_, "The planner was not able to calculate trajectory with end point within tolerance." << targets_euclidean_distance);
                    RCLCPP_WARN(logger_, "%s target planning attempt %d failed: trajectory is empty.",
                                goal_msg.goal.movement_type.c_str(), attempts + 1);
                }
            }
            else
            {
                RCLCPP_WARN(logger_, "%s target planning attempt %d failed.",
                            goal_msg.goal.movement_type.c_str(), attempts + 1);
                RCLCPP_WARN(logger_, "moveit::core::MoveItErrorCode = %d.",
                            solution.error_code.val);
            }
            attempts++;
        }
    }
    else if (goal_msg.goal.movement_type == "cartesian")
    {

        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {
            RCLCPP_DEBUG(logger_, "Cartesian path planning attempt %d with step size %.3f, jump threshold %.3f",
                         attempts + 1, goal_msg.goal.config.step_size, goal_msg.goal.config.jump_threshold);

            // Handle start state
            if (!goal_msg.goal.start_joint_values.empty())
            {
                moveit::core::RobotState start_state(*moveit_cpp_ptr_->getCurrentState());
                start_state.setJointGroupPositions(joint_model_group_ptr, goal_msg.goal.start_joint_values);
                planning_components_->setStartState(start_state);
            }
            else
            {
                planning_components_->setStartStateToCurrentState();
            }

            // Get the initial robot state
            auto start_state = planning_components_->getStartState();

            // Get the end-effector link model
            const moveit::core::LinkModel *ee_link = joint_model_group_ptr->getLinkModel(tcp_frame_);

            // Retrieve the global pose of the end-effector at the start
            Eigen::Isometry3d start_pose = start_state->getGlobalLinkTransform(ee_link);
            // Assume that waypoints is filled with one or more geometry_msgs::msg::Pose
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(goal_msg.goal.pose_target);

            // Convert geometry_msgs::msg::Pose to Eigen::Isometry3d
            EigenSTL::vector_Isometry3d eigen_waypoints;
            eigen_waypoints.push_back(start_pose);

            for (const auto &wp : waypoints)
            {
                Eigen::Isometry3d eigen_pose;
                tf2::fromMsg(wp, eigen_pose);
                eigen_waypoints.push_back(eigen_pose);
            }

            std::vector<moveit::core::RobotStatePtr> trajectory_states;

            // Creating the callback to check for collisions on cartesian path
            moveit::core::GroupStateValidityCallbackFn custom_callback =
                std::bind(&MoveItCppPlanner::isStateValid, this, std::placeholders::_1, std::placeholders::_2);

            double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
                start_state.get(),
                joint_model_group_ptr,
                trajectory_states,
                ee_link,
                eigen_waypoints,
                true,
                moveit::core::MaxEEFStep(goal_msg.goal.config.step_size),
                moveit::core::JumpThreshold(goal_msg.goal.config.jump_threshold),
                custom_callback,
                kinematics::KinematicsQueryOptions(),
                nullptr);

            if (fraction >= 1.0)
            {

                RCLCPP_DEBUG_STREAM(logger_, "trajectory_states length: " << trajectory_states.size());
                // Construct a RobotTrajectory from the computed states
                robot_trajectory::RobotTrajectory robot_trajectory(robot_model_ptr, planning_group_);
                for (const auto &state : trajectory_states)
                {
                    robot_trajectory.addSuffixWayPoint(*state, 0.0);
                }

                // Convert to message
                moveit_msgs::msg::RobotTrajectory traj_msg = convertToMsg(robot_trajectory);
                double length = computePathLength(traj_msg);
                trajectories.emplace_back(traj_msg, length);
                RCLCPP_DEBUG_STREAM(logger_, "Calculated traj length: " << length);
            }
            else
            {
                RCLCPP_WARN(logger_, "Cartesian path planning attempt %d failed (%.2f%% achieved)",
                            attempts + 1, fraction * 100.0);
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

std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveItCppPlanner::applyTimeParameterization(
    const moveit_msgs::msg::RobotTrajectory &input_traj,
    const manymove_msgs::msg::MovementConfig &config)
{
    // 1) Basic checks
    if (input_traj.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(logger_, "Cannot parameterize an empty trajectory.");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    auto robot_model_ptr = moveit_cpp_ptr_->getRobotModel();
    if (!robot_model_ptr)
    {
        RCLCPP_ERROR(logger_, "Robot model is null in applyTimeParameterization");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    // 2) Convert input message → RobotTrajectory
    auto robot_traj_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(
        robot_model_ptr,
        planning_components_->getPlanningGroupName());

    auto current_state = moveit_cpp_ptr_->getCurrentState();
    if (!current_state)
    {
        RCLCPP_ERROR(logger_, "No current state in applyTimeParameterization");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    // Load the input_traj into robot_traj_ptr
    robot_traj_ptr->setRobotTrajectoryMsg(*current_state, input_traj);

    // 3) Time parameterization loop
    //    We'll do the TOTG or iterative parabolic, etc. and also clamp cartesian speed
    double velocity_scaling_factor = config.velocity_scaling_factor;
    double acceleration_scaling_factor = config.acceleration_scaling_factor;

    /// TODO: very if this many iterations are needed:
    const int max_iterations = 32;
    for (int iteration = 0; iteration < max_iterations; ++iteration)
    {
        // Reset durations for all waypoints after [0]
        for (size_t i = 1; i < robot_traj_ptr->getWayPointCount(); i++)
        {
            robot_traj_ptr->setWayPointDurationFromPrevious(i, 0.0);
        }

        // Choose the smoothing method based on config
        bool time_param_success = false;
        if (config.smoothing_type == "time_optimal")
        {
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*robot_traj_ptr,
                                                              velocity_scaling_factor,
                                                              acceleration_scaling_factor);
        }
        /// TODO: Is ruckig still developed? Is it worth keeping since it doesn't work on cartesian paths? To test.
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
            // Attempt fallback if first try fails, try TOTG (again)
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

    // If we exit the loop, we never found a speed < config.max_cartesian_speed
    RCLCPP_ERROR(logger_, "Failed to limit Cartesian speed after multiple iterations.");
    return {false, moveit_msgs::msg::RobotTrajectory()};
}

bool MoveItCppPlanner::executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    // (1) Basic checks
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(logger_, "Empty trajectory, aborting (MoveItCppPlanner).");
        return false;
    }

    if (!follow_joint_traj_client_)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action client not initialized! (MoveItCppPlanner)");
        return false;
    }

    // (2) Wait for the action server
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory server not available (MoveItCppPlanner).");
        return false;
    }

    // (3) Create the goal
    control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = trajectory.joint_trajectory;

    RCLCPP_INFO(logger_, "Sending FollowJointTrajectory goal (MoveItCppPlanner) ...");

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
        RCLCPP_DEBUG(logger_, "Partial execution (MoveItCppPlanner): time_from_start %.2f",
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
            RCLCPP_INFO(logger_, "FollowJointTrajectory succeeded (MoveItCppPlanner).");
            success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger_, "FollowJointTrajectory was aborted (MoveItCppPlanner).");
            success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(logger_, "FollowJointTrajectory canceled (MoveItCppPlanner).");
            success = false;
            break;
        default:
            RCLCPP_ERROR(logger_, "Unknown result from FollowJointTrajectory (MoveItCppPlanner).");
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
        RCLCPP_ERROR(logger_, "Goal handle not received in time (MoveItCppPlanner).");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory goal rejected (MoveItCppPlanner).");
        return false;
    }

    // (8) Wait for the result
    RCLCPP_INFO(logger_, "Waiting for FollowJointTrajectory result (MoveItCppPlanner) ...");
    auto status = result_future.wait_for(std::chrono::seconds(300));
    if (status != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution timed out (MoveItCppPlanner).");
        return false;
    }

    bool exec_success = result_future.get();
    if (!exec_success)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution failed (MoveItCppPlanner).");
        return false;
    }

    RCLCPP_INFO(logger_, "Trajectory execution succeeded (MoveItCppPlanner).");
    return true;
}

bool MoveItCppPlanner::isStateValid(const moveit::core::RobotState *state,
                                    const moveit::core::JointModelGroup *group) const
{
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    collision_request.group_name = group->getName();
    collision_request.max_contacts = 1;

    auto psm = moveit_cpp_ptr_->getPlanningSceneMonitor();
    if (!psm)
    {
        RCLCPP_ERROR(logger_, "PlanningSceneMonitor is null.");
        return false;
    }

    planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm);
    if (!locked_scene)
    {
        RCLCPP_ERROR(logger_, "LockedPlanningSceneRO is null.");
        return false;
    }

    // IMPORTANT: DO NOT overwrite positions! Just update transforms.
    moveit::core::RobotState temp_state(*state);
    temp_state.update(); // ONLY THIS

    locked_scene->checkCollision(collision_request, collision_result, temp_state);

    if (collision_result.collision)
    {
        RCLCPP_WARN(logger_, "Collision detected for group '%s'.", group->getName().c_str());
    }

    return !collision_result.collision;
}

bool MoveItCppPlanner::sendControlledStop(double deceleration_time)
{
    RCLCPP_INFO(logger_, "Constructing a short 'controlled stop' trajectory (%.2fs) [SINGLE-POINT].",
                deceleration_time);

    // 1) Make sure the FollowJointTrajectory action server is up
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(logger_, "Cannot send stop trajectory, FollowJointTrajectory server is not available.");
        return false;
    }

    // 2) Get current robot state
    auto current_state = moveit_cpp_ptr_->getCurrentState();
    if (!current_state)
    {
        RCLCPP_ERROR(logger_, "Failed to get current robot state for stop trajectory.");
        return false;
    }
    const auto &joint_model_group = current_state->getJointModelGroup(planning_group_);
    if (!joint_model_group)
    {
        RCLCPP_ERROR(logger_, "JointModelGroup '%s' not found.", planning_group_.c_str());
        return false;
    }

    // Grab joint names and positions
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    std::vector<double> positions;
    current_state->copyJointGroupPositions(joint_model_group, positions);

    // 3) Build a SINGLE-point trajectory
    control_msgs::action::FollowJointTrajectory::Goal stop_goal;
    stop_goal.trajectory.joint_names = joint_names;

    // We set velocities to zero
    std::vector<double> velocities(positions.size(), 0.0);

    // Only Point: current position, zero velocity: it will "spring-back" to the position it was when the command is issued.
    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.positions = positions;
    p0.velocities = velocities;
    p0.accelerations.resize(positions.size(), 0.0);

    // Use deceleration_time to define how long we give the controller to ramp to zero velocity.
    // A larger deceleration_time will produce a smoother (but slower) stop.
    p0.time_from_start = rclcpp::Duration::from_seconds(deceleration_time);

    stop_goal.trajectory.points.push_back(p0);

    // 4) Send the new "stop" goal
    RCLCPP_INFO(logger_, "Sending single-point 'stop' trajectory.");

    auto send_goal_future = follow_joint_traj_client_->async_send_goal(stop_goal);
    if (send_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Timeout while sending stop trajectory goal.");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "Stop trajectory goal was rejected by the joint trajectory server.");
        return false;
    }

    // Wait for the result
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
        RCLCPP_INFO(logger_, "Single-point stop completed successfully.");
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

void MoveItCppPlanner::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

bool MoveItCppPlanner::isJointStateValid(const std::vector<double> &joint_positions) const
{
    // Create a RobotState from the planner's RobotModel
    auto robot_model = moveit_cpp_ptr_->getRobotModel();
    if (!robot_model)
    {
        RCLCPP_ERROR(logger_, "Robot model is null in isJointStateValid()");
        return false; // or throw
    }

    const moveit::core::JointModelGroup *jmg = robot_model->getJointModelGroup(planning_group_);
    if (!jmg)
    {
        RCLCPP_ERROR(logger_, "JointModelGroup '%s' not found in isJointStateValid().", planning_group_.c_str());
        return false;
    }

    moveit::core::RobotState temp_state(robot_model);
    // set all joints to default or current first
    temp_state.setToDefaultValues();
    temp_state.setJointGroupPositions(jmg, joint_positions);
    temp_state.update();

    // Reuse the existing isStateValid(...) by passing `&temp_state, jmg`
    return isStateValid(&temp_state, jmg);
}

bool MoveItCppPlanner::isTrajectoryStartValid(const moveit_msgs::msg::RobotTrajectory &traj,
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