#pragma once

#include "planner_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <future>
#include <map>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "manymove_msgs/msg/move_manipulator_goal.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <moveit/robot_model/robot_model.h>

/**
 * @class MoveGroupPlanner
 * @brief A planner class integrating with MoveGroupInterface for motion planning in ROS2.
 *
 * This class implements the PlannerInterface using MoveGroupInterface as the
 * underlying planning framework. It provides functionalities for trajectory
 * generation, time parameterization, and optionally execution through a
 * FollowJointTrajectory action server.It provides utilities to handle multiple
 * movement types (joint, pose, named, cartesian).
 */
class MoveGroupPlanner : public PlannerInterface
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    /**
     * @brief Constructor for MoveGroupPlanner.
     * @param node Shared pointer to the ROS2 node.
     * @param planning_group Name of the planning group (as configured in MoveIt).
     * @param base_frame The base frame of the robot.
     * @param tcp_frame The tool center point (TCP) frame for the manipulator.
     * @param traj_controller Name of the trajectory controller to use for execution.
     */
    MoveGroupPlanner(
        const rclcpp::Node::SharedPtr &node,
        const std::string &planning_group,
        const std::string &base_frame,
        const std::string &tcp_frame,
        const std::string &traj_controller);

    /**
     * @brief Default destructor.
     */
    ~MoveGroupPlanner() override = default;

    /**
     * @brief Plan a trajectory for a single goal.
     * @param goal The PlanManipulator goal containing movement type and parameters.
     * @return A pair containing a success flag and the planned robot trajectory.
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_msgs::action::PlanManipulator::Goal &goal) override;

    /**
     * @brief Apply time parameterization to a single trajectory.
     * @param trajectory Pointer to the robot trajectory.
     * @param config Movement configuration specifying scaling factors and smoothing type.
     * @return True if the time parameterization succeeded, false otherwise, plus the resulting RobotTrajectory.
     *
     * @details Most of the industrial and collaborative robots have a maximum cartesian speed over which the robot will perform
     * and emergency stop. Moreover, safety regulations in collaborative applications require the enforcement of maximum cartesian
     * speed limits. While this package is not meant to provide functionalities compliant with safety regulations, most robots
     * will come with such functionalities from factory, and they can't (or shouldn't) be overruled or removed.
     * This function not only applies the time parametrization required for the trajectory to be executed with a smooth motion,
     * but also reduces the velocity scaling if the calculated cartesian speed at any segment of the trajectory exceeds the
     * cartesian limit set on the @p config parameter. Currently this function only limits the velocity scaling factor, not the
     * acceleration scaling factor: this allows for faster movements as the acceleration is not reduced together with the
     * velocity, but try to keep velocities and accelerations coherent with the cartesian speed you want to obtain. Having really
     * slow moves with high accelerations may cause jerky and instable moves, so when you set the @p config param always try to
     * keep the velocity and acceleration scaling factors coherent with the maximum cartesian speed you set.
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParameterization(
        const moveit_msgs::msg::RobotTrajectory &input_traj,
        const manymove_msgs::msg::MovementConfig &config);

    /**
     * @brief Execute a planned trajectory on the robot.
     * @param trajectory The trajectory to execute.
     * @return True if execution succeeded, false otherwise.
     *
     * @details This function doesn't use the internal functionalities of its implementation (MoveGroup or MoveItCPP):
     * instead, it uses a standard MoveIt functionality as the controller's trajectory execution action server, which
     * in this context is usually <robot_name>_traj_controller/follow_joint_trajectory. Older versions of this package
     * did use the specific functions of its implementation, for example the execute() and asyncExecute() funcions, but
     * this created differences in if and how parallel planning and execution would be handled dependin on which implementation
     * is used. Using the traj_controller separates the concerns of planning and execution and makes the implementation more
     * consistant.
     */
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override;

    /**
     * @brief Send a controlled stop command to the robot.
     * @param deceleration_time The duration (in seconds) over which the robot’s velocities should be ramped down to zero.
     * @return True if the stop command was sent and executed successfully, false otherwise.
     *
     * @details This function sends a single-point trajectory to the robot’s trajectory controller that holds the current
     * joint positions (with zero velocities) and gives the controller a deceleration window. The effect is a “spring-back”
     * stop where the robot decelerates smoothly. Increasing the deceleration_time leads to a smoother stop, but also increases
     * the movement required to decelerate.
     */
    bool sendControlledStop(double deceleration_time = 0.25);

    /**
     * @brief Retrieve the action client for FollowJointTrajectory.
     * @return A shared pointer to the action client used to send joint trajectory execution goals.
     */
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr getFollowJointTrajClient() const;

    /**
     * @brief Check whether a given set of joint positions is valid (i.e. collision free).
     * @param joint_positions A vector of joint positions.
     * @return True if the joint state is valid (no collisions), false otherwise.
     */
    bool isJointStateValid(const std::vector<double> &joint_positions) const;

    /**
     * @brief Checks if the start of the trajectory (the first waypoint)
     *        is within a specified tolerance of the given current joint state.
     * @param traj The planned trajectory.
     * @param current_joint_state A vector of doubles representing the current joint positions.
     * @param tolerance The maximum allowed difference (in radians) for each joint.
     * @return true if each joint position in the first waypoint is within tolerance, false otherwise.
     */
    bool isTrajectoryStartValid(const moveit_msgs::msg::RobotTrajectory &traj,
                                const std::vector<double> &current_joint_state,
                                double tolerance) const;

    bool isTrajectoryValid(
        const trajectory_msgs::msg::JointTrajectory &joint_traj_msg,
        const moveit_msgs::msg::Constraints &path_constraints,
        const std::string &group,
        bool verbose,
        std::vector<std::size_t> *invalid_index) const;

    bool isTrajectoryValid(
        const robot_trajectory::RobotTrajectory &trajectory,
        const moveit_msgs::msg::Constraints &path_constraints,
        const std::string &group,
        bool verbose,
        std::vector<std::size_t> *invalid_index) const;

    const std::string &getPlanningGroup() const;

private:
    /**
     * @brief Compute the total path length of a trajectory.
     * @param trajectory The input trajectory message.
     * @return The computed path length, combining joint and Cartesian distance.
     */
    double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const;

    /**
     * @brief Compute the maximum Cartesian speed within a trajectory.
     * @param trajectory A pointer to the robot trajectory to analyze.
     * @return The maximum speed in meters/second found along the trajectory.
     */
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    /**
     * @brief Compare two sets of joint targets for equality within a tolerance.
     * @param j1 The first joint target vector.
     * @param j2 The second joint target vector.
     * @param tolerance The maximum allowed difference for each joint.
     * @return True if all joints match within the tolerance, false otherwise.
     */
    bool areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Collision-check callback.
     * @param state Pointer to the RobotState being tested for collisions.
     * @param group Pointer to the JointModelGroup for which collision checks should be performed.
     * @return True if the state is free of collisions, false otherwise.
     *
     * @details This function is provided as a custom validity checker for
     * moveit::core::CartesianInterpolator::computeCartesianPath(), ensuring that
     * intermediate states do not collide with any known obstacles or the robot
     * itself. Without isStateValid() used as a callback function, computeCartesianPath()
     * wouldn't check for collisions just for mesh objects, while it did for primitive objects.
     * I couldn't figure if I missed some setting to avoid this problem, if I found some edge
     * cases that resulted in this abnormal behavior, or if it is in fact the intended behavior
     * of computeCartesianPath(). Regardless, this seems to solve the issue.
     * The function performs a collision check on the given @p state for the
     * specified joint model group. It uses a read only locked version of the current
     * Planning Scene to verify whether the @p state is in collision. If a collision is
     * detected, the function logs a warning and returns false. Otherwise, it returns true.
     */
    bool isStateValid(const moveit::core::RobotState *state, const moveit::core::JointModelGroup *group) const;

    rclcpp::Node::SharedPtr node_; ///< Shared pointer to the ROS2 node.
    rclcpp::Logger logger_;        ///< Logger for output messages.
    std::string planning_group_;   ///< The planning group name in MoveIt.
    std::string base_frame_;       ///< Base frame of the robot.
    std::string tcp_frame_;        ///< Tool center point (TCP) frame for the manipulator.
    std::string traj_controller_;  ///< Name of the trajectory controller to be used.

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_; ///< Shared pointer to MoveGroupInterface.
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;               ///< Planning scene monitor

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_traj_client_; ///< Action client for FollowJointTrajectory.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;                          ///< /joint_states subscriber.

    mutable std::mutex js_mutex_;
    std::map<std::string, double> current_positions_;
    std::map<std::string, double> current_velocities_;
};
