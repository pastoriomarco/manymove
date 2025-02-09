#pragma once

#include <vector>
#include <string>
#include <utility>
#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <manymove_msgs/msg/movement_config.hpp>
#include <manymove_msgs/action/plan_manipulator.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

/**
 * @class PlannerInterface
 * @brief Abstract interface for motion planners used in the manymove_planner package.
 *
 * This interface defines the essential methods that a planner implementation must provide:
 * - planning a trajectory for a given goal,
 * - executing a given trajectory,
 * - applying time parameterization (smoothing) while enforcing a maximum Cartesian speed,
 * - and sending a controlled stop command.
 */
class PlannerInterface
{
public:
    /**
     * @brief Virtual destructor for PlannerInterface.
     */
    virtual ~PlannerInterface() = default;

    /**
     * @brief Plan a trajectory to achieve a specified goal.
     * @param goal The target goal for the manipulator.
     * @return A pair containing a success flag (true if planning succeeded) and the planned robot trajectory.
     */
    virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_msgs::action::PlanManipulator::Goal &goal) = 0;

    /**
     * @brief Execute a given trajectory on the manipulator.
     * @param trajectory The trajectory to execute.
     * @return True if execution was successful, false otherwise.
     */
    virtual bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) = 0;

    /**
     * @brief Apply time parameterization to a trajectory.
     * @param input_traj The raw robot trajectory message (without time stamps) produced by the planner.
     * @param config The movement configuration that specifies velocity and acceleration scaling factors,
     * and the maximum allowed Cartesian speed.
     * @return A pair where the first element is true if time parameterization succeeded and the second element is the
     * resulting trajectory with computed time stamps.
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
    virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParameterization(
        const moveit_msgs::msg::RobotTrajectory &input_traj,
        const manymove_msgs::msg::MovementConfig &config) = 0;

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
    virtual bool sendControlledStop(double deceleration_time = 0.25) = 0;

    /**
     * @brief Retrieve the action client for FollowJointTrajectory.
     * @return A shared pointer to the action client used to send joint trajectory execution goals.
     */
    virtual rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr getFollowJointTrajClient() const = 0;

    /**
     * @brief Check whether a given set of joint positions is valid (i.e. collision free).
     * @param joint_positions A vector of joint positions.
     * @return True if the joint state is valid (no collisions), false otherwise.
     */
    virtual bool isJointStateValid(const std::vector<double> &joint_positions) const = 0;

protected:
    /**
     * @brief Protected constructor to prevent direct instantiation of the interface.
     */
    PlannerInterface() = default;
};
