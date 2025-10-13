// Copyright 2025 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

// MoveitCpp
#include <geometry_msgs/msg/point_stamped.h>

#include "manymove_planner/compat/moveit_compat.hpp"
#include "manymove_planner/compat/moveit_includes_compat.hpp"

// For linear movements
#include <tf2_eigen/tf2_eigen.hpp>
#if __has_include(<moveit/robot_state/robot_state.hpp>)
#include <moveit/robot_state/robot_state.hpp>
#else
#include <moveit/robot_state/robot_state.h>
#endif
#include "manymove_planner/compat/cartesian_interpolator_compat.hpp"

// For visualization
#if __has_include(<moveit_visual_tools/moveit_visual_tools.hpp>)
#include <moveit_visual_tools/moveit_visual_tools.hpp>
#else
#include <moveit_visual_tools/moveit_visual_tools.h>
#endif

namespace rvt = rviz_visual_tools;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");

  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {executor.spin();}).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  static const std::string PLANNING_GROUP = "lite6";
  static const std::string LOGNAME = "moveit_cpp_tutorial";
  static const std::vector<std::string> CONTROLLERS{1, "lite6_traj_controller"};

  /* Otherwise robot with zeros joint_states */
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  auto planning_scene_monitor = manymove_planner_compat::getPlanningSceneMonitorRw(moveit_cpp_ptr);
  if (planning_scene_monitor) {
    planning_scene_monitor->providePlanningSceneService();
  }

  auto planning_components =
    std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a
  // script
  moveit_visual_tools::MoveItVisualTools visual_tools(
    node, "link_base", "moveit_cpp_tutorial", planning_scene_monitor);
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Define possible joint moves:
  std::vector<double> rest_joint_values = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
  std::vector<double> scan_sx_joint_values = {-0.175, -0.419, 1.378, 0.349, 1.535, -0.977};
  std::vector<double> scan_dx_joint_values = {0.733, -0.297, 1.378, -0.576, 1.692, 1.291};

  // Planning with MoveItCpp
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // There are multiple ways to set the start and the goal states of the plan
  // they are illustrated in the following plan examples
  //
  // Plan #1
  // ^^^^^^^
  //
  // We can set the start state of the plan to the current state of the robot
  planning_components->setStartStateToCurrentState();

  // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message
  // type as follow
  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.header.frame_id = "link_base";
  target_pose1.pose.orientation.x = 1.0;
  target_pose1.pose.orientation.y = 0.0;
  target_pose1.pose.orientation.z = 0.0;
  target_pose1.pose.orientation.w = 0.0;
  target_pose1.pose.position.x = 0.20;
  target_pose1.pose.position.y = -0.15;
  target_pose1.pose.position.z = 0.15;
  planning_components->setGoal(target_pose1, "link_tcp");

  // Now, we call the PlanningComponents to compute the plan and visualize it.
  // Note that we are just planning
  auto plan_solution1 = planning_components->plan();

  // Check if PlanningComponents succeeded in finding the plan
  if (plan_solution1) {
    // Visualize the start pose in Rviz
    visual_tools.publishAxisLabeled(
      robot_start_state->getGlobalLinkTransform("link_tcp"), "start_pose");
    // Visualize the goal pose in Rviz
    visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
    visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
    // Visualize the trajectory in Rviz
    visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    manymove_planner_compat::executePlanningComponent(*planning_components);  // Execute
                                                                              // the plan
  }

  // Plan #1 visualization:
  //
  // .. image:: images/moveitcpp_plan1.png
  //    :width: 250pt
  //    :align: center
  //
  // Start the next plan
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // --------------------------------------------------
  // Plan #2: Move to Joint Target Position (rest_joint_values)
  // --------------------------------------------------

  planning_components->setStartStateToCurrentState();

  // Define the joint target
  // std::vector<double> rest_joint_values = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};

  // Create a RobotState object and set the joint values
  moveit::core::RobotState goal_joint_state(robot_model_ptr);
  goal_joint_state.setJointGroupPositions(joint_model_group_ptr, rest_joint_values);

  // Set the joint target as the goal
  planning_components->setGoal(goal_joint_state);

  // Plan to the joint target
  auto plan_solution6 = planning_components->plan();
  if (plan_solution6) {
    // Visualize the start and goal joint states
    moveit::core::RobotState current_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution6.start_state, current_state);

    visual_tools.publishAxisLabeled(current_state.getGlobalLinkTransform("link_tcp"), "start_pose");
    // Publish the robot state with a specified color
    // visual_tools.publishRobotState(goal_joint_state, rvt::GREEN);
    // Publish a text label near the robot state
    visual_tools.publishText(text_pose, "Goal Joint State", rvt::WHITE, rvt::XLARGE);

    // Visualize the trajectory
    visual_tools.publishTrajectoryLine(plan_solution6.trajectory, joint_model_group_ptr);
    visual_tools.trigger();

    // Execute the plan
    manymove_planner_compat::executePlanningComponent(*planning_components);  // Execute
                                                                              // the plan
  }

  // Visualization
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Plan #3
  // ^^^^^^^
  //
  // Here we will set the current state of the plan using
  // moveit::core::RobotState

  planning_components->setStartStateToCurrentState();

  auto start_state = *(moveit_cpp_ptr->getCurrentState());
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = 1.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.0;
  start_pose.position.x = 0.25;
  start_pose.position.y = 0.0;
  start_pose.position.z = 0.25;

  start_state.setFromIK(joint_model_group_ptr, start_pose);

  // TO PLAN FROM A DETERMINED START STATE:
  planning_components->setStartState(start_state);

  // We will reuse the old goal that we had and plan to it. << NOPE! I activated execution, so
  // change of plans!
  target_pose1.pose.position.z = 0.15;
  planning_components->setGoal(target_pose1, "link_tcp");

  auto plan_solution2 = planning_components->plan();
  if (plan_solution2) {
    moveit::core::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution2.start_state, robot_state);

    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("link_tcp"), "start_pose");
    visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
    visual_tools.publishText(
      text_pose, "moveit::core::RobotState_Start_State", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan_solution2.trajectory, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    // manymove_planner_compat::executePlanningComponent(*planning_components); // Execute the plan
  }

  // Plan #3 visualization:
  //
  // .. image:: images/moveitcpp_plan2.png
  //    :width: 250pt
  //    :align: center
  //
  // Start the next plan
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Plan #4
  // ^^^^^^^
  //
  // We can also set the goal of the plan using
  // moveit::core::RobotState

  planning_components->setStartStateToCurrentState();

  auto target_state = *robot_start_state;
  geometry_msgs::msg::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 0.0;
  target_pose2.orientation.z = 0.0;
  target_pose2.orientation.w = 0.0;
  target_pose2.position.x = 0.25;
  target_pose2.position.y = -0.05;
  target_pose2.position.z = 0.3;

  target_state.setFromIK(joint_model_group_ptr, target_pose2);

  planning_components->setGoal(target_state);

  auto plan_solution3 = planning_components->plan();
  if (plan_solution3) {
    moveit::core::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);

    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("link_tcp"), "start_pose");

    size_t num_waypoints = plan_solution3.trajectory->getWayPointCount();
    if (num_waypoints == 0) {
      RCLCPP_WARN(LOGGER, "Planned trajectory has no waypoints.");
    } else {
      const moveit::core::RobotState & last_waypoint =
        plan_solution3.trajectory->getWayPoint(num_waypoints - 1);
      Eigen::Isometry3d ee_transform = last_waypoint.getGlobalLinkTransform("link_tcp");

      // Convert Eigen::Isometry3d to geometry_msgs::Pose
      geometry_msgs::msg::Pose actual_goal_pose = tf2::toMsg(ee_transform);

      visual_tools.publishAxisLabeled(actual_goal_pose, "target_pose");
      visual_tools.publishText(text_pose, "Actual Goal Pose", rvt::WHITE, rvt::XLARGE);
    }

    visual_tools.publishTrajectoryLine(plan_solution3.trajectory, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    manymove_planner_compat::executePlanningComponent(*planning_components);  // Execute
                                                                              // the plan
  }

  // Plan #4 visualization:
  //
  // .. image:: images/moveitcpp_plan3.png
  //    :width: 250pt
  //    :align: center
  //
  // Start the next plan
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Plan #5
  // ^^^^^^^
  //
  // We can set the start state of the plan to the current state of the robot
  // We can set the goal of the plan using the name of a group states
  // for panda robot we have one named robot state for "panda_arm" planning group called "ready"
  // see `panda_arm.xacro
  // <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/panda_arm.xacro#L13>`_

  /* // Set the start state of the plan from a named robot state */
  /* planning_components->setStartState("ready"); // Not implemented yet */
  // Set the goal state of the plan from a named robot state

  planning_components->setStartStateToCurrentState();

  // setting the goal as the rest joint target to give a move to do for the next collision avoidance
  // planning_components->setGoal("home");
  planning_components->setGoal(
    target_pose1,
    "link_tcp");  // WARNING: if modified it will have
                  // to
                  // be set anew

  // Again we will reuse the old start that we had and plan from it.
  auto plan_solution4 = planning_components->plan();
  if (plan_solution4) {
    moveit::core::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution4.start_state, robot_state);

    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("link_tcp"), "start_pose");
    visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
    visual_tools.publishText(text_pose, "Goal_Pose_From_Named_State", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan_solution4.trajectory, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    manymove_planner_compat::executePlanningComponent(*planning_components);  // Execute
                                                                              // the plan
  }

  // Plan #5 visualization:
  //
  // .. image:: images/moveitcpp_plan4.png
  //    :width: 250pt
  //    :align: center
  //
  // Start the next plan
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Plan #6
  // ^^^^^^^
  //
  // We can also generate motion plans around objects in the collision scene.
  //
  // First we create the collision object

  planning_components->setStartStateToCurrentState();

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "link_base";
  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {0.1, 0.1, 0.1};

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.3;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.3;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add object to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
    scene->processCollisionObjectMsg(collision_object);
  }  // Unlock PlanningScene
  planning_components->setStartStateToCurrentState();
  planning_components->setGoal("home");

  auto plan_solution5 = planning_components->plan();
  if (plan_solution5) {
    visual_tools.publishText(
      text_pose, "Planning_Around_Collision_Object", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan_solution5.trajectory, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    manymove_planner_compat::executePlanningComponent(*planning_components);  // Execute
                                                                              // the plan
  }

  // Plan #6 visualization:
  //
  // .. image:: images/moveitcpp_plan5.png
  //    :width: 250pt
  //    :align: center
  //
  // END_TUTORIAL
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Define the waypoints as EigenSTL::vector_Isometry3d
  EigenSTL::vector_Isometry3d waypoints;

  Eigen::Isometry3d wp_start_pose =
    planning_components->getStartState()->getGlobalLinkTransform("link_tcp");
  waypoints.push_back(wp_start_pose);

  Eigen::Isometry3d waypoint1 = wp_start_pose;
  waypoint1.translate(
    Eigen::Vector3d(
      0.1, 0.0,
      0.0));                                  // Move 10 cm upward
  waypoints.push_back(waypoint1);

  Eigen::Isometry3d waypoint2 = waypoint1;
  waypoint2.translate(
    Eigen::Vector3d(
      0.0, 0.1,
      0.0));                                  // Move 10 cm sideways
  waypoints.push_back(waypoint2);

  // Retrieve joint_model_group and link_model
  auto link_model = joint_model_group_ptr->getLinkModel("link_tcp");

  // Prepare a vector of RobotStatePtr for the resulting trajectory states
  std::vector<moveit::core::RobotStatePtr> trajectory_states;
  moveit::core::RobotStatePtr wp_start_state = planning_components->getStartState();

  // Compute Cartesian path using the correct signature
  manymove_msgs::msg::MovementConfig demo_cfg;
  demo_cfg.jump_threshold = 0.0;
  demo_cfg.cartesian_precision_translational = 0.001;
  demo_cfg.cartesian_precision_rotational = 0.05;
  demo_cfg.cartesian_precision_max_resolution = 0.005;

  double fraction = manymove_planner_compat::computeCartesianPathCompat(
    wp_start_state.get(),
    // Pass the raw pointer
    joint_model_group_ptr,
    // Joint model group
    trajectory_states,
    // Resulting trajectory
    // states
    link_model,
    // Link to control
    waypoints,
    // Waypoints for Cartesian
    // motion
    true,
    // Global reference frame
    moveit::core::MaxEEFStep(0.01),
    // Maximum end-effector step
    // size
    demo_cfg,
    // Config for precision /
    // jump
    moveit::core::GroupStateValidityCallbackFn(),
    // No validity callback
    kinematics::KinematicsQueryOptions(),
    // Default kinematics
    // options
    nullptr  // No IK cost
             // function
  );

  // Check if the path was successfully computed
  if (fraction > 0.99) {
    RCLCPP_INFO(LOGGER, "Successfully computed Cartesian path. Executing...");

    // Build a RobotTrajectory from the computed states
    robot_trajectory::RobotTrajectory trajectory(moveit_cpp_ptr->getRobotModel(), PLANNING_GROUP);
    double dt = 0.1;  // time interval between waypoints
    for (const auto & rs : trajectory_states) {
      trajectory.addSuffixWayPoint(*rs, dt);
    }

    // Visualize the trajectory
    visual_tools.publishTrajectoryLine(
      std::make_shared<robot_trajectory::RobotTrajectory>(trajectory), joint_model_group_ptr);
    visual_tools.trigger();

    // Execute the trajectory
    auto trajectory_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory);
    manymove_planner_compat::executeTrajectory(
      *moveit_cpp_ptr, PLANNING_GROUP, trajectory_ptr, CONTROLLERS);
  } else {
    RCLCPP_WARN(LOGGER, "Could not compute full Cartesian path.");
  }

  visual_tools.prompt("Press 'next' to end the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
