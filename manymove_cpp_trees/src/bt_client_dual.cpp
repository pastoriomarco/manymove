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

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/decorators/force_failure_node.h>

#include "manymove_cpp_trees/action_nodes_objects.hpp"
#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include "manymove_cpp_trees/action_nodes_signals.hpp"
#include "manymove_cpp_trees/action_nodes_logic.hpp"
#include "manymove_cpp_trees/robot.hpp"
#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/object.hpp"
#include "manymove_cpp_trees/tree_helper.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"
#include "manymove_cpp_trees/hmi_service_node.hpp"

#include "manymove_msgs/action/set_output.hpp"
#include "manymove_msgs/action/get_input.hpp"
#include "manymove_msgs/action/check_robot_state.hpp"
#include "manymove_msgs/action/reset_robot_state.hpp"

#include <string>
#include <vector>

using geometry_msgs::msg::Pose;
using namespace manymove_cpp_trees;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("bt_client_node");
  RCLCPP_INFO(node->get_logger(), "BT Client Node started (Purely Programmatic XML).");

  // ----------------------------------------------------------------------------
  // 1) Create blackboard, keys and nodes
  // ----------------------------------------------------------------------------

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

  std::vector<manymove_cpp_trees::BlackboardEntry> keys;

  // Define all params and blackboard keys for the robot:
  RobotParams rp_1 = defineRobotParams(node, blackboard, keys, "_1");
  RobotParams rp_2 = defineRobotParams(node, blackboard, keys, "_2");

  // ----------------------------------------------------------------------------
  // 2) Setup moves
  // ----------------------------------------------------------------------------

  auto move_configs = defineMovementConfigs();

  // We define the joint targets we need for the joint moves as vectors of doubles.
  // Be careful that the number of values must match the number of DOF of the robot (here, 6 DOF)
  std::vector<double> joint_rest_1 = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};

  std::vector<double> joint_rest_2 = {0.0, 0.785, -0.785, 0.0, -1.57, 0.0};

  std::string named_home_1 = "home";
  std::string named_home_2 = "home";

  // Test poses to place the object, these are not overwritten later (for now)
  Pose drop_target_1 = createPose(0.2, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0);
  Pose approach_drop_target_1 = drop_target_1;
  approach_drop_target_1.position.z += 0.02;

  Pose drop_target_2 = createPose(0.3, 1.05, 0.2, 1.0, 0.0, 0.0, 0.0);
  Pose approach_drop_target_2 = drop_target_2;
  approach_drop_target_2.position.z += 0.02;

  // Populate the blackboard with the poses, one unique key for each pose we want to use.
  // Be careful not to use names that may conflict with the keys automatically created for the
  // moves. (Usually move_{move_id})
  blackboard->set("pick_target_1_key", Pose());
  blackboard->set("approach_pick_target_1_key", Pose());

  blackboard->set("drop_target_1_key", drop_target_1);
  blackboard->set("approach_drop_target_1_key", approach_drop_target_1);

  blackboard->set("pick_target_2_key", Pose());
  blackboard->set("approach_pick_target_2_key", Pose());

  blackboard->set("drop_target_2_key", drop_target_2);
  blackboard->set("approach_drop_target_2_key", approach_drop_target_2);

  // Compose the sequences of moves. Each of the following sequences represent a logic
  std::string tcp_frame_name_1 = rp_1.prefix + rp_1.tcp_frame;
  std::string tcp_frame_name_2 = rp_2.prefix + rp_2.tcp_frame;

  std::vector<Move> rest_position_1 = {
    {rp_1.prefix, tcp_frame_name_1, "joint", move_configs["max_move"], "", joint_rest_1},
  };
  std::vector<Move> rest_position_2 = {
    {rp_2.prefix, tcp_frame_name_2, "joint", move_configs["max_move"], "", joint_rest_2},
  };

  // Sequences for Pick/Drop/Homing
  std::vector<Move> pick_sequence_1 = {
    {rp_1.prefix, tcp_frame_name_1, "pose", move_configs["mid_move"], "approach_pick_target_1_key"},
    {rp_1.prefix, tcp_frame_name_1, "cartesian", move_configs["slow_move"], "pick_target_1_key"},
  };

  std::vector<Move> pick_sequence_2 = {
    {rp_2.prefix, tcp_frame_name_2, "pose", move_configs["mid_move"], "approach_pick_target_2_key"},
    {rp_2.prefix, tcp_frame_name_2, "cartesian", move_configs["slow_move"], "pick_target_2_key"},
  };

  std::vector<Move> drop_sequence_1 = {
    {rp_1.prefix, tcp_frame_name_1, "pose", move_configs["mid_move"], "approach_pick_target_1_key"},
    {rp_1.prefix, tcp_frame_name_1, "pose", move_configs["max_move"], "approach_drop_target_1_key"},
    {rp_1.prefix, tcp_frame_name_1, "cartesian", move_configs["slow_move"], "drop_target_1_key"},
  };

  std::vector<Move> drop_sequence_2 = {
    {rp_2.prefix, tcp_frame_name_2, "pose", move_configs["mid_move"], "approach_pick_target_2_key"},
    {rp_2.prefix, tcp_frame_name_2, "pose", move_configs["max_move"], "approach_drop_target_2_key"},
    {rp_2.prefix, tcp_frame_name_2, "cartesian", move_configs["slow_move"], "drop_target_2_key"},
  };

  std::vector<Move> home_position_1 = {
    {rp_1.prefix, tcp_frame_name_1, "cartesian", move_configs["max_move"],
     "approach_drop_target_1_key"},
    {rp_1.prefix, tcp_frame_name_1, "named", move_configs["max_move"], "", {}, named_home_1},
    {rp_1.prefix, tcp_frame_name_1, "joint", move_configs["max_move"], "", joint_rest_1},
  };

  std::vector<Move> home_position_2 = {
    {rp_2.prefix, tcp_frame_name_2, "cartesian", move_configs["max_move"],
     "approach_drop_target_2_key"},
    {rp_2.prefix, tcp_frame_name_2, "named", move_configs["max_move"], "", {}, named_home_2},
    {rp_2.prefix, tcp_frame_name_2, "joint", move_configs["max_move"], "", joint_rest_2},
  };

  // build the xml snippets for the single moves of robot 1
  // or translate them directly if they are only used once
  std::string to_rest_1_xml = buildMoveXML(rp_1.prefix,
                                           rp_1.prefix + "toRest",
                                           rest_position_1,
                                           blackboard);

  std::string pick_object_1_xml = buildMoveXML(rp_1.prefix,
                                               rp_1.prefix + "pick",
                                               pick_sequence_1,
                                               blackboard);

  std::string drop_object_1_xml = buildMoveXML(rp_1.prefix,
                                               rp_1.prefix + "drop",
                                               drop_sequence_1,
                                               blackboard);

  std::string to_home_1_xml = buildMoveXML(rp_1.prefix,
                                           rp_1.prefix + "home",
                                           home_position_1,
                                           blackboard);

  // Translate it to xml tree leaf or branch
  std::string prep_sequence_1_xml = sequenceWrapperXML(rp_1.prefix + "ComposedPrepSequence",
                                                     {to_rest_1_xml});
  std::string pick_sequence_1_xml = sequenceWrapperXML(rp_1.prefix + "ComposedPickSequence",
                                                     {pick_object_1_xml});
  std::string drop_sequence_1_xml = sequenceWrapperXML(rp_1.prefix + "ComposedDropSequence",
                                                     {drop_object_1_xml});
  std::string home_sequence_1_xml = sequenceWrapperXML(rp_1.prefix + "ComposedHomeSequence",
                                                     {to_home_1_xml, to_rest_1_xml});

  // build the xml snippets for the single moves of robot 1
  std::string to_rest_2_xml = buildMoveXML(rp_2.prefix,
                                           rp_2.prefix + "toRest",
                                           rest_position_2,
                                           blackboard);

  std::string pick_object_2_xml = buildMoveXML(rp_2.prefix,
                                               rp_2.prefix + "pick",
                                               pick_sequence_2,
                                               blackboard);

  std::string drop_object_2_xml = buildMoveXML(rp_2.prefix,
                                               rp_2.prefix + "drop",
                                               drop_sequence_2,
                                               blackboard);

  std::string to_home_2_xml = buildMoveXML(rp_2.prefix,
                                           rp_2.prefix + "home",
                                           home_position_2,
                                           blackboard);

  // Translate it to xml tree leaf or branch
  std::string prep_sequence_2_xml = sequenceWrapperXML(rp_2.prefix + "ComposedPrepSequence",
                                                     {to_rest_2_xml});
  std::string pick_sequence_2_xml = sequenceWrapperXML(rp_2.prefix + "ComposedPickSequence",
                                                     {pick_object_2_xml});
  std::string drop_sequence_2_xml = sequenceWrapperXML(rp_2.prefix + "ComposedDropSequence",
                                                     {drop_object_2_xml});
  std::string home_sequence_2_xml = sequenceWrapperXML(rp_2.prefix + "ComposedHomeSequence",
                                                     {to_home_2_xml, to_rest_2_xml});

  // ----------------------------------------------------------------------------
  // 3) Build blocks for objects handling
  // ----------------------------------------------------------------------------

  blackboard->set("ground_id_key", "obstacle_ground");
  blackboard->set("ground_shape_key", "box");
  blackboard->set("ground_dimension_key", std::vector<double>{0.8, 2.0, 0.1});
  blackboard->set("ground_pose_key", createPoseRPY(0.0, 0.5, -0.051, 0.0, 0.0, 0.0));
  blackboard->set("ground_scale_key", std::vector<double>{1.0, 1.0, 1.0});

  blackboard->set("wall_id_key", "obstacle_wall");
  blackboard->set("wall_shape_key", "box");
  blackboard->set("wall_dimension_key", std::vector<double>{0.8, 0.02, 0.8});
  blackboard->set("wall_pose_key", createPoseRPY(0.0, 0.5, 0.4, 0.0, 0.0, 0.0));
  blackboard->set("wall_scale_key", std::vector<double>{1.0, 1.0, 1.0});

  blackboard->set("cylinder_id_key", "graspable_cylinder");
  blackboard->set("cylinder_shape_key", "cylinder");
  blackboard->set("cylinder_dimension_key", std::vector<double>{0.1, 0.005});
  blackboard->set("cylinder_pose_key", createPoseRPY(0.2, 0.7, 0.105, 0.0, 1.57, 0.0));
  blackboard->set("cylinder_scale_key", std::vector<double>{1.0, 1.0, 1.0});

  blackboard->set("mesh_id_key", "graspable_mesh");
  blackboard->set("mesh_shape_key", "mesh");
  blackboard->set("mesh_file_key", "package://manymove_object_manager/meshes/unit_tube.stl");
  blackboard->set("mesh_scale_key", std::vector<double>{0.01, 0.01, 0.1});                //< The
                                                                                          // tube is
                                                                                          // vertical
                                                                                          // with
                                                                                          // dimension
                                                                                          // 1m x 1m
                                                                                          // x 1m.
                                                                                          // We
                                                                                          // scale
                                                                                          // it to
                                                                                          // 10x10x100
                                                                                          // mm
  blackboard->set("mesh_pose_key", createPoseRPY(0.1, -0.2, 0.2005, 0.785, 1.57, 0.0));   //< We
                                                                                          // place
                                                                                          // it on
                                                                                          // the
                                                                                          // floor
                                                                                          // and lay
                                                                                          // it on
                                                                                          // its
                                                                                          // side,
                                                                                          // X+
                                                                                          // facing
                                                                                          // down

  // Create object actions xml snippets (the object are created directly in the create*() functions
  // relative to each type of object action)
  std::string check_ground_obj_xml =
    buildObjectActionXML("check_ground", createCheckObjectExists("ground_id_key"));
  std::string check_wall_obj_xml =
    buildObjectActionXML("check_wall", createCheckObjectExists("wall_id_key"));
  std::string check_cylinder_obj_xml = buildObjectActionXML("check_cylinder",
                                                            createCheckObjectExists(
                                                              "cylinder_id_key"));
  std::string check_mesh_obj_xml =
    buildObjectActionXML("check_mesh", createCheckObjectExists("mesh_id_key"));

  std::string add_ground_obj_xml =
    buildObjectActionXML("add_ground",
                         createAddObject("ground_id_key", "ground_shape_key",
                                         "ground_dimension_key", "ground_pose_key",
                                         "ground_scale_key", ""));
  std::string add_wall_obj_xml =
    buildObjectActionXML("add_wall",
                         createAddObject("wall_id_key", "wall_shape_key", "wall_dimension_key",
                                         "wall_pose_key",
                                         "wall_scale_key", ""));
  std::string add_cylinder_obj_xml =
    buildObjectActionXML("add_cylinder",
                         createAddObject("cylinder_id_key", "cylinder_shape_key",
                                         "cylinder_dimension_key",
                                         "cylinder_pose_key", "cylinder_scale_key", ""));
  std::string add_mesh_obj_xml =
    buildObjectActionXML("add_mesh",
                         createAddObject("mesh_id_key", "mesh_shape_key", "", "mesh_pose_key",
                                         "mesh_scale_key",
                                         "mesh_file_key"));

  // Compose the check and add sequence for objects
  std::string init_ground_obj_xml = fallbackWrapperXML("init_ground_obj", {check_ground_obj_xml,
                                                                           add_ground_obj_xml});
  std::string init_wall_obj_xml = fallbackWrapperXML("init_wall_obj", {check_wall_obj_xml,
                                                                       add_wall_obj_xml});
  std::string init_cylinder_obj_xml = fallbackWrapperXML("init_cylinder_obj",
                                                       {check_cylinder_obj_xml,
                                                        add_cylinder_obj_xml});
  std::string init_mesh_obj_xml = fallbackWrapperXML("init_mesh_obj", {check_mesh_obj_xml,
                                                                       add_mesh_obj_xml});

  // the name of the link to attach the object to, and the object to manipulate
  blackboard->set("tcp_frame_name_1_key", tcp_frame_name_1);
  blackboard->set("object_to_manipulate_1_key", "graspable_mesh");
  blackboard->set("tcp_frame_name_2_key", tcp_frame_name_2);
  blackboard->set("object_to_manipulate_2_key", "graspable_cylinder");

  // The gripper is pneumatic so the jaws are not dynamically updated, we don't want touch links:
  std::vector<std::string> touch_links_empty = {};
  blackboard->set("touch_links_empty_key", touch_links_empty);

  std::string attach_obj_1_xml = buildObjectActionXML("attach_obj_to_manipulate_1",
                                                      createAttachObject(
                                                        "object_to_manipulate_1_key",
                                                        "tcp_frame_name_1_key",
                                                        "touch_links_empty_key"));
  std::string detach_obj_1_xml = fallbackWrapperXML("detach_obj_to_manipulate_1_always_success",
                                                  {buildObjectActionXML("detach_obj_to_manipulate_1",
                                                                        createDetachObject(
                                                                          "object_to_manipulate_1_key",
                                                                          "tcp_frame_name_1_key")),
                                                   "<AlwaysSuccess />"});
  std::string remove_obj_1_xml = fallbackWrapperXML("remove_obj_to_manipulate_1_always_success",
                                                  {buildObjectActionXML("remove_obj_to_manipulate_1",
                                                                        createRemoveObject(
                                                                          "object_to_manipulate_1_key")),
                                                   "<AlwaysSuccess />"});

  std::string attach_obj_2_xml = buildObjectActionXML("attach_obj_to_manipulate_2",
                                                      createAttachObject(
                                                        "object_to_manipulate_2_key",
                                                        "tcp_frame_name_2_key",
                                                        "touch_links_empty_key"));
  std::string detach_obj_2_xml = fallbackWrapperXML("detach_obj_to_manipulate_2_always_success",
                                                  {buildObjectActionXML("detach_obj_to_manipulate_2",
                                                                        createDetachObject(
                                                                          "object_to_manipulate_2_key",
                                                                          "tcp_frame_name_2_key")),
                                                   "<AlwaysSuccess />"});
  std::string remove_obj_2_xml = fallbackWrapperXML("remove_obj_to_manipulate_2_always_success",
                                                  {buildObjectActionXML("remove_obj_to_manipulate_2",
                                                                        createRemoveObject(
                                                                          "object_to_manipulate_2_key")),
                                                   "<AlwaysSuccess />"});
  // ----------------------------------------------------------------------------
  // 4) Add GetObjectPoseAction Node and nodes to attach/detach objects
  // ----------------------------------------------------------------------------

  // Utility world frame key
  blackboard->set("world_frame_key", "world");

  // Define the transformation and reference orientation
  blackboard->set("pick_pre_transform_xyz_rpy_1_key", std::vector<double>{-0.002, 0.0, 0.0, 0.0,
                                                                          1.57, 0.0});
  blackboard->set("approach_pick_pre_transform_xyz_rpy_1_key", std::vector<double>{-0.05, 0.0, 0.0,
                                                                                   0.0, 1.57, 0.0});
  blackboard->set("pick_post_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, -0.025, 3.14,
                                                                           0.0, 0.0});

  // Translate get_pose_action to xml tree leaf
  std::string get_pick_pose_1_xml =
    buildObjectActionXML("get_pick_pose_1",
                         createGetObjectPose("object_to_manipulate_1_key",
                                             "pick_target_1_key",
                                             "world_frame_key",
                                             "pick_pre_transform_xyz_rpy_1_key",
                                             "pick_post_transform_xyz_rpy_1_key"));
  std::string get_approach_pose_1_xml = buildObjectActionXML("get_approach_pose_1",
                                                             createGetObjectPose(
                                                               "object_to_manipulate_1_key",
                                                               "approach_pick_target_1_key",
                                                               "world_frame_key",
                                                               "approach_pick_pre_transform_xyz_rpy_1_key",
                                                               "pick_post_transform_xyz_rpy_1_key"));

  // Define the object ID and pose_key where the pose will be stored for robot 2

  // Define the transformation and reference orientation
  blackboard->set("pick_pre_transform_xyz_rpy_2_key", std::vector<double>{-0.002, 0.0, 0.0, 0.0,
                                                                          1.57, 0.0});
  blackboard->set("approach_pick_pre_transform_xyz_rpy_2_key", std::vector<double>{-0.05, 0.0, 0.0,
                                                                                   0.0, 1.57, 0.0});
  blackboard->set("pick_post_transform_xyz_rpy_2_key", std::vector<double>{0.0, 0.0, -0.025, 3.14,
                                                                           0.0, 0.0});

  // Translate get_pose_action to xml tree leaf
  std::string get_pick_pose_2_xml =
    buildObjectActionXML("get_pick_pose_2",
                         createGetObjectPose("object_to_manipulate_2_key",
                                             "pick_target_2_key",
                                             "world_frame_key",
                                             "pick_pre_transform_xyz_rpy_2_key",
                                             "pick_post_transform_xyz_rpy_2_key"));
  std::string get_approach_pose_2_xml = buildObjectActionXML("get_approach_pose_2",
                                                             createGetObjectPose(
                                                               "object_to_manipulate_2_key",
                                                               "approach_pick_target_2_key",
                                                               "world_frame_key",
                                                               "approach_pick_pre_transform_xyz_rpy_2_key",
                                                               "pick_post_transform_xyz_rpy_2_key"));

  // ----------------------------------------------------------------------------
  // 5) Define Signals calls:
  // ----------------------------------------------------------------------------

  // Let's send and receive signals only if the robot is real, and let's fake a delay on inputs
  // otherwise
  // Robot 1
  std::string signal_gripper_close_1_xml =
    (rp_1.is_real ? buildSetOutputXML(rp_1.prefix, "GripperClose", "controller", 0, 1) : "");
  std::string signal_gripper_open_1_xml =
    (rp_1.is_real ? buildSetOutputXML(rp_1.prefix, "GripperOpen", "controller", 0, 0) : "");
  std::string check_gripper_close_1_xml =
    (rp_1.is_real ? buildWaitForInput(rp_1.prefix, "WaitForSensor", "controller", 0,
                                      1) :
      "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");
  std::string check_gripper_open_1_xml =
    (rp_1.is_real ? buildWaitForInput(rp_1.prefix, "WaitForSensor", "controller", 0,
                                      0) :
      "<Delay delay_msec=\"250\">\n  <AlwaysSuccess />\n</Delay>\n");
  std::string check_robot_state_1_xml = buildCheckRobotStateXML(rp_1.prefix, "CheckRobot",
                                                                "robot_ready", "error_code",
                                                                "robot_mode", "robot_state",
                                                                "robot_msg");
  std::string reset_robot_state_1_xml = buildResetRobotStateXML(rp_1.prefix, "ResetRobot",
                                                                rp_1.model);

  std::string check_reset_robot_1_xml =
    (rp_1.is_real ? fallbackWrapperXML(rp_1.prefix + "CheckResetFallback",
                                     {check_robot_state_1_xml,
                                      reset_robot_state_1_xml}) :
      "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

  // Robot 2
  std::string signal_gripper_close_2_xml =
    (rp_2.is_real ? buildSetOutputXML(rp_2.prefix, "GripperClose", "controller", 0, 1) : "");
  std::string signal_gripper_open_2_xml =
    (rp_2.is_real ? buildSetOutputXML(rp_2.prefix, "GripperOpen", "controller", 0, 0) : "");
  std::string check_gripper_close_2_xml =
    (rp_2.is_real ? buildWaitForInput(rp_2.prefix, "WaitForSensor", "controller", 0,
                                      1) :
      "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");
  std::string check_gripper_open_2_xml =
    (rp_2.is_real ? buildWaitForInput(rp_2.prefix, "WaitForSensor", "controller", 0,
                                      0) :
      "<Delay delay_msec=\"250\">\n  <AlwaysSuccess />\n</Delay>\n");
  std::string check_robot_state_2_xml = buildCheckRobotStateXML(rp_2.prefix, "CheckRobot",
                                                                "robot_ready", "error_code",
                                                                "robot_mode", "robot_state",
                                                                "robot_msg");
  std::string reset_robot_state_2_xml = buildResetRobotStateXML(rp_2.prefix, "ResetRobot",
                                                                rp_2.model);

  std::string check_reset_robot_2_xml =
    (rp_2.is_real ? fallbackWrapperXML(rp_2.prefix + "CheckResetFallback",
                                     {check_robot_state_2_xml,
                                      reset_robot_state_2_xml}) :
      "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

  // ----------------------------------------------------------------------------
  // 6) Combine the objects and moves in a sequences that can run a number of times:
  // ----------------------------------------------------------------------------

  // Let's build the full sequence in logically separated blocks:
  // General
  std::string spawn_fixed_objects_xml = sequenceWrapperXML("SpawnFixedObjects",
                                                         {init_ground_obj_xml,
                                                          init_wall_obj_xml});

  // Robot 1
  std::string get_grasp_object_poses_1_xml = sequenceWrapperXML("GetGraspPoses",
                                                              {get_pick_pose_1_xml,
                                                               get_approach_pose_1_xml});
  std::string go_to_pick_pose_1_xml = sequenceWrapperXML("GoToPickPose", {pick_sequence_1_xml});
  std::string close_gripper_1_xml = sequenceWrapperXML("CloseGripper", {signal_gripper_close_1_xml,
                                                                        check_gripper_close_1_xml,
                                                                        attach_obj_1_xml});
  std::string open_gripper_1_xml = sequenceWrapperXML("OpenGripper", {signal_gripper_open_1_xml,
                                                                      detach_obj_1_xml});

  // Robot 2
  std::string get_grasp_object_poses_2_xml = sequenceWrapperXML("GetGraspPoses",
                                                              {get_pick_pose_2_xml,
                                                               get_approach_pose_2_xml});
  std::string go_to_pick_pose_2_xml = sequenceWrapperXML("GoToPickPose", {pick_sequence_2_xml});
  std::string close_gripper_2_xml = sequenceWrapperXML("CloseGripper", {signal_gripper_close_2_xml,
                                                                        check_gripper_close_2_xml,
                                                                        attach_obj_2_xml});
  std::string open_gripper_2_xml = sequenceWrapperXML("OpenGripper", {signal_gripper_open_2_xml,
                                                                      detach_obj_2_xml});

  // Dedicated spawnable objects per robot:
  std::string spawn_graspable_objects_1_xml = sequenceWrapperXML("SpawnGraspableObjects",
                                                                 {init_mesh_obj_xml});
  std::string spawn_graspable_objects_2_xml = sequenceWrapperXML("SpawnGraspableObjects",
                                                                 {init_cylinder_obj_xml});

  // Parallel startup subsequences:
  std::string startup_sequence_1_xml = sequenceWrapperXML("StartUpSequence_1",
                                                        {check_reset_robot_1_xml,
                                                         prep_sequence_1_xml});
  std::string startup_sequence_2_xml = sequenceWrapperXML("StartUpSequence_2",
                                                        {check_reset_robot_2_xml,
                                                         prep_sequence_2_xml});
  std::string parallel_sub_startup_sequences_xml = parallelWrapperXML("Parallel_startupSequences",
                                                                    {startup_sequence_1_xml,
                                                                     startup_sequence_2_xml}, 2,
                                                                      1);

  // Set up a sequence to reset the scene:
  std::string reset_graspable_objects_xml = sequenceWrapperXML("reset_graspable_objects",
                                                             {open_gripper_1_xml,
                                                              remove_obj_1_xml, open_gripper_2_xml,
                                                              remove_obj_2_xml});

  // General startup sequence:
  std::string startup_sequence_xml = sequenceWrapperXML("StartUpSequence",
                                                      {check_reset_robot_1_xml,
                                                       check_reset_robot_2_xml,
                                                       spawn_fixed_objects_xml,
                                                       reset_graspable_objects_xml,
                                                       parallel_sub_startup_sequences_xml});

  // ROBOT 1
  // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other
  // children
  std::string repeat_forever_wrapper_1_xml = repeatSequenceWrapperXML("RepeatForever",
                                                                    {check_reset_robot_1_xml, //< We
                                                                                              // check
                                                                                              // if
                                                                                              // the
                                                                                              // robot
                                                                                              // is
                                                                                              // active,
                                                                                              // if
                                                                                              // not
                                                                                              // we
                                                                                              // try
                                                                                              // to
                                                                                              // reset
                                                                                              // it
                                                                     spawn_graspable_objects_1_xml, //<
                                                                                                    // We
                                                                                                    // add
                                                                                                    // all
                                                                                                    // the
                                                                                                    // objects
                                                                                                    // to
                                                                                                    // the
                                                                                                    // scene
                                                                     get_grasp_object_poses_1_xml, //<
                                                                                                   // We
                                                                                                   // get
                                                                                                   // the
                                                                                                   // updated
                                                                                                   // poses
                                                                                                   // relative
                                                                                                   // to
                                                                                                   // the
                                                                                                   // objects
                                                                     go_to_pick_pose_1_xml, //< Prep
                                                                                            // sequence
                                                                                            // and
                                                                                            // pick
                                                                                            // sequence
                                                                     close_gripper_1_xml, //< We
                                                                                          // attach
                                                                                          // the
                                                                                          // object
                                                                     drop_sequence_1_xml, //< Drop
                                                                                          // sequence
                                                                     open_gripper_1_xml, //< We
                                                                                         // detach
                                                                                         // the
                                                                                         // object
                                                                     home_sequence_1_xml, //< Homing
                                                                                          // sequence
                                                                     remove_obj_1_xml}, //< We
                                                                                        // delete
                                                                                        // the
                                                                                        // object
                                                                                        // for it to
                                                                                        // be added
                                                                                        // on the
                                                                                        // next
                                                                                        // cycle in
                                                                                        // the
                                                                                        // original
                                                                                        // position
                                                                      -1); //< num_cycles=-1 for
                                                                           // infinite

  // ROBOT 2
  // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other
  // children
  std::string repeat_forever_wrapper_2_xml = repeatSequenceWrapperXML("RepeatForever",
                                                                    {check_reset_robot_2_xml, //< We
                                                                                              // check
                                                                                              // if
                                                                                              // the
                                                                                              // robot
                                                                                              // is
                                                                                              // active,
                                                                                              // if
                                                                                              // not
                                                                                              // we
                                                                                              // try
                                                                                              // to
                                                                                              // reset
                                                                                              // it
                                                                     spawn_graspable_objects_2_xml, //<
                                                                                                    // We
                                                                                                    // add
                                                                                                    // all
                                                                                                    // the
                                                                                                    // objects
                                                                                                    // to
                                                                                                    // the
                                                                                                    // scene
                                                                     get_grasp_object_poses_2_xml, //<
                                                                                                   // We
                                                                                                   // get
                                                                                                   // the
                                                                                                   // updated
                                                                                                   // poses
                                                                                                   // relative
                                                                                                   // to
                                                                                                   // the
                                                                                                   // objects
                                                                     go_to_pick_pose_2_xml, //< Prep
                                                                                            // sequence
                                                                                            // and
                                                                                            // pick
                                                                                            // sequence
                                                                     close_gripper_2_xml, //< We
                                                                                          // attach
                                                                                          // the
                                                                                          // object
                                                                     drop_sequence_2_xml, //< Drop
                                                                                          // sequence
                                                                     open_gripper_2_xml, //< We
                                                                                         // detach
                                                                                         // the
                                                                                         // object
                                                                     home_sequence_2_xml, //< Homing
                                                                                          // sequence
                                                                     remove_obj_2_xml}, //< We
                                                                                        // delete
                                                                                        // the
                                                                                        // object
                                                                                        // for it to
                                                                                        // be added
                                                                                        // on the
                                                                                        // next
                                                                                        // cycle in
                                                                                        // the
                                                                                        // original
                                                                                        // position
                                                                      -1); //< num_cycles=-1 for
                                                                           // infinite

  // Runningh both robot sequences in parallel:
  std::string parallel_repeat_forever_sequences_xml = parallelWrapperXML(
    "PARALLEL_MOTION_SEQUENCES",
    {repeat_forever_wrapper_1_xml, repeat_forever_wrapper_2_xml},
    2,
    1);

  std::string retry_forever_wrapper_xml = retrySequenceWrapperXML("CycleForever",
                                                                {startup_sequence_xml,
                                                                 parallel_repeat_forever_sequences_xml},
                                                                  -1);

  // MasterSequence with startup sequence and RepeatForever as child to set BehaviorTree ID and root
  // main_tree_to_execute in the XML
  std::vector<std::string> master_branches_xml = {retry_forever_wrapper_xml};
  std::string master_body = sequenceWrapperXML("GlobalMasterSequence", master_branches_xml);

  // ----------------------------------------------------------------------------
  // 7) Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
  // ----------------------------------------------------------------------------

  std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

  RCLCPP_INFO(node->get_logger(),
              "=== Programmatically Generated Tree XML ===\n%s",
              final_tree_xml.c_str());

  // 8) Register node types
  BT::BehaviorTreeFactory factory;
  registerAllNodeTypes(factory);

  // 9) Create the tree from final_tree_xml
  BT::Tree tree;
  try {
    tree = factory.createTreeFromText(final_tree_xml, blackboard);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create tree: %s", ex.what());
    return 1;
  }

  // 10) ZMQ publisher (optional, to visualize in Groot)
  BT::PublisherZMQ publisher(tree);

  // Create the HMI Service Node and pass the same blackboard ***
  auto hmi_node = std::make_shared<manymove_cpp_trees::HMIServiceNode>("hmi_service_node",
                                                                       blackboard, keys);
  RCLCPP_INFO(node->get_logger(), "HMI Service Nodes instantiated.");

  // Create a MultiThreadedExecutor so that both nodes can be spun concurrently.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(hmi_node);

  // 11) Tick the tree in a loop.
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    executor.spin_some();
    BT::NodeStatus status = tree.tickRoot();

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "BT ended SUCCESS.");
      break;
    }
    else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "BT ended FAILURE.");
      break;
    }
    rate.sleep();
  }

  tree.rootNode()->halt();
  rclcpp::shutdown();
  return 0;
}
