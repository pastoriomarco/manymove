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

#include "manymove_cpp_trees/main_imports_helper.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("bt_client_node");
  RCLCPP_INFO(node->get_logger(), "BT Client Node started for UR manipulators.");

  // ----------------------------------------------------------------------------
  // 1) Create a blackboard and set "node"
  // ----------------------------------------------------------------------------
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

  std::vector<manymove_cpp_trees::BlackboardEntry> keys;

  // Provide UR-friendly defaults (model falls back to ur3e, tcp frame to tool0).
  RobotParams rp = defineRobotParams(node, blackboard, keys, "", "ur3e", "", "tool0");

  // ----------------------------------------------------------------------------
  // 2) Setup joint targets, poses and moves
  // ----------------------------------------------------------------------------

  auto move_configs = defineMovementConfigs();

  // Adjusting only the move params of default moves
  auto & max_move = move_configs["max_move"];
  max_move.planner_id = "RRTConnectkConfigDefault";
  max_move.planning_time = 0.05;
  max_move.plan_number_limit = 32;
  max_move.plan_number_target = 12;

  auto & mid_move = move_configs["mid_move"];
  mid_move.planner_id = "RRTConnectkConfigDefault";
  mid_move.planning_time = 0.05;
  mid_move.plan_number_limit = 32;
  mid_move.plan_number_target = 12;

  auto & slow_move = move_configs["slow_move"];
  slow_move.planner_id = "RRTConnectkConfigDefault";
  slow_move.planning_time = 0.05;
  slow_move.plan_number_limit = 32;
  slow_move.plan_number_target = 12;

  // Typical UR 6-DOF configurations (radians) for rest and scanning poses.
  std::vector<double> joint_rest = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  std::string named_home = "home";

  blackboard->set("pick_target_key", Pose());
  blackboard->set("approach_pick_target_key", Pose());

  Pose drop_target = createPoseRPY(0.3, 0.3, 0.25, 3.14, 0.0, -1.57);
  blackboard->set("drop_target_key", drop_target);

  Pose approach_drop_target = drop_target;
  approach_drop_target.position.z += 0.05;
  blackboard->set("approach_drop_target_key", approach_drop_target);

  std::string tcp_frame_name = rp.prefix + rp.tcp_frame;

  std::vector<Move> rest_position = {
    {rp.prefix, tcp_frame_name, "joint", move_configs["max_move"], "", joint_rest},
  };

  std::vector<Move> pick_sequence = {
    {rp.prefix, tcp_frame_name, "pose", move_configs["max_move"], "approach_pick_target_key"},
    {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_slow_move"],
      "pick_target_key"},
  };

  std::vector<Move> drop_sequence = {
    {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_mid_move"],
      "approach_pick_target_key"},
    // {rp.prefix, tcp_frame_name, "joint", move_configs["max_move"], "", joint_rest},
    {rp.prefix, tcp_frame_name, "pose", move_configs["max_move"], "approach_drop_target_key"},
    {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_slow_move"],
      "drop_target_key"},
  };

  std::vector<Move> exit_drop_position = {
    {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_mid_move"],
      "approach_drop_target_key"},
    // {rp.prefix, tcp_frame_name, "named", move_configs["max_move"], "", {}, named_home},
  };

  std::string to_rest_reset_xml =
    buildMoveXML(rp.prefix, rp.prefix + "toRest", rest_position, blackboard, true);
  std::string to_rest_xml =
    buildMoveXML(rp.prefix, rp.prefix + "toRest", rest_position, blackboard);
  std::string pick_object_xml =
    buildMoveXML(rp.prefix, rp.prefix + "pick", pick_sequence, blackboard);
  std::string drop_object_xml =
    buildMoveXML(rp.prefix, rp.prefix + "drop", drop_sequence, blackboard);
  std::string to_drop_exit_xml = buildMoveXML(
    rp.prefix, rp.prefix + "home", exit_drop_position,
    blackboard);

  std::string prep_sequence_xml =
    sequenceWrapperXML(rp.prefix + "ComposedPrepSequence", {to_rest_reset_xml});
  std::string home_sequence_xml =
    sequenceWrapperXML(rp.prefix + "ComposedHomeSequence", {to_drop_exit_xml, to_rest_xml});

  // ----------------------------------------------------------------------------
  // 3) Build blocks for objects handling
  // ----------------------------------------------------------------------------

  blackboard->set("tcp_frame_name_key", tcp_frame_name);
  blackboard->set(
    "touch_links",
    std::vector<std::string>{"robotiq_85_right_finger_tip_link",
      "robotiq_85_left_finger_tip_link"});

  blackboard->set("world_frame_key", "world");
  blackboard->set("identity_transform_key", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Objects in the scene:
  // This is the new unified helper function to create all the snippets to handle any kind of
  // objects
  ObjectSnippets ground = createObjectSnippets(
    blackboard, keys, "ground",  // object name
    "box",  // shape
    createPoseRPY(0.0, 0.0, -0.051, 0.0, 0.0, 0.0),  // pose of the object*/
    {1.0, 1.0, 0.1},  // primitive dimensions
    "",  // mesh file path
    {1.0, 1.0, 1.0},  // scale
    "",  // link name to attach/detach
    {}  // contact links to attach/detach
  );

  ObjectSnippets wall = createObjectSnippets(
    blackboard, keys, "wall", "box", createPoseRPY(0.0, -0.15, 0.1, 0.0, 0.0, 0.0),
    {1.0, 0.02, 0.2});

  ObjectSnippets graspable = createObjectSnippets(
    blackboard, keys, "graspable", "box", createPoseRPY(0.15, -0.35, 0.1, 0.0, 0.0, -0.785),
    {0.1, 0.01, 0.01}, "", {1.0, 1.0, 1.0}, "tcp_frame_name_key", "touch_links");

  blackboard->set(
    "pick_pre_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, -0.1675, 0.0, 0.0, 0.0});
  blackboard->set(
    "approach_pre_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, -0.225, 0.0, 0.0, 0.0});
  blackboard->set(
    "post_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, 0.0, 3.14, 0.0, 1.57});

  // Translate get_pose_action to xml tree leaf
  std::string get_pick_pose_xml = buildObjectActionXML(
    "get_pick_pose", createGetObjectPose(
      "graspable_key", "pick_target_key", "world_frame_key",
      "pick_pre_transform_xyz_rpy_1_key", "post_transform_xyz_rpy_1_key"));

  std::string get_approach_pose_xml = buildObjectActionXML(
    "get_approach_pose", createGetObjectPose(
      "graspable_key", "approach_pick_target_key", "world_frame_key",
      "approach_pre_transform_xyz_rpy_1_key", "post_transform_xyz_rpy_1_key"));

  // ----------------------------------------------------------------------------
  // 4) Define Signals calls
  // ----------------------------------------------------------------------------

  std::string gripper_action_server = rp.gripper_action_server;
  const std::string deprecated_suffix = "gripper_command";
  if (
    !gripper_action_server.empty() &&
    gripper_action_server.size() >= deprecated_suffix.size() &&
    gripper_action_server.compare(
      gripper_action_server.size() - deprecated_suffix.size(), deprecated_suffix.size(),
      deprecated_suffix) == 0)
  {
    std::string converted =
      gripper_action_server.substr(0, gripper_action_server.size() - deprecated_suffix.size()) +
      "gripper_cmd";
    RCLCPP_WARN(
      node->get_logger(),
      "Gripper action server '%s' uses deprecated suffix 'gripper_command'; using '%s' instead.",
      gripper_action_server.c_str(), converted.c_str());
    gripper_action_server = converted;
    rp.gripper_action_server = converted;
  }

  const bool has_gripper_action_server = !gripper_action_server.empty();
  if (!has_gripper_action_server) {
    RCLCPP_WARN(
      node->get_logger(),
      "Param. 'gripper_action_server' empty; falling back to timed delays for gripper control.");
  }
  const std::string gripper_close_action_xml =
    (has_gripper_action_server ?
    "<GripperCommandAction position=\"0.75\" max_effort=\"40.0\" action_server=\"" +
    gripper_action_server + "\"/>" :
    "<Delay delay_msec=\"500\">\n  <AlwaysSuccess />\n</Delay>\n");
  const std::string gripper_open_action_xml =
    (has_gripper_action_server ?
    "<GripperCommandAction position=\"0.25\" max_effort=\"40.0\" action_server=\"" +
    gripper_action_server + "\"/>" :
    "<Delay delay_msec=\"500\">\n  <AlwaysSuccess />\n</Delay>\n");
  std::string check_robot_state_xml = buildCheckRobotStateXML(
    rp.prefix, "CheckRobot", "robot_ready", "error_code", "robot_mode", "robot_state", "robot_msg");
  std::string reset_robot_state_xml = buildResetRobotStateXML(rp.prefix, "ResetRobot", rp.model);

  std::string check_reset_robot_xml =
    (rp.is_real ?
    fallbackWrapperXML(
      rp.prefix + "CheckResetFallback", {check_robot_state_xml, reset_robot_state_xml}) :
    "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

  // ----------------------------------------------------------------------------
  // 5) Full behavior composition
  // ----------------------------------------------------------------------------

  std::string spawn_fixed_objects_xml =
    sequenceWrapperXML("SpawnFixedObjects", {ground.init_xml, wall.init_xml});
  std::string spawn_graspable_objects_xml =
    sequenceWrapperXML("SpawnGraspableObjects", {graspable.init_xml});
  std::string get_grasp_object_poses_xml =
    sequenceWrapperXML("GetGraspPoses", {get_pick_pose_xml, get_approach_pose_xml});
  std::string go_to_pick_pose_xml = sequenceWrapperXML("GoToPickPose", {pick_object_xml});
  std::string close_gripper_xml = sequenceWrapperXML(
    "CloseGripper", {gripper_close_action_xml, graspable.attach_xml});
  std::string open_gripper_xml =
    sequenceWrapperXML("OpenGripper", {gripper_open_action_xml, graspable.detach_xml});

  std::string reset_graspable_objects_xml =
    sequenceWrapperXML("reset_graspable_objects", {open_gripper_xml, graspable.remove_xml});

  std::string startup_sequence_xml = sequenceWrapperXML(
    "StartUpSequence", {check_reset_robot_xml, spawn_fixed_objects_xml,
      open_gripper_xml, prep_sequence_xml});

  std::string repeat_forever_wrapper_xml = repeatSequenceWrapperXML(
    "RepeatForever",
  {
    check_reset_robot_xml,
    spawn_graspable_objects_xml,
    get_grasp_object_poses_xml,
    go_to_pick_pose_xml,
    close_gripper_xml,
    drop_object_xml,
    open_gripper_xml,
    home_sequence_xml,
    graspable.remove_xml
  },
    -1);

  std::string retry_forever_wrapper_xml =
    retrySequenceWrapperXML("CycleForever", {startup_sequence_xml, repeat_forever_wrapper_xml}, -1);

  std::vector<std::string> master_branches_xml = {retry_forever_wrapper_xml};
  std::string master_body = sequenceWrapperXML("GlobalMasterSequence", master_branches_xml);
  std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

  RCLCPP_INFO(
    node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

  // ----------------------------------------------------------------------------
  // 6) Register node types and build tree
  // ----------------------------------------------------------------------------

  BT::BehaviorTreeFactory factory;
  registerAllNodeTypes(factory);

  BT::Tree tree;
  try {
    tree = factory.createTreeFromText(final_tree_xml, blackboard);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create tree: %s", ex.what());
    return 1;
  }

  BT::PublisherZMQ publisher(tree);
  (void)publisher;

  auto hmi_node =
    std::make_shared<manymove_cpp_trees::HMIServiceNode>("hmi_service_node", blackboard, keys);
  RCLCPP_INFO(node->get_logger(), "HMI Service Node instantiated.");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(hmi_node);

  manymove_cpp_trees::setHmiMessage(
    blackboard, rp.prefix, "Waiting for start command", "green");

  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    executor.spin_some();
    BT::NodeStatus status = tree.tickRoot();

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "BT ended SUCCESS.");
      break;
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "BT ended FAILURE.");
      break;
    }
    rate.sleep();
  }

  tree.rootNode()->halt();
  rclcpp::shutdown();
  return 0;
}
