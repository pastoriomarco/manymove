#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/decorators/force_failure_node.h>

#include "manymove_cpp_trees/action_nodes_objects.hpp"
#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include "manymove_cpp_trees/action_nodes_signals.hpp"
#include "manymove_cpp_trees/action_nodes_logic.hpp"
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bt_client_node");
    RCLCPP_INFO(node->get_logger(), "BT Client Node started (Purely Programmatic XML).");

    std::string robot_model_1;
    node->declare_parameter<std::string>("robot_model_1", "lite6");
    node->get_parameter_or<std::string>("robot_model_1", robot_model_1, "");

    std::string robot_model_2;
    node->declare_parameter<std::string>("robot_model_2", "uf850");
    node->get_parameter_or<std::string>("robot_model_2", robot_model_2, "");

    // This parameter indicates the prefix to apply to the robot's action servers
    std::string robot_prefix_1;
    node->declare_parameter<std::string>("robot_prefix_1", "L_");
    node->get_parameter_or<std::string>("robot_prefix_1", robot_prefix_1, "");

    std::string robot_prefix_2;
    node->declare_parameter<std::string>("robot_prefix_2", "R_");
    node->get_parameter_or<std::string>("robot_prefix_2", robot_prefix_2, "");

    std::string tcp_frame_1;
    node->declare_parameter<std::string>("tcp_frame_1", "");
    node->get_parameter_or<std::string>("tcp_frame_1", tcp_frame_1, "");

    std::string tcp_frame_2;
    node->declare_parameter<std::string>("tcp_frame_2", "");
    node->get_parameter_or<std::string>("tcp_frame_2", tcp_frame_2, "");

    // This parameter is to be set true if we are connected to a real robot that exposes the necessary services for manymove_signals
    bool is_robot_real;
    node->declare_parameter<bool>("is_robot_real", false);
    node->get_parameter_or<bool>("is_robot_real", is_robot_real, false);

    // ----------------------------------------------------------------------------
    // 1) Create blackboard, keys and nodes
    // ----------------------------------------------------------------------------

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

    /**
     * The following keys are important for the execution control logic: they are modified through
     * the HMI services and let you pause/stop, resume or abort/reset execution.
     */
    // Setting blackboard keys to control execution:

    // Robot 1
    blackboard->set(robot_prefix_1 + "collision_detected", false);
    blackboard->set(robot_prefix_1 + "stop_execution", true);
    blackboard->set(robot_prefix_1 + "execution_resumed", false);

    // Robot 2
    blackboard->set(robot_prefix_2 + "collision_detected", false);
    blackboard->set(robot_prefix_2 + "stop_execution", true);
    blackboard->set(robot_prefix_2 + "execution_resumed", false);

    // General:
    blackboard->set(robot_prefix_1 + "abort_mission", false);
    blackboard->set(robot_prefix_2 + "abort_mission", false);
    RCLCPP_INFO(node->get_logger(), "Blackboard: created execution control keys");

    // Create the HMI Service Node and pass the same blackboard ***
    auto hmi_node_1 = std::make_shared<manymove_cpp_trees::HMIServiceNode>(robot_prefix_1 + "hmi_service_node", blackboard, robot_prefix_1);
    auto hmi_node_2 = std::make_shared<manymove_cpp_trees::HMIServiceNode>(robot_prefix_2 + "hmi_service_node", blackboard, robot_prefix_2);
    RCLCPP_INFO(node->get_logger(), "HMI Service Nodes instantiated.");

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

    // Original pick test poses: they should be overwritten by the blackboard key that will be dynamically updated getting the grasp pose object
    Pose pick_target_1 = createPose(0.2, -0.1, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_pick_target_1 = pick_target_1;
    approach_pick_target_1.position.z += 0.02;

    Pose pick_target_2 = createPose(0.2, 1.1, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_pick_target_2 = pick_target_2;
    approach_pick_target_2.position.z += 0.02;

    // Test poses to place the object, these are not overwritten later (for now)
    Pose drop_target_1 = createPose(0.2, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_drop_target_1 = drop_target_1;
    approach_drop_target_1.position.z += 0.02;

    Pose drop_target_2 = createPose(0.3, 1.05, 0.2, 1.0, 0.0, 0.0, 0.0);
    Pose approach_drop_target_2 = drop_target_2;
    approach_drop_target_2.position.z += 0.02;

    // Populate the blackboard with the poses, one unique key for each pose we want to use.
    // Be careful not to use names that may conflict with the keys automatically created for the moves. (Usually move_{move_id})
    blackboard->set("pick_target_1", pick_target_1);
    blackboard->set("approach_pick_target_1", approach_pick_target_1);

    blackboard->set("drop_target_1", drop_target_1);
    blackboard->set("approach_drop_target_1", approach_drop_target_1);

    blackboard->set("pick_target_2", pick_target_2);
    blackboard->set("approach_pick_target_2", approach_pick_target_2);

    blackboard->set("drop_target_2", drop_target_2);
    blackboard->set("approach_drop_target_2", approach_drop_target_2);

    // Compose the sequences of moves. Each of the following sequences represent a logic
    std::vector<Move> rest_position_1 = {
        {robot_prefix_1, "joint", "", joint_rest_1, "", move_configs["max_move"]},
    };
    std::vector<Move> rest_position_2 = {
        {robot_prefix_2, "joint", "", joint_rest_2, "", move_configs["max_move"]},
    };

    // Sequences for Pick/Drop/Homing
    std::vector<Move> pick_sequence_1 = {
        {robot_prefix_1, "pose", "approach_pick_target_1", {}, "", move_configs["mid_move"]},
        {robot_prefix_1, "cartesian", "pick_target_1", {}, "", move_configs["slow_move"]},
    };

    std::vector<Move> pick_sequence_2 = {
        {robot_prefix_2, "pose", "approach_pick_target_2", {}, "", move_configs["mid_move"]},
        {robot_prefix_2, "cartesian", "pick_target_2", {}, "", move_configs["slow_move"]},
    };

    std::vector<Move> drop_sequence_1 = {
        {robot_prefix_1, "pose", "approach_pick_target_1", {}, "", move_configs["mid_move"]},
        {robot_prefix_1, "pose", "approach_drop_target_1", {}, "", move_configs["max_move"]},
        {robot_prefix_1, "cartesian", "drop_target_1", {}, "", move_configs["slow_move"]},
    };

    std::vector<Move> drop_sequence_2 = {
        {robot_prefix_2, "pose", "approach_pick_target_2", {}, "", move_configs["mid_move"]},
        {robot_prefix_2, "pose", "approach_drop_target_2", {}, "", move_configs["max_move"]},
        {robot_prefix_2, "cartesian", "drop_target_2", {}, "", move_configs["slow_move"]},
    };

    std::vector<Move> home_position_1 = {
        {robot_prefix_1, "cartesian", "approach_drop_target_1", {}, "", move_configs["max_move"]},
        {robot_prefix_1, "named", "", {}, named_home_1, move_configs["max_move"]},
        {robot_prefix_1, "joint", "", joint_rest_1, "", move_configs["max_move"]},
    };

    std::vector<Move> home_position_2 = {
        {robot_prefix_2, "cartesian", "approach_drop_target_2", {}, "", move_configs["max_move"]},
        {robot_prefix_2, "named", "", {}, named_home_2, move_configs["max_move"]},
        {robot_prefix_2, "joint", "", joint_rest_2, "", move_configs["max_move"]},
    };

    // build the xml snippets for the single moves of robot 1
    // or translate them directly if they are only used once
    std::string to_rest_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "toRest", rest_position_1, blackboard, true);

    std::string pick_object_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "pick", pick_sequence_1, blackboard, true);

    std::string drop_object_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "drop", drop_sequence_1, blackboard, true);

    std::string to_home_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "home", home_position_1, blackboard, true);

    // Translate it to xml tree leaf or branch
    std::string prep_sequence_1_xml = sequenceWrapperXML(
        robot_prefix_1 + "ComposedPrepSequence", {to_rest_1_xml});
    std::string pick_sequence_1_xml = sequenceWrapperXML(
        robot_prefix_1 + "ComposedPickSequence", {pick_object_1_xml});
    std::string drop_sequence_1_xml = sequenceWrapperXML(
        robot_prefix_1 + "ComposedDropSequence", {drop_object_1_xml});
    std::string home_sequence_1_xml = sequenceWrapperXML(
        robot_prefix_1 + "ComposedHomeSequence", {to_home_1_xml, to_rest_1_xml});

    // build the xml snippets for the single moves of robot 1
    std::string to_rest_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "toRest", rest_position_2, blackboard, true);

    std::string pick_object_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "pick", pick_sequence_2, blackboard, true);

    std::string drop_object_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "drop", drop_sequence_2, blackboard, true);

    std::string to_home_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "home", home_position_2, blackboard, true);

    // Translate it to xml tree leaf or branch
    std::string prep_sequence_2_xml = sequenceWrapperXML(
        robot_prefix_2 + "ComposedPrepSequence", {to_rest_2_xml});
    std::string pick_sequence_2_xml = sequenceWrapperXML(
        robot_prefix_2 + "ComposedPickSequence", {pick_object_2_xml});
    std::string drop_sequence_2_xml = sequenceWrapperXML(
        robot_prefix_2 + "ComposedDropSequence", {drop_object_2_xml});
    std::string home_sequence_2_xml = sequenceWrapperXML(
        robot_prefix_2 + "ComposedHomeSequence", {to_home_2_xml, to_rest_2_xml});

    // ----------------------------------------------------------------------------
    // 3) Build blocks for objects handling
    // ----------------------------------------------------------------------------

    std::vector<double> ground_dimension = {0.8, 2.0, 0.1};
    auto ground_pose = createPoseRPY(0.0, 0.5, -0.05, 0.0, 0.0, 0.0);

    std::vector<double> wall_dimension = {0.8, 0.02, 0.8};
    auto wall_pose = createPoseRPY(0.0, 0.5, 0.4, 0.0, 0.0, 0.0);

    std::vector<double> cylinderdimension = {0.1, 0.005};
    auto cylinderpose = createPoseRPY(0.2, 0.7, 0.105, 0.0, 1.57, 0.0);

    std::string mesh_file = "package://manymove_object_manager/meshes/unit_tube.stl";

    std::vector<double> mesh_scale = {0.01, 0.01, 0.1};                  //< The tube is vertical with dimension 1m x 1m x 1m. We scale it to 10x10x100 mm
    auto mesh_pose = createPoseRPY(0.1, -0.2, 0.2005, 0.785, 1.57, 0.0); //< We place it on the floor and lay it on its side, X+ facing down

    // Create object actions xml snippets (the object are created directly in the create*() functions relative to each type of object action)
    std::string check_ground_obj_xml = buildObjectActionXML("check_ground", createCheckObjectExists("obstacle_ground"));
    std::string check_wall_obj_xml = buildObjectActionXML("check_wall", createCheckObjectExists("obstacle_wall"));
    std::string check_cylinder_obj_xml = buildObjectActionXML("check_cylinder", createCheckObjectExists("graspable_cylinder"));
    std::string check_mesh_obj_xml = buildObjectActionXML("check_mesh", createCheckObjectExists("graspable_mesh"));

    std::string add_ground_obj_xml = buildObjectActionXML("add_ground", createAddPrimitiveObject("obstacle_ground", "box", ground_dimension, ground_pose));
    std::string add_wall_obj_xml = buildObjectActionXML("add_wall", createAddPrimitiveObject("obstacle_wall", "box", wall_dimension, wall_pose));
    std::string add_cylinder_obj_xml = buildObjectActionXML("add_cylinder", createAddPrimitiveObject("graspable_cylinder", "cylinder", cylinderdimension, cylinderpose));
    std::string add_mesh_obj_xml = buildObjectActionXML("add_mesh", createAddMeshObject("graspable_mesh", mesh_pose, mesh_file, mesh_scale[0], mesh_scale[1], mesh_scale[2]));

    // Compose the check and add sequence for objects
    std::string init_ground_obj_xml = fallbackWrapperXML("init_ground_obj", {check_ground_obj_xml, add_ground_obj_xml});
    std::string init_wall_obj_xml = fallbackWrapperXML("init_wall_obj", {check_wall_obj_xml, add_wall_obj_xml});
    std::string init_cylinder_obj_xml = fallbackWrapperXML("init_cylinder_obj", {check_cylinder_obj_xml, add_cylinder_obj_xml});
    std::string init_mesh_obj_xml = fallbackWrapperXML("init_mesh_obj", {check_mesh_obj_xml, add_mesh_obj_xml});

    // the name of the link to attach the object to, and the object to manipulate
    std::string tcp_frame_name_1 = robot_prefix_1 + tcp_frame_1;
    std::string tcp_frame_name_2 = robot_prefix_2 + tcp_frame_2;
    std::string object_to_manipulate_1 = "graspable_mesh";
    std::string object_to_manipulate_2 = "graspable_cylinder";

    std::string attach_obj_1_xml = buildObjectActionXML("attach_obj_to_manipulate_1", createAttachObject(object_to_manipulate_1, tcp_frame_name_1));
    std::string detach_obj_1_xml = buildObjectActionXML("attach_obj_to_manipulate_1", createDetachObject(object_to_manipulate_1, tcp_frame_name_1));
    std::string remove_obj_1_xml = buildObjectActionXML("remove_obj_to_manipulate_1", createRemoveObject(object_to_manipulate_1));

    std::string attach_obj_2_xml = buildObjectActionXML("attach_obj_to_manipulate_2", createAttachObject(object_to_manipulate_2, tcp_frame_name_2));
    std::string detach_obj_2_xml = buildObjectActionXML("attach_obj_to_manipulate_2", createDetachObject(object_to_manipulate_2, tcp_frame_name_2));
    std::string remove_obj_2_xml = buildObjectActionXML("remove_obj_to_manipulate_2", createRemoveObject(object_to_manipulate_2));

    // ----------------------------------------------------------------------------
    // 4) Add GetObjectPoseAction Node and nodes to attach/detach objects
    // ----------------------------------------------------------------------------

    // Define the object ID and pose_key where the pose will be stored for robot 1
    std::string pick_pose_key_1 = "pick_target_1";
    std::string approach_pose_key_1 = "approach_pick_target_1";

    // Define the transformation and reference orientation
    std::vector<double> pick_pre_transform_xyz_rpy_1 = {-0.002, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> approach_pre_transform_xyz_rpy_1 = {-0.05, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> post_transform_xyz_rpy_1 = {0.0, 0.0, -0.025, 3.14, 0.0, 0.0};

    // Translate get_pose_action to xml tree leaf
    std::string get_pick_pose_1_xml = buildObjectActionXML(
        "get_pick_pose_1", createGetObjectPose(
                               object_to_manipulate_1,
                               pick_pose_key_1,
                               pick_pre_transform_xyz_rpy_1,
                               post_transform_xyz_rpy_1));
    std::string get_approach_pose_1_xml = buildObjectActionXML(
        "get_approach_pose_1", createGetObjectPose(
                                   object_to_manipulate_1,
                                   approach_pose_key_1,
                                   approach_pre_transform_xyz_rpy_1,
                                   post_transform_xyz_rpy_1));

    // Define the object ID and pose_key where the pose will be stored for robot 2
    std::string pick_pose_key_2 = "pick_target_2";
    std::string approach_pose_key_2 = "approach_pick_target_2";

    // Define the transformation and reference orientation
    std::vector<double> pick_pre_transform_xyz_rpy_2 = {-0.002, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> approach_pre_transform_xyz_rpy_2 = {-0.05, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> post_transform_xyz_rpy_2 = {0.0, 0.0, -0.025, 3.14, 0.0, 0.0};

    // Translate get_pose_action to xml tree leaf
    std::string get_pick_pose_2_xml = buildObjectActionXML(
        "get_pick_pose_2", createGetObjectPose(
                               object_to_manipulate_2,
                               pick_pose_key_2,
                               pick_pre_transform_xyz_rpy_2,
                               post_transform_xyz_rpy_2));
    std::string get_approach_pose_2_xml = buildObjectActionXML(
        "get_approach_pose_2", createGetObjectPose(
                                   object_to_manipulate_2,
                                   approach_pose_key_2,
                                   approach_pre_transform_xyz_rpy_2,
                                   post_transform_xyz_rpy_2));

    // ----------------------------------------------------------------------------
    // 5) Define Signals calls:
    // ----------------------------------------------------------------------------

    // Let's send and receive signals only if the robot is real, and let's fake a delay on inputs otherwise
    // Robot 1
    std::string signal_gripper_close_1_xml = (is_robot_real ? buildSetOutputXML(robot_prefix_1, "GripperClose", "controller", 0, 1) : "");
    std::string signal_gripper_open_1_xml = (is_robot_real ? buildSetOutputXML(robot_prefix_1, "GripperOpen", "controller", 0, 0) : "");
    std::string check_gripper_close_1_xml = (is_robot_real ? buildCheckInputXML(robot_prefix_1, "WaitForSensor", "controller", 0, 1, true, 0) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");
    std::string check_gripper_open_1_xml = (is_robot_real ? buildCheckInputXML(robot_prefix_1, "WaitForSensor", "controller", 0, 0, true, 0) : "<Delay delay_msec=\"250\">\n  <AlwaysSuccess />\n</Delay>\n");
    std::string check_robot_state_1_xml = buildCheckRobotStateXML(robot_prefix_1, "CheckRobot", "robot_ready", "error_code", "robot_mode", "robot_state", "robot_msg");
    std::string reset_robot_state_1_xml = buildResetRobotStateXML(robot_prefix_1, "ResetRobot", robot_model_1);

    std::string check_reset_robot_1_xml = (is_robot_real ? fallbackWrapperXML(robot_prefix_1 + "CheckResetFallback", {check_robot_state_1_xml, reset_robot_state_1_xml}) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

    // Robot 2
    std::string signal_gripper_close_2_xml = (is_robot_real ? buildSetOutputXML(robot_prefix_2, "GripperClose", "controller", 0, 1) : "");
    std::string signal_gripper_open_2_xml = (is_robot_real ? buildSetOutputXML(robot_prefix_2, "GripperOpen", "controller", 0, 0) : "");
    std::string check_gripper_close_2_xml = (is_robot_real ? buildCheckInputXML(robot_prefix_2, "WaitForSensor", "controller", 0, 1, true, 0) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");
    std::string check_gripper_open_2_xml = (is_robot_real ? buildCheckInputXML(robot_prefix_2, "WaitForSensor", "controller", 0, 0, true, 0) : "<Delay delay_msec=\"250\">\n  <AlwaysSuccess />\n</Delay>\n");
    std::string check_robot_state_2_xml = buildCheckRobotStateXML(robot_prefix_2, "CheckRobot", "robot_ready", "error_code", "robot_mode", "robot_state", "robot_msg");
    std::string reset_robot_state_2_xml = buildResetRobotStateXML(robot_prefix_2, "ResetRobot", robot_model_2);

    std::string check_reset_robot_2_xml = (is_robot_real ? fallbackWrapperXML(robot_prefix_2 + "CheckResetFallback", {check_robot_state_2_xml, reset_robot_state_2_xml}) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

    // ----------------------------------------------------------------------------
    // 6) Combine the objects and moves in a sequences that can run a number of times:
    // ----------------------------------------------------------------------------

    // Let's build the full sequence in logically separated blocks:
    // General
    std::string spawn_fixed_objects_xml = sequenceWrapperXML("SpawnFixedObjects", {init_ground_obj_xml, init_wall_obj_xml});

    // Robot 1
    std::string get_grasp_object_poses_1_xml = sequenceWrapperXML("GetGraspPoses", {get_pick_pose_1_xml, get_approach_pose_1_xml});
    std::string go_to_pick_pose_1_xml = sequenceWrapperXML("GoToPickPose", {pick_sequence_1_xml});
    std::string close_gripper_1_xml = sequenceWrapperXML("CloseGripper", {signal_gripper_close_1_xml, check_gripper_close_1_xml, attach_obj_1_xml});
    std::string open_gripper_1_xml = sequenceWrapperXML("OpenGripper", {signal_gripper_open_1_xml, detach_obj_1_xml});

    // Robot 2
    std::string get_grasp_object_poses_2_xml = sequenceWrapperXML("GetGraspPoses", {get_pick_pose_2_xml, get_approach_pose_2_xml});
    std::string go_to_pick_pose_2_xml = sequenceWrapperXML("GoToPickPose", {pick_sequence_2_xml});
    std::string close_gripper_2_xml = sequenceWrapperXML("CloseGripper", {signal_gripper_close_2_xml, check_gripper_close_2_xml, attach_obj_2_xml});
    std::string open_gripper_2_xml = sequenceWrapperXML("OpenGripper", {signal_gripper_open_2_xml, detach_obj_2_xml});

    // Dedicated spawnable objects per robot:
    std::string spawn_graspable_objects_1_xml = sequenceWrapperXML("SpawnGraspableObjects", {init_mesh_obj_xml});
    std::string spawn_graspable_objects_2_xml = sequenceWrapperXML("SpawnGraspableObjects", {init_cylinder_obj_xml});

    // Parallel startup subsequences:
    std::string startup_sequence_1_xml = sequenceWrapperXML("StartUpSequence_1", {check_reset_robot_1_xml, prep_sequence_1_xml});
    std::string startup_sequence_2_xml = sequenceWrapperXML("StartUpSequence_2", {check_reset_robot_2_xml, prep_sequence_2_xml});
    std::string parallel_sub_startup_sequences_xml = parallelWrapperXML("Parallel_startupSequences", {startup_sequence_1_xml, startup_sequence_2_xml}, 2, 1);

    // General startup sequence:
    std::string startup_sequence_xml = sequenceWrapperXML("StartUpSequence", {check_reset_robot_1_xml, spawn_fixed_objects_xml, parallel_sub_startup_sequences_xml});

    // ROBOT 1
    // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other children
    std::string repeat_forever_wrapper_1_xml = repeatWrapperXML(
        "RepeatForever",
        {check_reset_robot_1_xml,       //< We check if the robot is active, if not we try to reset it
         spawn_graspable_objects_1_xml, //< We add all the objects to the scene
         get_grasp_object_poses_1_xml,  //< We get the updated poses relative to the objects
         go_to_pick_pose_1_xml,         //< Prep sequence and pick sequence
         close_gripper_1_xml,           //< We attach the object
         drop_sequence_1_xml,           //< Drop sequence
         open_gripper_1_xml,            //< We detach the object
         home_sequence_1_xml,           //< Homing sequence
         remove_obj_1_xml},             //< We delete the object for it to be added on the next cycle in the original position
        -1);                            //< num_cycles=-1 for infinite

    // ROBOT 2
    // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other children
    std::string repeat_forever_wrapper_2_xml = repeatWrapperXML(
        "RepeatForever",
        {check_reset_robot_2_xml,       //< We check if the robot is active, if not we try to reset it
         spawn_graspable_objects_2_xml, //< We add all the objects to the scene
         get_grasp_object_poses_2_xml,  //< We get the updated poses relative to the objects
         go_to_pick_pose_2_xml,         //< Prep sequence and pick sequence
         close_gripper_2_xml,           //< We attach the object
         drop_sequence_2_xml,           //< Drop sequence
         open_gripper_2_xml,            //< We detach the object
         home_sequence_2_xml,           //< Homing sequence
         remove_obj_2_xml},             //< We delete the object for it to be added on the next cycle in the original position
        -1);                            //< num_cycles=-1 for infinite

    // Runningh both robot sequences in parallel:
    std::string parallel_repeat_forever_sequences_xml = parallelWrapperXML("PARALLEL_MOTION_SEQUENCES", {repeat_forever_wrapper_1_xml, repeat_forever_wrapper_2_xml}, 2, 1);

    // MasterSequence with startup sequence and RepeatForever as child to set BehaviorTree ID and root main_tree_to_execute in the XML
    std::vector<std::string> master_branches_xml = {startup_sequence_xml, parallel_repeat_forever_sequences_xml};
    std::string master_body = sequenceWrapperXML("GlobalMasterSequence", master_branches_xml);

    // ----------------------------------------------------------------------------
    // 7) Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    // ----------------------------------------------------------------------------

    std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    // 8) Register node types
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<PlanningAction>("PlanningAction");
    factory.registerNodeType<ExecuteTrajectory>("ExecuteTrajectory");
    factory.registerNodeType<ResetTrajectories>("ResetTrajectories");

    factory.registerNodeType<AddCollisionObjectAction>("AddCollisionObjectAction");
    factory.registerNodeType<RemoveCollisionObjectAction>("RemoveCollisionObjectAction");
    factory.registerNodeType<AttachDetachObjectAction>("AttachDetachObjectAction");
    factory.registerNodeType<CheckObjectExistsAction>("CheckObjectExistsAction");
    factory.registerNodeType<GetObjectPoseAction>("GetObjectPoseAction");

    factory.registerNodeType<SetOutputAction>("SetOutputAction");
    factory.registerNodeType<GetInputAction>("GetInputAction");
    factory.registerNodeType<CheckRobotStateAction>("CheckRobotStateAction");
    factory.registerNodeType<ResetRobotStateAction>("ResetRobotStateAction");
    factory.registerNodeType<StopMotionAction>("StopMotionAction");

    factory.registerNodeType<CheckBlackboardKeyValue>("CheckBlackboardKeyValueInt");
    factory.registerNodeType<SetBlackboardKeyValue>("SetBlackboardKeyValue");
    factory.registerNodeType<BT::RetryNode>("RetryNode");
    factory.registerNodeType<RetryPauseAbortNode>("RetryPauseAbortNode");

    // 9) Create the tree from final_tree_xml
    BT::Tree tree;
    try
    {
        tree = factory.createTreeFromText(final_tree_xml, blackboard);
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to create tree: %s", ex.what());
        return 1;
    }

    // 10) ZMQ publisher (optional, to visualize in Groot)
    BT::PublisherZMQ publisher(tree);

    // Create a MultiThreadedExecutor so that both nodes can be spun concurrently.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(hmi_node_1);
    executor.add_node(hmi_node_2);

    // 11) Tick the tree in a loop.
    rclcpp::Rate rate(100);
    while (rclcpp::ok())
    {
        executor.spin_some();
        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "BT ended SUCCESS.");
            break;
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_ERROR(node->get_logger(), "BT ended FAILURE.");
            break;
        }
        rate.sleep();
    }

    tree.rootNode()->halt();
    rclcpp::shutdown();
    return 0;
}
