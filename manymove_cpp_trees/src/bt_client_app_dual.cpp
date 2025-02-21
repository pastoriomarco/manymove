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

    double tube_length;
    node->declare_parameter<double>("tube_length", 0.1);
    node->get_parameter_or<double>("tube_length", tube_length, 0.1);

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
    blackboard->set(robot_prefix_1 + "stop_execution", false);
    blackboard->set(robot_prefix_1 + "execution_resumed", false);

    // Robot 2
    blackboard->set(robot_prefix_2 + "collision_detected", false);
    blackboard->set(robot_prefix_2 + "stop_execution", false);
    blackboard->set(robot_prefix_2 + "execution_resumed", false);

    // General:
    blackboard->set(robot_prefix_1 + "abort_mission", false);
    blackboard->set(robot_prefix_2 + "abort_mission", false);
    RCLCPP_INFO(node->get_logger(), "Blackboard: created execution control keys");

    // Create the HMI Service Node and pass the same blackboard ***
    auto hmi_node_1 = std::make_shared<manymove_cpp_trees::HMIServiceNode>(robot_prefix_1 + "hmi_service_node", blackboard, robot_prefix_1);
    auto hmi_node_2 = std::make_shared<manymove_cpp_trees::HMIServiceNode>(robot_prefix_2 + "hmi_service_node", blackboard, robot_prefix_2);
    RCLCPP_INFO(node->get_logger(), "HMI Service Nodes instantiated.");

    //

    // ----------------------------------------------------------------------------
    // 2) Build blocks for poses and objects handling
    // ----------------------------------------------------------------------------

    //

    // Original pick pose: it will be overwritten by the blackboard key that will be dynamically updated getting the grasp pose object.
    // Note that the pose obtained from an object will refer to the object itels and will have to be modified if you want to align
    // the TCP orientation to a direction that is not the object's Z+ (more about it later).
    // I may also define a pick pose here to align the TCP as we want since we know the position of the object, but this mechanism
    // will allow me to get the pose recognized by a vision system and to realign the TCP Z+ to the desired pick orientation.
    Pose pick_target_1 = createPose(0.2, -0.1, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_pick_target_1 = pick_target_1;
    approach_pick_target_1.position.z += 0.02;

    // Drop pose: this is not overwritten later (for now)
    // Note that, this pose directly refer to the pose that the TCP will allign to and it's referred to the world frame, regardless of
    // how the robot is oriented. For example, this pose has the Z+ facing downard, 45 degrees in the XZ plane, in between X- and Z-.
    Pose drop_target_1 = createPoseRPY(0.57, -0.25, 0.72, -0.785, -3.14, 1.57);
    Pose approach_drop_target_1 = drop_target_1;
    // We need to modify the exit pose accordingly: we move 5cm in X+ and Z+ to obtain a 45 degree exit in the XZ plane, in the opposite
    // direction of the Z+ of the TCP in the drop_target_1 pose.
    approach_drop_target_1.position.x += 0.075;
    approach_drop_target_1.position.z += 0.075;

    // For each pose to get/set dynamically, rember to create a unique blackboard key accordingly.
    // Be careful not to use names that may conflict with the keys automatically
    // created for the moves. (Usually move_{move_id})
    // We also create a string to store the key name, or we'll risk to misuse it later
    std::string pick_target_1_key_name = "pick_target_1";
    std::string approach_pick_target_1_key_name = "approach_pick_target_1";
    blackboard->set(pick_target_1_key_name, pick_target_1);
    blackboard->set(approach_pick_target_1_key_name, approach_pick_target_1);

    std::string drop_target_1_key_name = "drop_target_1";
    std::string approach_drop_target_1_key_name = "approach_drop_target_1";
    blackboard->set(drop_target_1_key_name, drop_target_1);
    blackboard->set(approach_drop_target_1_key_name, approach_drop_target_1);

    // Once the object will be moved, the drop position will have to be retrieved with the new pose of the object, so I need a new blackboard
    // key for the new pose:
    std::string dropped_target_key_name = "dropped_target";
    // then I set a temporary value, it will be overwritten later:
    blackboard->set(dropped_target_key_name, drop_target_1);

    // While the pick/drop pose is either dependant of object pose or fixed, we can set a fixed insert pose and a load pose that will depend
    // on the length of the object. For this reason we've set a variable, and we also set a blackboard key here for the object's length.
    // The blackboard key will let us set the length dynamically from the HMI if needed, without shutting down the application.
    blackboard->set("tube_length", tube_length);

    // We also set an grasp offset from the end of the tube, we'll use it later to define the pick pose, and now to define the insert pose
    double grasp_offset = 0.025;

    // We keep the naming consistent with the robot_prefix associated, or else the move will fail to plan!
    // The insert pose will depend on the drop_target_1 pose.
    Pose insert_target_2 = drop_target_1;
    insert_target_2.position.y += (grasp_offset + 0.002);
    // We approach from outside the insert position to factor the insert's length
    Pose approach_insert_target_2 = insert_target_2;
    approach_insert_target_2.position.y += 0.075;

    std::string insert_target_2_key_name = "insert_target_2";
    std::string approach_insert_target_2_key_name = "approach_insert_target_2";
    blackboard->set(insert_target_2_key_name, insert_target_2);
    blackboard->set(approach_insert_target_2_key_name, approach_insert_target_2);

    std::string object_to_manipulate_1 = "graspable_mesh";
    std::string object_to_manipulate_2 = "renamed_mesh";

    std::string graspable_mesh_file = "package://manymove_object_manager/meshes/unit_tube.stl";
    std::string machine_mesh_file = "package://manymove_object_manager/meshes/custom_scene/machine.stl";
    std::string endplate_mesh_file = "package://manymove_object_manager/meshes/custom_scene/end_plate.stl";
    std::string slider_mesh_file = "package://manymove_object_manager/meshes/custom_scene/slider.stl";

    std::vector<double> graspable_mesh_scale = {0.01, 0.01, tube_length};
    auto graspable_mesh_pose = createPoseRPY(((tube_length / 2) + 0.973 + 0.005), -0.6465, 0.8055, 1.57, 2.05, 1.57);

    // std::vector<double> cylinderdimension = {0.01, 0.1};
    // auto cylinderpose = createPoseRPY(0.571, -0.6235, 0.725, 1.57, 3.14, 0.0);
    // std::string check_cylinder_obj_xml = buildObjectActionXML("check_cylinder", createCheckObjectExists("graspable_cylinder"));
    // std::string add_cylinder_obj_xml = buildObjectActionXML("add_cylinder", createAddPrimitiveObject("graspable_cylinder", "cylinder", cylinderdimension, cylinderpose));
    // std::string init_cylinder_obj_xml = fallbackWrapperXML("init_cylinder_obj", {check_cylinder_obj_xml, add_cylinder_obj_xml});

    // Create object actions xml snippets (the object are created directly in the create*() functions relative to each type of object action)
    std::string check_graspable_mesh_obj_xml = buildObjectActionXML("check_" + object_to_manipulate_1, createCheckObjectExists(object_to_manipulate_1));
    std::string check_renamed_mesh_obj_xml = buildObjectActionXML("check_" + object_to_manipulate_2, createCheckObjectExists(object_to_manipulate_2));
    std::string check_machine_mesh_obj_xml = buildObjectActionXML("check_machine_mesh", createCheckObjectExists("machine_mesh"));
    std::string check_endplate_mesh_obj_xml = buildObjectActionXML("check_endplate_mesh", createCheckObjectExists("endplate_mesh"));
    std::string check_slider_mesh_obj_xml = buildObjectActionXML("check_slider_mesh", createCheckObjectExists("slider_mesh"));

    std::string add_graspable_mesh_obj_xml = buildObjectActionXML(
        "add_graspable_mesh",
        createAddMeshObject(
            object_to_manipulate_1,
            graspable_mesh_pose,
            graspable_mesh_file,
            graspable_mesh_scale[0], graspable_mesh_scale[1], graspable_mesh_scale[2]));

    // When we switch from a robot to another we remove the object from the scene to free up the name to add it again concurrently
    // Then we add it again in the same position as the removed object but with another name, so it will be used by the other
    // robot to plan while checking for collisions
    std::string add_renamed_mesh_obj_xml = buildObjectActionXML(
        "add_renamed_mesh",
        createAddMeshObject(
            object_to_manipulate_2,
            dropped_target_key_name, // We use the overload with the blakboard key to retrive it dynamically
            graspable_mesh_file,
            graspable_mesh_scale[0], graspable_mesh_scale[1], graspable_mesh_scale[2]));

    // Here we add the objects that create the scene
    std::string add_machine_mesh_obj_xml = buildObjectActionXML(
        "add_machine_mesh",
        createAddMeshObject(
            "machine_mesh",
            createPoseRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            machine_mesh_file,
            1.0, 1.0, 1.0));

    // We save the name of the endplate mesh since we'll use it to get the pose for the second robot to load the object in the machine
    std::string endplate_name = "endplate_mesh";
    Pose endplate_pose = createPoseRPY(0.571, -0.6235, 0.725, 1.57, 3.14, 0.0);
    Pose endplate_approach_pose = endplate_pose;
    endplate_approach_pose.position.y += 0.05;

    // We build the same variables we build for the graspable objects also for the endplate on the machine
    std::string load_target_2_key_name = "load_target_2";
    std::string approach_load_target_2_key_name = "approach_load_target_2";
    blackboard->set(load_target_2_key_name, endplate_pose);
    blackboard->set(approach_load_target_2_key_name, endplate_approach_pose);

    std::string add_endplate_mesh_obj_xml = buildObjectActionXML(
        "add_endplate_mesh",
        createAddMeshObject(
            endplate_name,
            load_target_2_key_name,
            endplate_mesh_file,
            1.0, 1.0, 1.0));

    std::string add_slider_mesh_obj_xml = buildObjectActionXML(
        "add_slider_mesh",
        createAddMeshObject(
            "slider_mesh",
            createPoseRPY(((tube_length) + 0.01), 0.0, 0.0, 0.0, 0.0, 0.0),
            slider_mesh_file,
            1.0, 1.0, 1.0));

    // Compose the check and add sequence for objects
    std::string init_graspable_mesh_obj_xml = fallbackWrapperXML("init_graspable_mesh_obj", {check_graspable_mesh_obj_xml, add_graspable_mesh_obj_xml});
    std::string init_machine_mesh_obj_xml = fallbackWrapperXML("init_machine_mesh_obj", {check_machine_mesh_obj_xml, add_machine_mesh_obj_xml});
    std::string init_endplate_mesh_obj_xml = fallbackWrapperXML("init_endplate_mesh_obj", {check_endplate_mesh_obj_xml, add_endplate_mesh_obj_xml});
    std::string init_slider_mesh_obj_xml = fallbackWrapperXML("init_slider_mesh_obj", {check_slider_mesh_obj_xml, add_slider_mesh_obj_xml});

    // the name of the link to attach the object to, and the object to manipulate
    std::string tcp_frame_name_1 = robot_prefix_1 + tcp_frame_1;
    std::string tcp_frame_name_2 = robot_prefix_2 + tcp_frame_2;

    std::string attach_obj_1_xml = buildObjectActionXML("attach_obj_to_manipulate_1", createAttachObject(object_to_manipulate_1, tcp_frame_name_1));
    std::string detach_obj_1_xml = buildObjectActionXML("attach_obj_to_manipulate_1", createDetachObject(object_to_manipulate_1, tcp_frame_name_1));
    std::string remove_obj_1_xml = buildObjectActionXML("remove_obj_to_manipulate_1", createRemoveObject(object_to_manipulate_1));

    std::string attach_obj_2_xml = buildObjectActionXML("attach_obj_to_manipulate_2", createAttachObject(object_to_manipulate_2, tcp_frame_name_2));
    std::string detach_obj_2_xml = buildObjectActionXML("attach_obj_to_manipulate_2", createDetachObject(object_to_manipulate_2, tcp_frame_name_2));
    std::string remove_obj_2_xml = buildObjectActionXML("remove_obj_to_manipulate_2", createRemoveObject(object_to_manipulate_2));

    //

    // ----------------------------------------------------------------------------
    // 3) Add GetObjectPoseAction Node and nodes to attach/detach objects
    // ----------------------------------------------------------------------------

    //

    // Define the transformation and reference orientation
    std::vector<double> pick_pre_transform_xyz_rpy_1 = {-0.002, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> approach_pick_pre_transform_xyz_rpy_1 = {-0.07, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> pick_post_transform_xyz_rpy_1 = {0.0, 0.0, ((-tube_length) / 2) + grasp_offset, 3.14, 0.0, 0.0};

    // Translate get_pose_action to xml tree leaf
    std::string get_pick_pose_1_xml = buildObjectActionXML(
        "get_pick_pose_1",
        createGetObjectPose(
            object_to_manipulate_1,
            pick_target_1_key_name,
            "world",
            pick_pre_transform_xyz_rpy_1,
            pick_post_transform_xyz_rpy_1));

    std::string get_approach_pose_1_xml = buildObjectActionXML(
        "get_approach_pose_1",
        createGetObjectPose(
            object_to_manipulate_1,
            approach_pick_target_1_key_name,
            "world",
            approach_pick_pre_transform_xyz_rpy_1,
            pick_post_transform_xyz_rpy_1));

    // We get the pose of the dropped object to use to rename it
    std::string get_dropped_object_pose_xml = buildObjectActionXML(
        "get_dropped_obj_pose",
        createGetObjectPose(
            object_to_manipulate_1,
            dropped_target_key_name,
            "world"));

    std::string rename_obj_1_xml = sequenceWrapperXML("rename_obj_to_manipulate_1", {get_dropped_object_pose_xml, remove_obj_1_xml, add_renamed_mesh_obj_xml, check_renamed_mesh_obj_xml});

    //

    // Now for the object to drop on the loading shaft of the second robot's gripper:

    //

    // Define the transformation and reference orientation
    std::vector<double> insert_pre_transform_xyz_rpy_2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> approach_insert_pre_transform_xyz_rpy_2 = {0.0, 0.0, -0.05, 0.0, 0.0, 0.0};
    std::vector<double> post_insert_transform_xyz_rpy_2 = {0.0, 0.0, ((-tube_length) / 2), 0.0, 0.0, -0.785};

    // Translate get_pose_action to xml tree leaf
    std::string get_insert_pose_2_xml = buildObjectActionXML(
        "get_insert_pose_2",
        createGetObjectPose(
            object_to_manipulate_1,
            insert_target_2_key_name,
            "world",
            insert_pre_transform_xyz_rpy_2,
            post_insert_transform_xyz_rpy_2));

    std::string get_approach_insert_pose_2_xml = buildObjectActionXML(
        "get_approach_insert_pose_2",
        createGetObjectPose(
            object_to_manipulate_1,
            approach_insert_target_2_key_name,
            "world",
            approach_insert_pre_transform_xyz_rpy_2,
            post_insert_transform_xyz_rpy_2));

    //

    // Now for the endplate to load the object in the machine:

    //

    // Define the transformation and reference orientation
    std::vector<double> load_pre_transform_xyz_rpy_2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> approach_load_pre_transform_xyz_rpy_2 = {0.0, 0.0, -0.025, 0.0, 0.0, 0.0};
    std::vector<double> load_post_transform_xyz_rpy_2 = {0.0, 0.0, -tube_length, 0.0, 0.0, 0.0};
    // std::vector<double> load_post_transform_xyz_rpy_2 = {0.0, 0.0, -tube_length/2, 0.0, 0.0, 0.0}; // to check for collision

    // Translate get_pose_action to xml tree leaf
    std::string get_load_pose_2_xml = buildObjectActionXML(
        "get_load_pose_2",
        createGetObjectPose(
            endplate_name,
            load_target_2_key_name,
            "world",
            load_pre_transform_xyz_rpy_2,
            load_post_transform_xyz_rpy_2));

    std::string get_approach_load_pose_2_xml = buildObjectActionXML(
        "get_approach_load_pose_2",
        createGetObjectPose(
            endplate_name,
            approach_load_target_2_key_name,
            "world",
            approach_load_pre_transform_xyz_rpy_2,
            load_post_transform_xyz_rpy_2));

    // Wrapping load pose retrieval in a sequence
    std::string get_load_poses_from_endplate_xml = sequenceWrapperXML("get_load_poses_from_endplate", {get_load_pose_2_xml, get_approach_load_pose_2_xml});

    // To help coordinating robot cycles, I create branches to wait for the existance (or absence) of an object.
    std::string wait_for_renamed_drop_obj_xml = buildWaitForObject(
        robot_prefix_2,
        "renamed_drop_exists",
        object_to_manipulate_2,
        true);

    std::string wait_for_renamed_obj_removed_xml = buildWaitForObject(
        robot_prefix_2,
        "renamed_drop_removed",
        object_to_manipulate_2,
        false);

    // Define the blackboard keys for the robots to interact with each other
    std::string robot_1_in_working_position_key_name = "robot_1_in_working_position";
    blackboard->set(robot_1_in_working_position_key_name, "false");

    std::string robot_2_in_working_position_key_name = "robot_2_in_working_position";
    blackboard->set(robot_2_in_working_position_key_name, "false");

    // Now I create branches to wait for the robots to be in position, or outside the working zone.
    std::string wait_for_robot_1_in_working_position_xml = buildWaitForKey(
        robot_prefix_1,
        "robot_1_in_working_position",
        robot_1_in_working_position_key_name,
        "true");

    std::string wait_for_robot_1_out_of_working_position_xml = buildWaitForKey(
        robot_prefix_1,
        "robot_1_out_of_working_position",
        robot_1_in_working_position_key_name,
        "false");

    std::string wait_for_robot_2_in_working_position_xml = buildWaitForKey(
        robot_prefix_2,
        "robot_2_in_working_position",
        robot_2_in_working_position_key_name,
        "true");

    std::string wait_for_robot_2_out_of_working_position_xml = buildWaitForKey(
        robot_prefix_2,
        "robot_2_out_of_working_position",
        robot_2_in_working_position_key_name,
        "false");

    // Branches to set the blackboard keys
    std::string set_robot_1_in_working_position_xml = buildSetBlackboardKey(
        robot_prefix_1,
        "robot_1_in_working_position",
        robot_1_in_working_position_key_name,
        "true");

    // Branches to set the blackboard keys
    std::string set_robot_1_out_of_working_position = buildSetBlackboardKey(
        robot_prefix_1,
        "robot_1_out_of_working_position",
        robot_1_in_working_position_key_name,
        "false");

    // Branches to set the blackboard keys
    std::string set_robot_2_in_working_position = buildSetBlackboardKey(
        robot_prefix_2,
        "robot_2_in_working_position",
        robot_2_in_working_position_key_name,
        "true");

    // Branches to set the blackboard keys
    std::string set_robot_2_out_of_working_position_xml = buildSetBlackboardKey(
        robot_prefix_2,
        "robot_2_out_of_working_position",
        robot_2_in_working_position_key_name,
        "false");

    //

    // ----------------------------------------------------------------------------
    // 4) Setup moves
    // ----------------------------------------------------------------------------

    //

    auto move_configs = defineMovementConfigs();

    // We define the joint targets we need for the joint moves as vectors of doubles.
    // Be careful that the number of values must match the number of DOF of the robot (here, 6 DOF)
    std::vector<double> joint_rest_1 = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};

    std::vector<double> joint_rest_2 = {0.0, 0.785, -0.785, 0.0, -1.57, 0.0};
    std::vector<double> joint_ready_2 = {0.3, -0.372, -1.582, 1.67, 1.845, 0.375};

    std::string named_home_1 = "home";
    std::string named_home_2 = "home";

    // Compose the sequences of moves. Each of the following sequences represent a logic
    // ROBOT 1
    std::vector<Move> rest_position_1 = {
        {robot_prefix_1, "joint", "", joint_rest_1, "", move_configs["max_move"]},
    };

    std::vector<Move> pick_sequence_1 = {
        {robot_prefix_1, "pose", approach_pick_target_1_key_name, {}, "", move_configs["max_move"]},
        {robot_prefix_1, "cartesian", pick_target_1_key_name, {}, "", move_configs["cartesian_slow_move"]},
    };

    std::vector<Move> wait_position_1 = {
        {robot_prefix_1, "pose", approach_pick_target_1_key_name, {}, "", move_configs["mid_move"]},
        {robot_prefix_1, "pose", approach_drop_target_1_key_name, {}, "", move_configs["max_move"]},
        // {robot_prefix_1, "joint", "", joint_rest_1, "", move_configs["max_move"]},
    };

    std::vector<Move> drop_sequence_1 = {
        // {robot_prefix_1, "pose", approach_pick_target_1_key_name, {}, "", move_configs["mid_move"]},
        // {robot_prefix_1, "cartesian", drop_target_1_key_name, {}, "", move_configs["slow_move"]},
        {robot_prefix_1, "pose", drop_target_1_key_name, {}, "", move_configs["mid_move"]},
    };

    std::vector<Move> exit_position_1 = {
        {robot_prefix_1, "pose", approach_drop_target_1_key_name, {}, "", move_configs["max_move"]},
    };

    std::vector<Move> home_position_1 = {
        {robot_prefix_1, "named", "", {}, named_home_1, move_configs["max_move"]},
    };

    // ROBOT 2
    // We keep the naming consistent with the robot_prefix associated, or else the move will fail to plan!
    std::vector<Move> rest_position_2 = {
        {robot_prefix_2, "joint", "", joint_rest_2, "", move_configs["max_move"]},
    };

    std::vector<Move> insert_sequence_2 = {
        {robot_prefix_2, "pose", approach_insert_target_2_key_name, {}, "", move_configs["max_move"]},
        {robot_prefix_2, "cartesian", insert_target_2_key_name, {}, "", move_configs["cartesian_slow_move"]},
    };

    std::vector<Move> load_sequence_2 = {
        // {robot_prefix_2, "pose", approach_pick_target_2, {}, "", move_configs["mid_move"]},
        {robot_prefix_2, "cartesian", approach_load_target_2_key_name, {}, "", move_configs["mid_move"]},
        {robot_prefix_2, "cartesian", load_target_2_key_name, {}, "", move_configs["cartesian_slow_move"]},
    };

    std::vector<Move> exit_sequence_2 = {
        {robot_prefix_2, "cartesian", approach_insert_target_2_key_name, {}, "", move_configs["max_move"]},
        // {robot_prefix_2, "joint", "", joint_ready_2, "", move_configs["max_move"]},
    };

    std::vector<Move> ready_position_2 = {
        {robot_prefix_2, "joint", "", joint_ready_2, "", move_configs["max_move"]},
    };

    std::vector<Move> home_position_2 = {
        {robot_prefix_2, "named", "", {}, named_home_2, move_configs["max_move"]},
    };

    // ROBOT 1
    // build the xml snippets for the single moves of robot 1
    // or translate them directly if they are only used once
    std::string rest_move_parallel_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "toRest", rest_position_1, blackboard, true);

    std::string pick_move_parallel_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "pick", pick_sequence_1, blackboard, true);

    std::string drop_move_parallel_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "drop", drop_sequence_1, blackboard, true);

    std::string wait_move_parallel_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "wait", wait_position_1, blackboard, true);

    std::string exit_move_parallel_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "exit", exit_position_1, blackboard, true);

    std::string home_move_parallel_1_xml = buildParallelPlanExecuteXML(
        robot_prefix_1, robot_prefix_1 + "home", home_position_1, blackboard, true);

    // We can compose sequences together into a xml tree leaf or branch
    std::string home_sequence_1_xml = sequenceWrapperXML(
        robot_prefix_1 + "ComposedHomeSequence_1", {home_move_parallel_1_xml, rest_move_parallel_1_xml});

    // ROBOT 2
    // build the xml snippets for the single moves of robot 2
    std::string rest_move_parallel_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "toRest", rest_position_2, blackboard, true);

    std::string insert_move_parallel_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "pick", insert_sequence_2, blackboard, true);

    std::string load_move_parallel_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "drop", load_sequence_2, blackboard, true);

    std::string exit_move_parallel_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "exit", exit_sequence_2, blackboard, true);

    std::string ready_move_parallel_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "toReady", ready_position_2, blackboard, true);

    std::string home_move_parallel_2_xml = buildParallelPlanExecuteXML(
        robot_prefix_2, robot_prefix_2 + "home", home_position_2, blackboard, true);

    // We can compose sequences together into a xml tree leaf or branch
    std::string home_sequence_2_xml = sequenceWrapperXML(
        robot_prefix_2 + "ComposedHomeSequence_2", {home_move_parallel_2_xml, rest_move_parallel_2_xml});

    //

    // ----------------------------------------------------------------------------
    // 5) Define Signals calls:
    // ----------------------------------------------------------------------------

    //

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

    //

    // ----------------------------------------------------------------------------
    // 6) Combine the objects and moves in a sequences that can run a number of times:
    // ----------------------------------------------------------------------------

    //

    // Let's build the full sequence in logically separated blocks:
    // General
    std::string spawn_fixed_objects_xml = sequenceWrapperXML("SpawnFixedObjects", {init_machine_mesh_obj_xml, init_endplate_mesh_obj_xml, init_slider_mesh_obj_xml}); //, init_cylinder_obj_xml});

    // Robot 1
    std::string go_to_rest_pose_1_xml = sequenceWrapperXML("GoToRestPose", {rest_move_parallel_1_xml});
    std::string get_grasp_object_poses_1_xml = sequenceWrapperXML("GetGraspPoses", {get_pick_pose_1_xml, get_approach_pose_1_xml});
    std::string go_to_pick_pose_1_xml = sequenceWrapperXML("GoToPickPose", {pick_move_parallel_1_xml});
    std::string close_gripper_1_xml = sequenceWrapperXML("CloseGripper", {signal_gripper_close_1_xml, check_gripper_close_1_xml, attach_obj_1_xml});
    std::string go_to_wait_pose_1_xml = sequenceWrapperXML("GoToWaitPose", {wait_move_parallel_1_xml});
    std::string go_to_drop_pose_1_xml = sequenceWrapperXML("GoToDropPose", {drop_move_parallel_1_xml});
    std::string open_gripper_1_xml = sequenceWrapperXML("OpenGripper", {signal_gripper_open_1_xml, detach_obj_1_xml});
    std::string go_to_exit_pose_1_xml = sequenceWrapperXML("GoToExitPose", {exit_move_parallel_1_xml});

    // Robot 2
    std::string go_to_rest_pose_2_xml = sequenceWrapperXML("GoToRestPose", {rest_move_parallel_2_xml});
    std::string get_grasp_object_poses_2_xml = sequenceWrapperXML("GetGraspPoses", {get_insert_pose_2_xml, get_approach_insert_pose_2_xml});
    std::string go_to_ready_pose_2_xml = sequenceWrapperXML("GoToReadyPose", {ready_move_parallel_2_xml});
    std::string go_to_insert_pose_2_xml = sequenceWrapperXML("GoToPickPose", {insert_move_parallel_2_xml});
    std::string go_to_load_pose_2_xml = sequenceWrapperXML("GoToLoadPose", {load_move_parallel_2_xml});
    std::string go_to_exit_pose_2_xml = sequenceWrapperXML("GoToExitPose", {exit_move_parallel_2_xml});

    // Dedicated spawnable objects per robot:
    std::string spawn_graspable_objects_1_xml = sequenceWrapperXML("SpawnGraspableObjects", {init_graspable_mesh_obj_xml});

    // Parallel startup subsequences:
    std::string startup_sequence_1_xml = sequenceWrapperXML("StartUpSequence_1", {check_reset_robot_1_xml, rest_move_parallel_1_xml});
    std::string startup_sequence_2_xml = sequenceWrapperXML("StartUpSequence_2", {check_reset_robot_2_xml, ready_move_parallel_2_xml});
    std::string parallel_sub_startup_sequences_xml = parallelWrapperXML("Parallel_startupSequences", {startup_sequence_1_xml, startup_sequence_2_xml}, 2, 1);

    // General startup sequence:
    std::string startup_sequence_xml = sequenceWrapperXML("StartUpSequence", {spawn_fixed_objects_xml, parallel_sub_startup_sequences_xml, get_load_poses_from_endplate_xml});

    // ROBOT 1
    // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other children
    std::string repeat_forever_wrapper_1_xml = repeatWrapperXML(
        "RepeatForever",
        {
            set_robot_1_out_of_working_position,          //<
            spawn_graspable_objects_1_xml,                //< We add all the objects to the scene
            get_grasp_object_poses_1_xml,                 //< We get the updated poses relative to the objects
            go_to_pick_pose_1_xml,                        //< Pick move sequence
            close_gripper_1_xml,                          //< We attach the object
            go_to_wait_pose_1_xml,                        //< x
            wait_for_robot_2_out_of_working_position_xml, //<
            wait_for_renamed_obj_removed_xml,             //< x
            go_to_drop_pose_1_xml,                        //< Drop move sequence            detach_obj_1_xml,                             //<
            set_robot_1_in_working_position_xml,          //<
            wait_for_robot_2_in_working_position_xml,     //<
            open_gripper_1_xml,                           //< We detach the object
            rename_obj_1_xml,                             //< We rename the object for the other robot to use, we will add the original one back on the next cycle in the original position
            go_to_exit_pose_1_xml,                        //< Exit move sequence
        },
        -1); //< num_cycles=-1 for infinite

    // ROBOT 2
    // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other children
    std::string repeat_forever_wrapper_2_xml = repeatWrapperXML(
        "RepeatForever",
        {
            set_robot_2_out_of_working_position_xml,      //<
            wait_for_robot_1_in_working_position_xml,     //<
            get_grasp_object_poses_2_xml,                 //< We get the updated poses relative to the objects
            go_to_insert_pose_2_xml,                      //< Prep sequence and pick sequence
            set_robot_2_in_working_position,              //<
            wait_for_robot_1_out_of_working_position_xml, //<
            wait_for_renamed_drop_obj_xml,                //<
            attach_obj_2_xml,                             //< close_gripper_2_xml,          //< We attach the object
            go_to_load_pose_2_xml,                        //< Load sequence
            go_to_exit_pose_2_xml,                        //<
            detach_obj_2_xml,                             //< open_gripper_2_xml,           //< We detach the object
            remove_obj_2_xml,                             //< Homing sequence
            go_to_ready_pose_2_xml,                       //< We delete the object for it to be added on the next cycle in the original position
        },
        -1); //< num_cycles=-1 for infinite

    // Runningh both robot sequences in parallel:
    std::string parallel_repeat_forever_sequences_xml = parallelWrapperXML("PARALLEL_MOTION_SEQUENCES", {repeat_forever_wrapper_1_xml, repeat_forever_wrapper_2_xml}, 2, 1);

    // MasterSequence with startup sequence and RepeatForever as child to set BehaviorTree ID and root main_tree_to_execute in the XML
    std::vector<std::string> master_branches_xml = {startup_sequence_xml, parallel_repeat_forever_sequences_xml};
    std::string master_body = sequenceWrapperXML("GlobalMasterSequence", master_branches_xml);

    //

    // ----------------------------------------------------------------------------
    // 7) Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    // ----------------------------------------------------------------------------

    //

    std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    //

    // 8) Register node types

    //

    BT::BehaviorTreeFactory factory;
    registerAllNodeTypes(factory);

    //

    // 9) Create the tree from final_tree_xml

    //

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

    //

    // 10) ZMQ publisher (optional, to visualize in Groot)

    //

    BT::PublisherZMQ publisher(tree);

    // Create a MultiThreadedExecutor so that both nodes can be spun concurrently.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(hmi_node_1);
    executor.add_node(hmi_node_2);

    //

    // 11) Tick the tree in a loop.

    //

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
