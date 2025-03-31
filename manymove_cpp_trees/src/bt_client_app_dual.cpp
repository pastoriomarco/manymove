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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bt_client_node");
    RCLCPP_INFO(node->get_logger(), "BT Client Node started (Purely Programmatic XML).");

    // ----------------------------------------------------------------------------
    // Create params for this application
    // ----------------------------------------------------------------------------

    double tube_length;
    node->declare_parameter<double>("tube_length", 0.1);
    node->get_parameter_or<double>("tube_length", tube_length, 0.1);

    // ----------------------------------------------------------------------------
    // Create blackboard, keys and nodes
    // ----------------------------------------------------------------------------

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

    // Define all params and blackboard keys for the robot:
    RobotParams rp_1 = defineRobotParams(node, blackboard, "_1");
    RobotParams rp_2 = defineRobotParams(node, blackboard, "_2");

    // Create the HMI Service Node and pass the same blackboard ***
    auto hmi_node_1 = std::make_shared<manymove_cpp_trees::HMIServiceNode>(rp_1.prefix + "hmi_service_node", blackboard, rp_1.prefix);
    auto hmi_node_2 = std::make_shared<manymove_cpp_trees::HMIServiceNode>(rp_2.prefix + "hmi_service_node", blackboard, rp_2.prefix);
    RCLCPP_INFO(node->get_logger(), "HMI Service Nodes instantiated.");

    //

    // ----------------------------------------------------------------------------
    // Build blocks for poses and objects handling
    // ----------------------------------------------------------------------------

    /**
     * The pose we'll use here all refer to some object: we'll create all the necessary constructs for each object.
     * The first object is the one that will be picked by the first robot. Here we assume it is placed in a known position by a mechanical
     * distributor. We will need to have the pose recognized by a vision system in the future, so we already create all the constructs necessary
     * to be able to update the pose dynamically during execution.`
     */

    // We set a name for the object to be manipulated, and all the required info to create it and insert it in the scene
    std::string object_to_manipulate_1 = "graspable_mesh";
    std::string graspable_mesh_file = "package://manymove_object_manager/meshes/unit_tube.stl";

    /**
     * The tube is 1m diameter x 1m high, with the center of mass corrisponding to the origin with the axis aligned to Z axis.
     * This way, thanks to the variable for tube lenght defined previously, we can scale as needed. To make the application more versatile,
     * in the future we can also make so the dimensions depend on a blackboard key and let the user modify it from the HMI.
     */
    std::vector<double> graspable_mesh_scale = {0.01, 0.01, tube_length};

    /**
     * This is the pose for the object, aligned so the X+ axis corresponds to the exit direction from the distributor's holder.
     * Since the gripper is aligned to the Z axis, we'll have to modify it later to get the grasp pose. Note we could approach from any direction
     * perpendicular to the Z axis of the object, but defining one specific alignment lets us define a specific direction for grasping.
     */
    auto graspable_mesh_pose = createPoseRPY(((tube_length / 2) + 0.973 + 0.005), -0.6465, 0.8055, 1.57, 2.05, 1.57);

    //
    std::string check_graspable_mesh_obj_xml = buildObjectActionXML("check_" + object_to_manipulate_1, createCheckObjectExists(object_to_manipulate_1));

    // We create the xml snippet to add the mesh to the scene through a behaviortree action.
    std::string add_graspable_mesh_obj_xml = buildObjectActionXML(
        "add_graspable_mesh",
        createAddMeshObject(
            object_to_manipulate_1,
            graspable_mesh_pose,
            graspable_mesh_file,
            graspable_mesh_scale[0], graspable_mesh_scale[1], graspable_mesh_scale[2]));

    // We define a logic fallback to check if the object is already on the scene, and if not we add it.
    std::string init_graspable_mesh_obj_xml = fallbackWrapperXML("init_graspable_mesh_obj", {check_graspable_mesh_obj_xml, add_graspable_mesh_obj_xml});

    // We define the name of the link to attach the object to
    std::string tcp_frame_name_1 = rp_1.prefix + rp_1.tcp_frame;

    // Then we create the xml snippets to attach, detach the object to/from a link, and to remove it from the scene.
    std::string attach_obj_1_xml = buildObjectActionXML("attach_obj_to_manipulate_1", createAttachObject(object_to_manipulate_1, tcp_frame_name_1));
    std::string detach_obj_1_xml = buildObjectActionXML("attach_obj_to_manipulate_1", createDetachObject(object_to_manipulate_1, tcp_frame_name_1));
    std::string remove_obj_1_xml = buildObjectActionXML("remove_obj_to_manipulate_1", createRemoveObject(object_to_manipulate_1));

    // Now that we have all we need for the first object, we can create the first poses relative to it.

    /**
     * For each pose to get/set dynamically, rember to create a unique blackboard key accordingly.
     * Be careful not to use names that may conflict with the keys automatically
     * created for the moves. (Usually move_{move_id})
     * We also create a string to store the key name, or we'll risk to misuse it later if we change it here.
     * The original pick pose will be overwritten by the blackboard key that will be dynamically updated getting the grasp pose object, so we
     * set an empty Pose.
     * Note that the pose obtained from an object will refer to the object itself and will have to be modified if you want to align
     * the TCP orientation to a direction that is not the object's Z+ (more about it later).
     * We may also define a pick pose here to align the TCP as we want since we know the position of the object, but this mechanism
     * will allow to get the pose recognized by a vision system and to realign the TCP Z+ to the desired pick orientation.
     * Since this pose will be overwritten, we use an all zeros pose that would not be reachable anyway.
     */
    std::string pick_target_1_key_name = "pick_target_1";
    std::string approach_pick_target_1_key_name = "approach_pick_target_1";
    blackboard->set(pick_target_1_key_name, Pose());
    blackboard->set(approach_pick_target_1_key_name, Pose());

    /**
     * Next we define how we pick the object. When we get a pose from an object in the scene, the orientation will depend on how the object was
     * inserted in the scene and, if it's a mesh, from the object's orientation in the original file. The gripper's approach direction is oriented
     * on its Z axis, but this object should be grasped perpendicularly to the object's Z axis. thus we need to modify the orientation to align it.
     * Given the original mesh file and the shape of the gripper's jaws, we want to move the reference pick point -2mm in the x direction, and rotate
     * it 90 degrees on Y axis, thus obtaining a pick orientation that aligns with the object's X axis.
     * The approach direction will just have to be some cm further, depending on the applcation: here, -7cm is about ok.
     */
    // Define the transformation and reference orientation
    std::vector<double> pick_pre_transform_xyz_rpy_1 = {-0.002, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> approach_pick_pre_transform_xyz_rpy_1 = {-0.07, 0.0, 0.0, 0.0, 1.57, 0.0};

    // We want the pick pose to be at a fixed distance from the end of the tube regardless of the length so we set a grasp offset:
    double grasp_offset = 0.025;
    std::vector<double> pick_post_transform_xyz_rpy_1 = {0.0, 0.0, ((-tube_length) / 2) + grasp_offset, 3.14, 0.0, 0.0};

    /**
     * Now we use all the variables we created until now to define the xml snippets that will get the pick and approach poses of the object dynamically,
     * modified as we set in the transforms and relative to the "world" frame.
     */
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

    // ----------------------------------------------------------------------------
    /**
     * With this the completed all the required snippets to handle the fist object and the first pose.
     * We'll proceed here with all the other objects and poses, commenting only where something changes from what we saw until now.
     */
    // ----------------------------------------------------------------------------

    /**
     * Drop pose: this is not overwritten later, as it doesn't need to change dynamically in this application.
     * Note that this pose directly refer to the pose that the TCP will allign to and it's referred to the world frame.
     * For example, this pose has the Z+ facing downard, 45 degrees in the XZ plane, in between X- and Z-.
     */
    Pose drop_target_1 = createPoseRPY(0.57, -0.25, 0.72, -0.785, -3.14, 1.57);
    Pose approach_drop_target_1 = drop_target_1;
    // We need to modify the exit pose accordingly: we move in X+ and Z+ to obtain a 45 degree exit in the XZ plane, in the opposite
    // direction of the Z+ of the TCP in the drop_target_1 pose.
    approach_drop_target_1.position.x += 0.075;
    approach_drop_target_1.position.z += 0.075;

    // We still create the blackboard key, even if we don't modify the pose later, since the pose is always passed through blackboard keys for consistency
    std::string drop_target_1_key_name = "drop_target_1";
    std::string approach_drop_target_1_key_name = "approach_drop_target_1";
    blackboard->set(drop_target_1_key_name, drop_target_1);
    blackboard->set(approach_drop_target_1_key_name, approach_drop_target_1);

    //

    // We name the second object to manipulate, which is the first object renamed
    // After we drop the obect on the other robot we want to respawn the original one, but we rename it. We start by defining the new name:
    std::string object_to_manipulate_2 = "renamed_mesh";

    // Create object actions xml snippets (the object are created directly in the create*() functions relative to each type of object action)
    std::string check_renamed_mesh_obj_xml = buildObjectActionXML("check_" + object_to_manipulate_2, createCheckObjectExists(object_to_manipulate_2));

    /**
     * Once the object will be moved, the drop position will have to be retrieved with the new pose of the object, so I need a new blackboard
     * key for the new pose. Note the
     */
    std::string dropped_target_key_name = "dropped_target";
    // Then I set a temporary value, it will be overwritten later:
    blackboard->set(dropped_target_key_name, drop_target_1);

    /**
     * When we switch from a robot to another we remove the object from the scene to free up the name to add it again concurrently
     * Then we add it again in the same position as the removed object but with another name, so it will be used by the other
     * robot to plan while checking for collisions
     */
    std::string add_renamed_mesh_obj_xml = buildObjectActionXML(
        "add_renamed_mesh",
        createAddMeshObject(
            object_to_manipulate_2,
            dropped_target_key_name, // We use the overload with the blakboard key to retrive it dynamically
            graspable_mesh_file,
            graspable_mesh_scale[0], graspable_mesh_scale[1], graspable_mesh_scale[2]));

    /**
     * We get the pose of the dropped object to use it to insert the renamed object
     * Note that this pose is only used to reinsert the object in the scene: to get the insert pose for the second object we will get
     * the pose of the newly created object and transform that for the TCP pose.
     */
    std::string get_dropped_object_pose_xml = buildObjectActionXML(
        "get_dropped_obj_pose",
        createGetObjectPose(
            object_to_manipulate_1,
            dropped_target_key_name,
            "world"));

    // Here we put together all the snippets to rename the object
    std::string rename_obj_1_xml = sequenceWrapperXML("rename_obj_to_manipulate_1", {get_dropped_object_pose_xml, remove_obj_1_xml, add_renamed_mesh_obj_xml, check_renamed_mesh_obj_xml});

    //

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

    // We define the other elements on the scene. We only
    std::string machine_mesh_file = "package://manymove_object_manager/meshes/custom_scene/machine.stl";
    std::string slider_mesh_file = "package://manymove_object_manager/meshes/custom_scene/slider.stl";
    std::string endplate_mesh_file = "package://manymove_object_manager/meshes/custom_scene/end_plate.stl";

    std::string check_machine_mesh_obj_xml = buildObjectActionXML("check_machine_mesh", createCheckObjectExists("machine_mesh"));
    std::string check_slider_mesh_obj_xml = buildObjectActionXML("check_slider_mesh", createCheckObjectExists("slider_mesh"));
    std::string check_endplate_mesh_obj_xml = buildObjectActionXML("check_endplate_mesh", createCheckObjectExists("endplate_mesh"));

    // Here we add the objects that create the scene. The machine is in a fixed position
    std::string add_machine_mesh_obj_xml = buildObjectActionXML(
        "add_machine_mesh",
        createAddMeshObject(
            "machine_mesh",
            createPoseRPY(0.0, 0.0, -0.001, 0.0, 0.0, 0.0),
            machine_mesh_file,
            1.0, 1.0, 1.0));

    // The slider is in a variable position. We still use the tube_length variable and not a dedicated blackboard key, this may change
    // if we want to set the length from HMI
    std::string add_slider_mesh_obj_xml = buildObjectActionXML(
        "add_slider_mesh",
        createAddMeshObject(
            "slider_mesh",
            createPoseRPY(((tube_length) + 0.01), 0.0, 0.0, 0.0, 0.0, 0.0),
            slider_mesh_file,
            1.0, 1.0, 1.0));

    // Save the name of the endplate mesh since we'll use it to get the pose for the second robot to load the object in the machine
    std::string endplate_name = "endplate_mesh";
    Pose endplate_pose = createPoseRPY(0.571, -0.6235, 0.725, 1.57, 3.14, 0.0);
    Pose endplate_approach_pose = endplate_pose;
    endplate_approach_pose.position.y += 0.05;

    // Build the same variables we build for the graspable objects also for the endplate on the machine
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

    // Compose the check and add sequence for objects
    std::string init_machine_mesh_obj_xml = fallbackWrapperXML("init_machine_mesh_obj", {check_machine_mesh_obj_xml, add_machine_mesh_obj_xml});
    std::string init_endplate_mesh_obj_xml = fallbackWrapperXML("init_endplate_mesh_obj", {check_endplate_mesh_obj_xml, add_endplate_mesh_obj_xml});
    std::string init_slider_mesh_obj_xml = fallbackWrapperXML("init_slider_mesh_obj", {check_slider_mesh_obj_xml, add_slider_mesh_obj_xml});

    // Set the name for the TCP of ROBOT 2
    std::string tcp_frame_name_2 = rp_2.prefix + rp_2.tcp_frame;

    // Create the xml snippets to attach/detach the second object to manipulate to/from a link, and to remove it from the scene
    std::string attach_obj_2_xml = buildObjectActionXML("attach_obj_to_manipulate_2", createAttachObject(object_to_manipulate_2, tcp_frame_name_2));
    std::string detach_obj_2_xml = buildObjectActionXML("attach_obj_to_manipulate_2", createDetachObject(object_to_manipulate_2, tcp_frame_name_2));
    std::string remove_obj_2_xml = buildObjectActionXML("remove_obj_to_manipulate_2", createRemoveObject(object_to_manipulate_2));

    //

    // Now for the object to drop on the loading shaft of the second robot's gripper:

    // Define the transformation and reference orientation
    std::vector<double> insert_pre_transform_xyz_rpy_2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> approach_insert_pre_transform_xyz_rpy_2 = {0.0, 0.0, -0.05, 0.0, 0.0, 0.0};
    std::vector<double> post_insert_transform_xyz_rpy_2 = {0.0, 0.0, ((-tube_length) / 2), 0.0, 0.0, -0.785};

    /**
     * A little note here: we can use both object_to_manipulate_1 or object_to_manipulate_2 to get the object pose, the difference is when you want
     * to read the pose. Below you'll see where the get_grasp_object_poses_2_xml is placed, which is BEFORE the object is released from the gripper
     * of the first robot, thus the renamed object doesn't exist yet.
     */
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

    // Define the transformation and reference orientation
    std::vector<double> load_pre_transform_xyz_rpy_2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> approach_load_pre_transform_xyz_rpy_2 = {0.0, 0.0, -0.025, 0.0, 0.0, 0.0};
    std::vector<double> load_post_transform_xyz_rpy_2 = {0.0, 0.0, -tube_length, 0.0, 0.0, 0.0};
    // std::vector<double> load_post_transform_xyz_rpy_2 = {0.0, 0.0, -tube_length/3, 0.0, 0.0, 0.0}; // to check for collision

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
        rp_2.prefix,
        "renamed_drop_exists",
        object_to_manipulate_2,
        true);

    std::string wait_for_renamed_obj_removed_xml = buildWaitForObject(
        rp_2.prefix,
        "renamed_drop_removed",
        object_to_manipulate_2,
        false);

    //

    // ----------------------------------------------------------------------------
    // Setup wait conditions
    // ----------------------------------------------------------------------------

    // Define the blackboard keys for the robots to interact with each other
    std::string robot_1_in_working_position_key_name = "robot_1_in_working_position";
    blackboard->set(robot_1_in_working_position_key_name, "false");

    std::string robot_2_in_working_position_key_name = "robot_2_in_working_position";
    blackboard->set(robot_2_in_working_position_key_name, "false");

    // Now I create branches to wait for the robots to be in position, or outside the working zone.
    std::string wait_for_robot_1_in_working_position_xml = buildWaitForKey(
        rp_1.prefix,
        "robot_1_in_working_position",
        robot_1_in_working_position_key_name,
        "true");

    std::string wait_for_robot_1_out_of_working_position_xml = buildWaitForKey(
        rp_1.prefix,
        "robot_1_out_of_working_position",
        robot_1_in_working_position_key_name,
        "false");

    std::string wait_for_robot_2_in_working_position_xml = buildWaitForKey(
        rp_2.prefix,
        "robot_2_in_working_position",
        robot_2_in_working_position_key_name,
        "true");

    std::string wait_for_robot_2_out_of_working_position_xml = buildWaitForKey(
        rp_2.prefix,
        "robot_2_out_of_working_position",
        robot_2_in_working_position_key_name,
        "false");

    // Branches to set the blackboard keys
    std::string set_robot_1_in_working_position_xml = buildSetBlackboardKey(
        rp_1.prefix,
        "robot_1_in_working_position",
        robot_1_in_working_position_key_name,
        "true");

    // Branches to set the blackboard keys
    std::string set_robot_1_out_of_working_position = buildSetBlackboardKey(
        rp_1.prefix,
        "robot_1_out_of_working_position",
        robot_1_in_working_position_key_name,
        "false");

    // Branches to set the blackboard keys
    std::string set_robot_2_in_working_position = buildSetBlackboardKey(
        rp_2.prefix,
        "robot_2_in_working_position",
        robot_2_in_working_position_key_name,
        "true");

    // Branches to set the blackboard keys
    std::string set_robot_2_out_of_working_position_xml = buildSetBlackboardKey(
        rp_2.prefix,
        "robot_2_out_of_working_position",
        robot_2_in_working_position_key_name,
        "false");

    //

    // ----------------------------------------------------------------------------
    // Setup moves
    // ----------------------------------------------------------------------------

    auto move_configs = defineMovementConfigs();

    // We define the joint targets we need for the joint moves as vectors of doubles.
    // Be careful that the number of values must match the number of DOF of the robot (here, 6 DOF)
    std::vector<double> joint_rest_1 = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};

    std::vector<double> joint_rest_2 = {0.0, 0.785, -0.785, 0.0, -1.57, 0.0};
    std::vector<double> joint_ready_2 = {0.314, -0.3837, -1.5874, 1.6747, 1.8665, 0.3663};

    std::string named_home_1 = "home";
    std::string named_home_2 = "home";

    // Compose the sequences of moves. Each of the following sequences represent a logic
    // ROBOT 1
    // build the xml snippets for the single moves of robot 1
    // or translate them directly if they are only used once
    std::string rest_move_parallel_1_xml = buildMoveXML(
        rp_1.prefix, rp_1.prefix + "toRest",
        {{rp_1.prefix, "joint", move_configs["max_move"], "", joint_rest_1}},
        blackboard);

    std::string pick_move_parallel_1_xml = buildMoveXML(
        rp_1.prefix, rp_1.prefix + "pick",
        {{rp_1.prefix, "pose", move_configs["max_move"], approach_pick_target_1_key_name},
         {rp_1.prefix, "cartesian", move_configs["cartesian_slow_move"], pick_target_1_key_name}},
        blackboard);

    std::string drop_move_parallel_1_xml = buildMoveXML(
        rp_1.prefix, rp_1.prefix + "drop",
        {{rp_1.prefix, "cartesian", move_configs["cartesian_max_move"], drop_target_1_key_name}},
        blackboard);

    std::string wait_move_parallel_1_xml = buildMoveXML(
        rp_1.prefix, rp_1.prefix + "wait",
        {{rp_1.prefix, "cartesian", move_configs["cartesian_mid_move"], approach_pick_target_1_key_name},
         {rp_1.prefix, "pose", move_configs["max_move"], approach_drop_target_1_key_name}},
        blackboard);

    std::string exit_move_parallel_1_xml = buildMoveXML(
        rp_1.prefix, rp_1.prefix + "exit",
        {{rp_1.prefix, "pose", move_configs["max_move"], approach_drop_target_1_key_name}},
        blackboard);

    std::string home_move_parallel_1_xml = buildMoveXML(
        rp_1.prefix, rp_1.prefix + "home",
        {{rp_1.prefix, "named", move_configs["max_move"], "", {}, named_home_1}},
        blackboard);

    // We can compose sequences together into a xml tree leaf or branch
    std::string home_sequence_1_xml = sequenceWrapperXML(
        rp_1.prefix + "ComposedHomeSequence_1", {home_move_parallel_1_xml, rest_move_parallel_1_xml});

    // ROBOT 2
    // build the xml snippets for the single moves of robot 2
    std::string rest_move_parallel_2_xml = buildMoveXML(
        rp_2.prefix, rp_2.prefix + "toRest",
        {{rp_2.prefix, "joint", move_configs["max_move"], "", joint_rest_2}},
        blackboard);

    std::string insert_move_parallel_2_xml = buildMoveXML(
        rp_2.prefix, rp_2.prefix + "pick",
        {{rp_2.prefix, "pose", move_configs["max_move"], approach_insert_target_2_key_name},
         {rp_2.prefix, "cartesian", move_configs["cartesian_slow_move"], insert_target_2_key_name}},
        blackboard);

    std::string load_move_parallel_2_xml = buildMoveXML(
        rp_2.prefix, rp_2.prefix + "drop",
        {{rp_2.prefix, "cartesian", move_configs["cartesian_mid_move"], approach_load_target_2_key_name},
         {rp_2.prefix, "cartesian", move_configs["cartesian_slow_move"], load_target_2_key_name}},
        blackboard);

    std::string exit_move_parallel_2_xml = buildMoveXML(
        rp_2.prefix, rp_2.prefix + "exit",
        {{rp_2.prefix, "cartesian", move_configs["cartesian_max_move"], approach_insert_target_2_key_name}},
        blackboard);

    std::string ready_move_parallel_2_xml = buildMoveXML(
        rp_2.prefix, rp_2.prefix + "toReady",
        {{rp_2.prefix, "joint", move_configs["max_move"], "", joint_ready_2}},
        blackboard);

    std::string home_move_parallel_2_xml = buildMoveXML(
        rp_2.prefix, rp_2.prefix + "home",
        {{rp_2.prefix, "named", move_configs["max_move"], "", {}, named_home_2}},
        blackboard);

    // We can compose sequences together into a xml tree leaf or branch
    std::string home_sequence_2_xml = sequenceWrapperXML(
        rp_2.prefix + "ComposedHomeSequence_2", {home_move_parallel_2_xml, rest_move_parallel_2_xml});

    //

    // ----------------------------------------------------------------------------
    // Define Signals calls:
    // ----------------------------------------------------------------------------

    // Let's send and receive signals only if the robot is real, and let's fake a delay on inputs otherwise
    // Robot 1
    std::string signal_gripper_close_1_xml = (rp_1.is_real ? buildSetOutputXML(rp_1.prefix, "GripperClose", "controller", 0, 1) : "");
    std::string signal_gripper_open_1_xml = (rp_1.is_real ? buildSetOutputXML(rp_1.prefix, "GripperOpen", "controller", 0, 0) : "");
    std::string check_gripper_close_1_xml = (rp_1.is_real ? buildWaitForInput(rp_1.prefix, "WaitForSensor", "controller", 0, 1) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");
    std::string check_gripper_open_1_xml = (rp_1.is_real ? buildWaitForInput(rp_1.prefix, "WaitForSensor", "controller", 0, 0) : "<Delay delay_msec=\"250\">\n  <AlwaysSuccess />\n</Delay>\n");
    std::string check_robot_state_1_xml = buildCheckRobotStateXML(rp_1.prefix, "CheckRobot", "robot_ready", "error_code", "robot_mode", "robot_state", "robot_msg");
    std::string reset_robot_state_1_xml = buildResetRobotStateXML(rp_1.prefix, "ResetRobot", rp_1.model);

    std::string check_reset_robot_1_xml = (rp_1.is_real ? fallbackWrapperXML(rp_1.prefix + "CheckResetFallback", {check_robot_state_1_xml, reset_robot_state_1_xml}) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

    // Robot 2
    std::string check_robot_state_2_xml = buildCheckRobotStateXML(rp_2.prefix, "CheckRobot", "robot_ready", "error_code", "robot_mode", "robot_state", "robot_msg");
    std::string reset_robot_state_2_xml = buildResetRobotStateXML(rp_2.prefix, "ResetRobot", rp_2.model);

    std::string check_reset_robot_2_xml = (rp_2.is_real ? fallbackWrapperXML(rp_2.prefix + "CheckResetFallback", {check_robot_state_2_xml, reset_robot_state_2_xml}) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

    //

    // ----------------------------------------------------------------------------
    // Combine the objects and moves in a sequences that can run a number of times:
    // ----------------------------------------------------------------------------

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
    std::string open_gripper_1_xml = sequenceWrapperXML("OpenGripper", {signal_gripper_open_1_xml, detach_obj_1_xml, check_gripper_open_1_xml});
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
            go_to_wait_pose_1_xml,                        //<
            wait_for_robot_2_out_of_working_position_xml, //<
            wait_for_renamed_obj_removed_xml,             //<
            go_to_drop_pose_1_xml,                        //< Drop move sequence
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
            attach_obj_2_xml,                             //< We attach the object
            go_to_load_pose_2_xml,                        //< Load sequence
            go_to_exit_pose_2_xml,                        //<
            detach_obj_2_xml,                             //< We detach the object
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
    // Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    // ----------------------------------------------------------------------------

    std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    //

    // Register node types
    BT::BehaviorTreeFactory factory;
    registerAllNodeTypes(factory);

    //

    // Create the tree from final_tree_xml
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

    // ZMQ publisher (optional, to visualize in Groot)
    BT::PublisherZMQ publisher(tree);

    // Create a MultiThreadedExecutor so that both nodes can be spun concurrently.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(hmi_node_1);
    executor.add_node(hmi_node_2);

    //

    // Tick the tree in a loop.
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
