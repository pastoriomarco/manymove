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
    // 1) Create a blackboard and set "node"
    // ----------------------------------------------------------------------------
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

    std::vector<manymove_cpp_trees::BlackboardEntry> keys;

    // Define all params and blackboard keys for the robot:
    RobotParams rp = defineRobotParams(node, blackboard, keys);

    // ----------------------------------------------------------------------------
    // 2) Setup joint targets, poses and moves
    // ----------------------------------------------------------------------------

    /*
     * defineMovementConfigs() creates the default configs to use to plan for moves as
     * defined in the helper function definition.
     * Here we created some default configs like "max_move", "mid_move" and "slow_move",
     * each with its own limits in scaling factors, max cartesian speed, step size and
     * so on, and we use them on any kind of move. We may also create specific configurations
     * for certain moves, for example with a finer step size on a short cartesian move, or
     * a lower plan number target for faster planning times in moves that are not time sensitive
     * during execution, but require planning times as short as possible in a dynamic environment.
     */
    auto move_configs = defineMovementConfigs();

    // We define the joint targets we need for the joint moves as vectors of doubles.
    // Be careful that the number of values must match the number of DOF of the robot (here, 6 DOF)
    std::vector<double> joint_rest = {0.0, 0.785, -0.785, 0.0, -1.57, 0.0};
    std::string named_home = "home";

    // Populate the blackboard with the poses, one unique key for each pose we want to use.
    // Be careful not to use names that may conflict with the keys automatically created for the moves. (Usually move_{move_id})

    // The pick target is to be obtained from the object later, so we put an empty pose for now.
    blackboard->set("pick_target_key", Pose());
    blackboard->set("approach_pick_target_key", Pose());

    // Drop poses to place the object, these are not overwritten later, so we hardcode them
    // Here we create the drop pose first, then we set it in the blackboard key
    // Notice that, since the object will be created with Z+ pointing up, we need to flip the Z axis by rotating 360 deg in X axis
    Pose drop_target = createPoseRPY(0.2, 0.0, 0.2, 3.14, 0.0, 0.785);
    blackboard->set("drop_target_key", drop_target);

    // The approach move from the drop pose is cartesian, we set an offset in the direction of the move (here, Z)
    Pose approach_drop_target = drop_target;
    approach_drop_target.position.z += 0.02;
    blackboard->set("approach_drop_target_key", approach_drop_target);

    /*
     * Then we compose the sequences of moves. Each of the following sequences represent a logic
     * sequence of moves that are somehow correlated, and not interrupted by operations on I/Os,
     * objects and logic conditions. For example the pick_sequence is a short sequence of moves composed by
     * a "pose" move to get in a position to be ready to approach the object, and the "cartesian"
     * move to get the gripper to the grasp position moving linearly to minimize chances of collisions.
     * As we'll se later, we can then compose these sequences of moves together to build bigger blocks
     * of logically corralated moves.
     * We could also keep all moves separated, but it'd be harder to obtain an easily understandable
     * tree later, expecially if we need to reuse a series of moves in a certain logic order.
     */
    std::vector<Move> rest_position = {
        {rp.prefix, "joint", move_configs["max_move"], "", joint_rest},
    };

    // Sequences for Pick/Drop/Homing
    std::vector<Move> pick_sequence = {
        {rp.prefix, "pose", move_configs["mid_move"], "approach_pick_target_key"},
        {rp.prefix, "cartesian", move_configs["cartesian_slow_move"], "pick_target_key"},
    };

    std::vector<Move> drop_sequence = {
        {rp.prefix, "cartesian", move_configs["cartesian_mid_move"], "approach_pick_target_key"},
        {rp.prefix, "pose", move_configs["max_move"], "approach_drop_target_key"},
        {rp.prefix, "cartesian", move_configs["cartesian_slow_move"], "drop_target_key"},
    };

    std::vector<Move> home_position = {
        {rp.prefix, "cartesian", move_configs["cartesian_mid_move"], "approach_drop_target_key"},
        {rp.prefix, "named", move_configs["max_move"], "", {}, named_home},
    };

    /*
     * Build move sequence blocks
     * The buildMoveXML creates the xml tree branch that manages the planning and execution of each move with the given
     * parameters. Unless the reset_trajs is set to true, each move that will plan and execute successfully will store the
     * trajectory in the blackboard, thus avoiding recalculating it the next cycle. This allow to save cycle time on all but
     * the first logic cycle, unless the scene changes. If a previously planned trajectory fails on execution it gets reset,
     * and a new one is planned. The manymove_planner checks for collisions before sending the traj, but also during the execution.
     *
     * Notice that on any string representing an XML snippet I use _xml at the end of the name to give better sense of
     * what's in that variable: if it ends with _xml, it could potentially be directly inserted as a tree branch or leaf.
     */
    std::string to_rest_reset_xml = buildMoveXML(
        rp.prefix, rp.prefix + "toRest", rest_position, blackboard, true); // this will run only on prep sequence, so we reset it afterwards

    std::string to_rest_xml = buildMoveXML(
        rp.prefix, rp.prefix + "toRest", rest_position, blackboard);

    std::string pick_object_xml = buildMoveXML(
        rp.prefix, rp.prefix + "pick", pick_sequence, blackboard);

    std::string drop_object_xml = buildMoveXML(
        rp.prefix, rp.prefix + "drop", drop_sequence, blackboard);

    std::string to_home_xml = buildMoveXML(
        rp.prefix, rp.prefix + "home", home_position, blackboard);

    /**
     * We can further combine the move sequence blocks in logic sequences.
     * This allows reusing and combining moves or sequences that are used more than once in the scene,
     * or that are used in different contexts, without risking to reuse the wrong trajectory.
     * When we call buildMoveXML() we create a new move with its own unique ID. If that move is used in more than one leaf,
     * we need to decide if we always want to try to reuse the previously successfull trajectory or not.
     * The manymove_planner logic checks for the start point of the traj to be valid and within tolerance, so we shouldn't
     * worry too much of undefined behavior, but it helps me reason on the moves sequence structure. Keeping the Move vectors at
     * minimum, using only the moves logically interconnected, makes the sequences easier to understand and debug.
     * It's also important if we want to combine sequences that retain their previously successful trajectory with sequences that don't:
     * if we need to repeat the prep sequence at some point we may be in a unkown position, so we don't want the traj to be reused.
     */

    // Translate it to xml tree leaf or branch
    std::string prep_sequence_xml = sequenceWrapperXML(
        rp.prefix + "ComposedPrepSequence", {to_rest_reset_xml});
    std::string home_sequence_xml = sequenceWrapperXML(
        rp.prefix + "ComposedHomeSequence", {to_home_xml, to_rest_xml});

    // ----------------------------------------------------------------------------
    // 3) Build blocks for objects handling
    // ----------------------------------------------------------------------------

    blackboard->set("ground_id_key", "obstacle_ground");
    blackboard->set("ground_shape_key", "box");
    blackboard->set("ground_dimension_key", std::vector<double>{0.8, 0.8, 0.1});
    blackboard->set("ground_pose_key", createPoseRPY(0.0, 0.0, -0.051, 0.0, 0.0, 0.0));
    blackboard->set("ground_scale_key", std::vector<double>{1.0, 1.0, 1.0});

    blackboard->set("wall_id_key", "obstacle_wall");
    blackboard->set("wall_shape_key", "box");
    blackboard->set("wall_dimension_key", std::vector<double>{0.8, 0.02, 0.8});
    blackboard->set("wall_pose_key", createPoseRPY(0.0, 0.4, 0.3, 0.0, 0.0, 0.0));
    blackboard->set("wall_scale_key", std::vector<double>{1.0, 1.0, 1.0});

    blackboard->set("cylinder_id_key", "graspable_cylinder");
    blackboard->set("cylinder_shape_key", "cylinder");
    blackboard->set("cylinder_dimension_key", std::vector<double>{0.1, 0.005});
    blackboard->set("cylinder_pose_key", createPoseRPY(0.1, 0.2, 0.005, 0.0, 1.57, 0.0));
    blackboard->set("cylinder_scale_key", std::vector<double>{1.0, 1.0, 1.0});

    blackboard->set("mesh_id_key", "graspable_mesh");
    blackboard->set("mesh_shape_key", "mesh");
    blackboard->set("mesh_file_key", "package://manymove_object_manager/meshes/unit_tube.stl");
    blackboard->set("mesh_scale_key", std::vector<double>{0.01, 0.01, 0.1});              //< The tube is vertical with dimension 1m x 1m x 1m. We scale it to 10x10x100 mm
    blackboard->set("mesh_pose_key", createPoseRPY(0.1, -0.2, 0.2005, 0.785, 1.57, 0.0)); //< We place it on the floor and lay it on its side, X+ facing down

    // Create object actions xml snippets (the object are created directly in the create*() functions relative to each type of object action)
    std::string check_ground_obj_xml = buildObjectActionXML("check_ground", createCheckObjectExists("ground_id_key"));
    std::string check_wall_obj_xml = buildObjectActionXML("check_wall", createCheckObjectExists("wall_id_key"));
    std::string check_cylinder_obj_xml = buildObjectActionXML("check_cylinder", createCheckObjectExists("cylinder_id_key"));
    std::string check_mesh_obj_xml = buildObjectActionXML("check_mesh", createCheckObjectExists("mesh_id_key"));

    std::string add_ground_obj_xml = buildObjectActionXML("add_ground", createAddObject("ground_id_key", "ground_shape_key", "ground_dimension_key", "ground_pose_key", "ground_scale_key", ""));
    std::string add_wall_obj_xml = buildObjectActionXML("add_wall", createAddObject("wall_id_key", "wall_shape_key", "wall_dimension_key", "wall_pose_key", "wall_scale_key", ""));
    std::string add_cylinder_obj_xml = buildObjectActionXML("add_cylinder", createAddObject("cylinder_id_key", "cylinder_shape_key", "cylinder_dimension_key", "cylinder_pose_key", "cylinder_scale_key", ""));
    std::string add_mesh_obj_xml = buildObjectActionXML("add_mesh", createAddObject("mesh_id_key", "mesh_shape_key", "", "mesh_pose_key", "mesh_scale_key", "mesh_file_key"));

    // Compose the check and add sequence for objects
    std::string init_ground_obj_xml = fallbackWrapperXML("init_ground_obj", {check_ground_obj_xml, add_ground_obj_xml});
    std::string init_wall_obj_xml = fallbackWrapperXML("init_wall_obj", {check_wall_obj_xml, add_wall_obj_xml});
    std::string init_cylinder_obj_xml = fallbackWrapperXML("init_cylinder_obj", {check_cylinder_obj_xml, add_cylinder_obj_xml});
    std::string init_mesh_obj_xml = fallbackWrapperXML("init_mesh_obj", {check_mesh_obj_xml, add_mesh_obj_xml});

    // the name of the link to attach the object to, and the object to manipulate
    blackboard->set("tcp_frame_name_key", rp.prefix + rp.tcp_frame);
    blackboard->set("object_to_manipulate_key", "graspable_mesh");

    blackboard->set("touch_links_key", rp.contact_links);

    std::string attach_obj_xml = buildObjectActionXML("attach_obj_to_manipulate", createAttachObject(
                                                                                      "object_to_manipulate_key",
                                                                                      "tcp_frame_name_key",
                                                                                      "touch_links_key"));
    std::string detach_obj_xml = fallbackWrapperXML("detach_obj_to_manipulate_always_success",
                                                    {buildObjectActionXML("detach_obj_to_manipulate", createDetachObject("object_to_manipulate_key", "tcp_frame_name_key")),
                                                     "<AlwaysSuccess />"});
    std::string remove_obj_xml = fallbackWrapperXML("remove_obj_to_manipulate_always_success",
                                                    {buildObjectActionXML("remove_obj_to_manipulate", createRemoveObject("object_to_manipulate_key")),
                                                     "<AlwaysSuccess />"});

    // ----------------------------------------------------------------------------
    // 4) Add GetObjectPoseAction Node and nodes to attach/detach objects
    // ----------------------------------------------------------------------------

    // Define the transformation and reference orientation
    /*
     * The reference orientation determines how the tranform behaves: if we leave the reference orientation to all zeroes, the
     * transform of the object will be referred to the frame we specify, here the "world" frame.
     * The object here is modelled vertically, with the simmetry axis aligned to Z.
     * We want to grasp the object aligning the Z axis of the gripper perpendicularly to the X axis of the object, thus we need
     * to rotate the Y 90 degrees, so 1.57 radians. The approach position is further away of about 5 cm in the X direction.
     * Getting this right with just one transform can be not very intuitive, so I also set up the function to enable a second transform.
     * Here, the second transform is the same for both poses, and creates a decentered grasping pose sliding in the original Z axis.
     * We then flip the grasp direction 180 degrees, or 3.14 radians, as we want the TCP's Z axis to be facing down.
     */
    blackboard->set("pick_pre_transform_xyz_rpy_1_key", std::vector<double>{0.02, 0.0, 0.0, 0.0, 1.57, 0.0});
    blackboard->set("approach_pick_pre_transform_xyz_rpy_1_key", std::vector<double>{-0.05, 0.0, 0.0, 0.0, 1.57, 0.0});
    blackboard->set("pick_post_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, -0.025, 3.14, 0.0, 0.0});

    // Utility world frame key
    blackboard->set("world_frame_key", "world");

    // Translate get_pose_action to xml tree leaf
    std::string get_pick_pose_xml = buildObjectActionXML(
        "get_pick_pose", createGetObjectPose(
                             "object_to_manipulate_key",
                             "pick_target_key",
                             "world_frame_key",
                             "pick_pre_transform_xyz_rpy_1_key",
                             "pick_post_transform_xyz_rpy_1_key"));
    std::string get_approach_pose_xml = buildObjectActionXML(
        "get_approach_pose", createGetObjectPose(
                                 "object_to_manipulate_key",
                                 "approach_pick_target_key",
                                 "world_frame_key",
                                 "approach_pick_pre_transform_xyz_rpy_1_key",
                                 "pick_post_transform_xyz_rpy_1_key"));

    // ----------------------------------------------------------------------------
    // 5) Define Signals calls:
    // ----------------------------------------------------------------------------

    // Let's send and receive signals only if the robot is real, and let's fake a delay on inputs otherwise
    std::string signal_gripper_close_xml = (rp.is_real ? buildSetOutputXML(rp.prefix, "GripperClose", "controller", 0, 1) : "");
    std::string signal_gripper_open_xml = (rp.is_real ? buildSetOutputXML(rp.prefix, "GripperOpen", "controller", 0, 0) : "");
    std::string check_gripper_close_xml = (rp.is_real ? buildWaitForInput(rp.prefix, "WaitForSensor", "controller", 0, 1) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");
    std::string check_gripper_open_xml = (rp.is_real ? buildWaitForInput(rp.prefix, "WaitForSensor", "controller", 0, 0) : "<Delay delay_msec=\"250\">\n  <AlwaysSuccess />\n</Delay>\n");
    std::string check_robot_state_xml = buildCheckRobotStateXML(rp.prefix, "CheckRobot", "robot_ready", "error_code", "robot_mode", "robot_state", "robot_msg");
    std::string reset_robot_state_xml = buildResetRobotStateXML(rp.prefix, "ResetRobot", rp.model);

    std::string check_reset_robot_xml = (rp.is_real ? fallbackWrapperXML(rp.prefix + "CheckResetFallback", {check_robot_state_xml, reset_robot_state_xml}) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

    // ----------------------------------------------------------------------------
    // 6) Combine the objects and moves in a sequences that can run a number of times:
    // ----------------------------------------------------------------------------

    // Let's build the full sequence in logically separated blocks:
    std::string spawn_fixed_objects_xml = sequenceWrapperXML("SpawnFixedObjects", {init_ground_obj_xml, init_wall_obj_xml});
    std::string spawn_graspable_objects_xml = sequenceWrapperXML("SpawnGraspableObjects", {init_cylinder_obj_xml, init_mesh_obj_xml});
    std::string get_grasp_object_poses_xml = sequenceWrapperXML("GetGraspPoses", {get_pick_pose_xml, get_approach_pose_xml});
    std::string go_to_pick_pose_xml = sequenceWrapperXML("GoToPickPose", {pick_object_xml});

    // Setting commands for gripper open/close
    std::string move_gripper_close_xml =
        "<GripperTrajAction joint_names=\"[drive_joint]\" positions=\"[0.8]\" time_from_start=\"1.0\" action_server=\"" + rp.gripper_action_server + "\"/>";
    std::string move_gripper_open_xml =
        "<GripperTrajAction joint_names=\"[drive_joint]\" positions=\"[0.4]\" time_from_start=\"1.0\" action_server=\"" + rp.gripper_action_server + "\"/>";

    std::string close_gripper_xml = sequenceWrapperXML("CloseGripper", {move_gripper_close_xml, attach_obj_xml});
    std::string open_gripper_xml = sequenceWrapperXML("OpenGripper", {move_gripper_open_xml, detach_obj_xml});

    // Set up a sequence to reset the scene:
    std::string reset_graspable_objects_xml = sequenceWrapperXML("reset_graspable_objects", {open_gripper_xml, remove_obj_xml});

    std::string startup_sequence_xml = sequenceWrapperXML(
        "StartUpSequence",
        {check_reset_robot_xml,
         spawn_fixed_objects_xml,
         reset_graspable_objects_xml,
         prep_sequence_xml});

    // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other children
    std::string repeat_forever_wrapper_xml = repeatSequenceWrapperXML(
        "RepeatForever",
        {check_reset_robot_xml,       //< We check if the robot is active, if not we try to reset it
         spawn_graspable_objects_xml, //< We add all the objects to the scene
         get_grasp_object_poses_xml,  //< We get the updated poses relative to the objects
         go_to_pick_pose_xml,         //< Prep sequence and pick sequence
         close_gripper_xml,           //< We attach the object
         drop_object_xml,             //< Drop sequence
         open_gripper_xml,            //< We detach the object
         home_sequence_xml,           //< Homing sequence
         remove_obj_xml},             //< We delete the object for it to be added on the next cycle in the original position
        -1);                          //< num_cycles=-1 for infinite

    std::string retry_forever_wrapper_xml = retrySequenceWrapperXML("RetryForever", {startup_sequence_xml, repeat_forever_wrapper_xml}, -1);

    // GlobalMasterSequence with RepeatForever as child to set BehaviorTree ID and root main_tree_to_execute in the XML
    std::vector<std::string> master_branches_xml = {retry_forever_wrapper_xml};
    std::string master_body = sequenceWrapperXML("GlobalMasterSequence", master_branches_xml);

    // ----------------------------------------------------------------------------
    // 7) Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    // ----------------------------------------------------------------------------
    std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    // 8) Register node types
    BT::BehaviorTreeFactory factory;
    registerAllNodeTypes(factory);

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

    // Create the HMI Service Node and pass the same blackboard ***
    auto hmi_node = std::make_shared<manymove_cpp_trees::HMIServiceNode>("hmi_service_node", blackboard, keys);
    RCLCPP_INFO(node->get_logger(), "HMI Service Node instantiated.");

    // Create a MultiThreadedExecutor so that both nodes can be spun concurrently.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(hmi_node);

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
