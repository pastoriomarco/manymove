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
    // Create a blackboard and set "node"
    // ----------------------------------------------------------------------------

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

    std::vector<manymove_cpp_trees::BlackboardEntry> keys;

    // Define all params and blackboard keys for the robot:
    RobotParams rp = defineRobotParams(node, blackboard, keys);
    auto move_configs = defineMovementConfigs();

    // ----------------------------------------------------------------------------
    // Build blocks for poses and objects handling
    // ----------------------------------------------------------------------------

    // UTILITY KEYS

    blackboard->set("world_frame_key", "world");
    blackboard->set("identity_transform_key", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    blackboard->set("tcp_frame_name_key", "link_tcp");

    // Objects in the scene:

    // This is the new unified helper function to create all the snippets to handle any kind of objects
    ObjectSnippets ground = createObjectSnippets(
        blackboard, keys,
        "ground",                                       /* object name */
        "box",                                          /* shape */
        createPoseRPY(0.0, 0.0, -0.051, 0.0, 0.0, 0.0), /* pose of the object */
        {1.0, 1.0, 0.1},                                /* primitive dimensions */
        "",                                             /* mesh file path */
        {1.0, 1.0, 1.0},                                /* scale */
        "",                                             /* link name to attach/detach */
        {}                                              /* contact links to attach/detach */
    );

    ObjectSnippets wall = createObjectSnippets(
        blackboard, keys, "wall", "box",
        createPoseRPY(0.0, -0.15, 0.1, 0.0, 0.0, 0.0), {1.0, 0.02, 0.2});

    ObjectSnippets graspable = createObjectSnippets(
        blackboard, keys, "graspable", "box",
        createPoseRPY(0.15, -0.25, 0.1, 0.0, 0.0, 0.0), {0.1, 0.005, 0.005},
        "", {1.0, 1.0, 1.0}, "tcp_frame_name_key");

    // ----------------------------------------------------------------------------
    // Add GetObjectPoseAction Node and nodes to attach/detach objects
    // ----------------------------------------------------------------------------

    blackboard->set("approach_pre_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, -0.05, 0.0, 0.0, 0.0});
    blackboard->set("post_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, 0.0, 3.14, 0.0, 0.0});

    // Utility world frame key
    blackboard->set("world_frame_key", "world");

    // Translate get_pose_action to xml tree leaf
    std::string get_pick_pose_xml = buildObjectActionXML(
        "get_pick_pose", createGetObjectPose(
                             "graspable_key",
                             "pick_target_key",
                             "world_frame_key",
                             "identity_transform_key",
                             "post_transform_xyz_rpy_1_key"));

    std::string get_approach_pose_xml = buildObjectActionXML(
        "get_approach_pose", createGetObjectPose(
                                 "graspable_key",
                                 "approach_pick_target_key",
                                 "world_frame_key",
                                 "approach_pre_transform_xyz_rpy_1_key",
                                 "post_transform_xyz_rpy_1_key"));

    // ----------------------------------------------------------------------------
    // Setup joint targets, poses and moves
    // ----------------------------------------------------------------------------

    // Named target, as defined in the robot's SRDF
    std::string named_home = "home";

    // We define the joint targets we need for the joint moves as vectors of doubles.
    // Be careful that the number of values must match the number of DOF of the robot (here, 6 DOF)
    std::vector<double> joint_rest = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};

    // Populate the blackboard with the poses, one unique key for each pose we want to use.
    // Be careful not to use names that may conflict with the keys automatically created for the moves. (Usually move_{move_id})

    // The pick target is to be obtained from the object later, so we put an empty pose for now.
    blackboard->set("pick_target_key", Pose());
    blackboard->set("approach_pick_target_key", Pose());

    // Drop poses to place the object, these are not overwritten later, so we hardcode them
    // Here we create the drop pose first, then we set it in the blackboard key
    Pose drop_target = createPose(0.2, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0);
    blackboard->set("drop_target_key", drop_target);

    // The approach move from the drop pose is cartesian, we set an offset in the direction of the move (here, Z)
    Pose approach_drop_target = drop_target;
    approach_drop_target.position.z += 0.02;
    blackboard->set("approach_drop_target_key", approach_drop_target);

    // Compose the TCP name:
    std::string tcp_frame_name = rp.prefix + rp.tcp_frame;

    std::vector<Move> rest_position = {
        {rp.prefix, tcp_frame_name, "joint", move_configs["max_move"], "", joint_rest},
    };

    // Sequences for Pick/Drop/Homing
    std::vector<Move> pick_sequence = {
        {rp.prefix, tcp_frame_name, "pose", move_configs["mid_move"], "approach_pick_target_key"},
        {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_slow_move"], "pick_target_key"},
    };

    std::vector<Move> drop_sequence = {
        {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_mid_move"], "approach_pick_target_key"},
        {rp.prefix, tcp_frame_name, "pose", move_configs["max_move"], "approach_drop_target_key"},
        {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_slow_move"], "drop_target_key"},
    };

    std::vector<Move> home_position = {
        {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_mid_move"], "approach_drop_target_key"},
        {rp.prefix, tcp_frame_name, "named", move_configs["max_move"], "", {}, named_home},
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

    // std::string scan_around_xml = buildMoveXML(
    //     rp.prefix, rp.prefix + "scanAround", scan_surroundings, blackboard);

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
        rp.prefix + "ComposedPrepSequence", {to_rest_reset_xml}); //, scan_around_xml});
    std::string home_sequence_xml = sequenceWrapperXML(
        rp.prefix + "ComposedHomeSequence", {to_home_xml, to_rest_xml});

    // ----------------------------------------------------------------------------
    // Combine the objects and moves in a sequences that can run a number of times:
    // ----------------------------------------------------------------------------

    // Let's build the full sequence in logically separated blocks:
    std::string spawn_fixed_objects_xml = sequenceWrapperXML("SpawnFixedObjects", {ground.init_xml, wall.init_xml});
    std::string spawn_graspable_objects_xml = sequenceWrapperXML("SpawnGraspableObjects", {graspable.init_xml});
    std::string get_grasp_object_poses_xml = sequenceWrapperXML("GetGraspPoses", {get_pick_pose_xml, get_approach_pose_xml});
    std::string go_to_pick_pose_xml = sequenceWrapperXML("GoToPickPose", {pick_object_xml});
    std::string close_gripper_xml = sequenceWrapperXML("CloseGripper", {graspable.attach_xml});
    std::string open_gripper_xml = sequenceWrapperXML("OpenGripper", {graspable.detach_xml});

    // Set up a sequence to reset the scene:
    std::string reset_graspable_objects_xml = sequenceWrapperXML("reset_graspable_objects", {open_gripper_xml, graspable.remove_xml});

    std::string startup_sequence_xml = sequenceWrapperXML(
        "StartUpSequence",
        {spawn_fixed_objects_xml,
         reset_graspable_objects_xml,
         prep_sequence_xml});

    // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other children
    std::string repeat_forever_wrapper_xml = repeatSequenceWrapperXML(
        "RepeatForever",
        {spawn_graspable_objects_xml, //< We add all the objects to the scene
         get_grasp_object_poses_xml,  //< We get the updated poses relative to the objects
         go_to_pick_pose_xml,         //< Prep sequence and pick sequence
         close_gripper_xml,           //< We attach the object
         drop_object_xml,             //< Drop sequence
         open_gripper_xml,            //< We detach the object
         home_sequence_xml,           //< Homing sequence
         graspable.remove_xml},       //< We delete the object for it to be added on the next cycle in the original position
        -1);                          //< num_cycles=-1 for infinite

    std::string retry_forever_wrapper_xml = retrySequenceWrapperXML("RetryForever", {startup_sequence_xml, repeat_forever_wrapper_xml}, -1);

    // GlobalMasterSequence with RepeatForever as child to set BehaviorTree ID and root main_tree_to_execute in the XML
    std::vector<std::string> master_branches_xml = {retry_forever_wrapper_xml};
    std::string master_body = sequenceWrapperXML("GlobalMasterSequence", master_branches_xml);

    // ----------------------------------------------------------------------------
    // Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    // ----------------------------------------------------------------------------
    std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    // Register node types
    BT::BehaviorTreeFactory factory;
    registerAllNodeTypes(factory);

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

    // Create the HMI Service Node and pass the same blackboard ***
    auto hmi_node = std::make_shared<manymove_cpp_trees::HMIServiceNode>("hmi_service_node", blackboard, keys);
    RCLCPP_INFO(node->get_logger(), "HMI Service Node instantiated.");

    // Create a MultiThreadedExecutor so that both nodes can be spun concurrently.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(hmi_node);

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
