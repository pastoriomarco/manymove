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
    // ----------------------------------------------------------------------------
    // 0. Preparing the node, blackboard and robot params
    // ----------------------------------------------------------------------------

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bt_client_node");
    RCLCPP_INFO(node->get_logger(), "BT Client Node started (Purely Programmatic XML).");

    // Create a blackboard and set "node"
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

    // Create the keys variable for HMI
    std::vector<manymove_cpp_trees::BlackboardEntry> keys;

    // Define all params and blackboard keys for the robot:
    RobotParams rp = defineRobotParams(node, blackboard, keys);
    auto move_configs = defineMovementConfigs();

    // UTILITY KEYS

    blackboard->set("world_frame_key", "world");
    blackboard->set("identity_transform_key", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    blackboard->set("tcp_frame_name_key", "link_tcp");

    // ----------------------------------------------------------------------------
    // 1. Create the scene
    // ----------------------------------------------------------------------------

    // ...
    // SECTION 1 CODE HERE
    // ...

    // ----------------------------------------------------------------------------
    // 2. Define the variable poses
    // ----------------------------------------------------------------------------

    // ...
    // SECTION 2 CODE HERE
    // ...

    // ----------------------------------------------------------------------------
    // 3. Define the moves
    // ----------------------------------------------------------------------------

    // ...
    // SECTION 3 CODE HERE
    // ...

    // ----------------------------------------------------------------------------
    // 4. Build higher level snippets
    // ----------------------------------------------------------------------------

    // ...
    // SECTION 4 CODE HERE
    // ...

    // ----------------------------------------------------------------------------
    // 5. Assembling the tree
    // ----------------------------------------------------------------------------

    // REMOVE THE NEXT LINE:
    std::string retry_forever_wrapper_xml = "";

    // ...
    // SECTION 5 CODE HERE
    // ...

    // GlobalMasterSequence with RepeatForever as child to set BehaviorTree ID and root main_tree_to_execute in the XML
    std::string master_body = sequenceWrapperXML("GlobalMasterSequence", {retry_forever_wrapper_xml});

    // Create the MasterTree
    std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

    // ----------------------------------------------------------------------------
    // 6. Setting up the overall cycle
    // ----------------------------------------------------------------------------

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
