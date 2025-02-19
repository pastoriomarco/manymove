#ifndef MANYMOVE_CPP_TREES_TREE_HELPER_HPP
#define MANYMOVE_CPP_TREES_TREE_HELPER_HPP

#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/object.hpp"
#include "manymove_msgs/msg/movement_config.hpp"

#include <behaviortree_cpp_v3/blackboard.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include "manymove_cpp_trees/action_nodes_objects.hpp"
#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include "manymove_cpp_trees/action_nodes_signals.hpp"
#include "manymove_cpp_trees/action_nodes_logic.hpp"
#include "manymove_cpp_trees/action_nodes_gripper.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace manymove_cpp_trees
{
    inline void registerAllNodeTypes(BT::BehaviorTreeFactory &factory)
    {

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

        factory.registerNodeType<CheckBlackboardKeyValue>("CheckBlackboardKeyValue");
        factory.registerNodeType<SetBlackboardKeyValue>("SetBlackboardKeyValue");
        factory.registerNodeType<BT::RetryNode>("RetryNode");
        factory.registerNodeType<RetryPauseAbortNode>("RetryPauseAbortNode");
        factory.registerNodeType<GripperCommandAction>("GripperCommandAction");
    }

    // ----------------------------------------------------------------------------
    // Builder functions to build xml tree snippets programmatically
    // ----------------------------------------------------------------------------

    /**
     * @brief Build a single parallel "plan + execute" block with unique naming
     *        using a static global counter for each move.
     *
     * The signature is:
     *  buildParallelPlanExecuteXML(node_prefix, moves, blackboard, [optional] reset_trajs)
     *
     * Example output:
     *
     *  <ResetTrajectories move_ids="id1,id2,..."/>
     *  <Parallel name="ParallelPlanExecute_{node_prefix}_{blockStartID}" success_threshold="2" failure_threshold="1">
     *    <Sequence name="PlanningSequence_{node_prefix}_{blockStartID}">
     *      <PlanningAction name="PlanMove_{globalID}" ...
     *         move_id="{globalID}"
     *         planned_move_id="{planned_move_id_{globalID}}"
     *         trajectory="{trajectory_{globalID}}"
     *         planning_validity="{validity_{globalID}}"/>
     *      ...
     *    </Sequence>
     *    <Sequence name="ExecutionSequence_{node_prefix}_{blockStartID}">
     *      <ExecuteTrajectory name="ExecMove_{globalID}"
     *         planned_move_id="{planned_move_id_{globalID}}"
     *         trajectory="{trajectory_{globalID}}"
     *         planning_validity="{validity_{globalID}}"/>
     *      ...
     *    </Sequence>
     *  </Parallel>
     *
     * Each move in @p moves increments a global counter, thus guaranteeing each
     * move_id, planned_move_id, validity_, trajectory_ are unique across the entire tree.
     *
     * This function also populates the blackboard with move IDs.
     *
     * WARNING:This function will require the output XML to be wrapped in a Control leaf node,
     *  since ResetTrajectories is detached from its own ParallelPlanExecute leaf node. This is done inside this
     *  function to allow grouping several ParallelPlanExecute in a single sequence to reduce tree's complexity. Moreover,
     *  you can set @p reset_trajs to false to avoid generating ResetTrajectories if your control logic don't require it,
     *  further simplifying the tree's structure.
     *
     * @param node_prefix A label for the parallel block (e.g., "preparatory" or "pickAndHoming")
     * @param moves       The vector of Move that we plan/execute in this parallel block
     * @param blackboard  The blackboard to populate with move IDs
     * @param robot_prefix A prefix for the robot's action servers
     * @param reset_trajs This condition generates the ResetTrajectories leaf node to reset all the sequence's trajs before planning and executing
     * @return A string with the generated XML snippet
     */
    std::string buildParallelPlanExecuteXML(const std::string &robot_prefix,
                                            const std::string &node_prefix,
                                            const std::vector<Move> &moves,
                                            BT::Blackboard::Ptr blackboard,
                                            bool reset_trajs = true);

    /**
     * @brief Builds an XML snippet for a single object action node based on the provided ObjectAction.
     * @param node_prefix A node_prefix to ensure unique node names within the tree.
     * @param action      The ObjectAction struct containing action details.
     * @return A string containing the XML snippet for the object action node.
     * @throws std::invalid_argument If an unsupported ObjectActionType is provided.
     */
    std::string buildObjectActionXML(const std::string &node_prefix, const ObjectAction &action);

    /**
     * @brief Build an XML snippet for SetOutputAction.
     * @param node_prefix  Used to construct a unique name attribute, e.g. "<SetOutputAction name='node_prefix_SetOutput' .../>".
     * @param io_type      The IO type input port (e.g. "tool" or "controller").
     * @param ionum        The IO channel number.
     * @param value        The value of the input to compare to. Accepts 0 or 1, any value that is not 0 will be considered 1.
     * @param robot_prefix A prefix for the robot's action servers
     * @return A string of the XML snippet.
     */
    std::string buildSetOutputXML(const std::string &robot_prefix,
                                  const std::string &node_prefix,
                                  const std::string &io_type,
                                  int ionum,
                                  int value);

    /**
     * @brief Build an XML snippet for GetInputAction.
     * @param node_prefix  Used to construct a unique name attribute.
     * @param io_type      The IO type input port (e.g. "tool" or "controller").
     * @param ionum        The IO channel number to read.
     * @param robot_prefix A prefix for the robot's action servers
     * @return A string of the XML snippet.
     */
    std::string buildGetInputXML(const std::string &robot_prefix,
                                 const std::string &node_prefix,
                                 const std::string &io_type,
                                 int ionum);

    /**
     * @brief Build an XML snippet for to check if an input value corresponds to the one requested.
     * @param node_prefix  Used to construct a unique name attribute.
     * @param io_type      The IO type input port (e.g. "tool" or "controller").
     * @param ionum        The IO channel number to read.
     * @param value        The value of the input to compare to. Accepts 0 or 1, any value that is not 0 will be considered 1.
     * @param robot_prefix A prefix for the robot's action servers
     * @param wait         If true the function waits for the input to have the right value.
     * @param timeout_ms   Milliseconds for timeout, if 0 then no timeout.
     * @return A string of the XML snippet.
     */
    std::string buildCheckInputXML(const std::string &robot_prefix,
                                   const std::string &node_prefix,
                                   const std::string &io_type,
                                   int ionum,
                                   int value,
                                   bool wait = true,
                                   int timeout_ms = 0);

    /**
     * @brief Build an XML snippet for to compare the result of the check if an object exist on the scene.
     * @param node_prefix  Used to construct a unique name attribute.
     * @param robot_prefix A prefix for the robot's action servers.
     * @param object_id    The ID of the object to wait for.
     * @param exists       If true the function waits for the object to exist, if false for the object not to exist.
     * @param timeout_ms   Milliseconds for timeout, if 0 then no timeout.
     * @return A string of the XML snippet.
     */
    std::string buildWaitForObject(const std::string &robot_prefix,
                                   const std::string &node_prefix,
                                   const std::string &object_id,
                                   bool exists = true,
                                   int timeout_ms = 0);

    /**
     * @brief Build an XML snippet for to check if an input value corresponds to the one requested.
     * @param node_prefix       Used to construct a unique name attribute.
     * @param robot_prefix      A prefix for the robot's action servers
     * @param key_id            The ID of the key to wait for.
     * @param expected_value    Expected value of the key to compare to.
     * @param timeout_ms        Milliseconds for timeout, if 0 then no timeout.
     * @return A string of the XML snippet.
     */
    std::string buildWaitForKey(const std::string &robot_prefix,
                                const std::string &node_prefix,
                                const std::string &key_id,
                                const std::string &expected_value,
                                int timeout_ms = 0);

    /**
     * @brief Build an XML snippet for SetBlackboardKeyValue.
     *
     * @param robot_prefix    A prefix for the robot's action servers (not strictly needed,
     *                        but we keep consistency with other build* functions).
     * @param node_prefix     Used to construct a unique name attribute for the node.
     * @param key             The name of the blackboard key to set.
     * @param value           The value to store in the blackboard key.
     * @return A string with the generated XML snippet for the SetBlackboardKeyValue node.
     */
    std::string buildSetBlackboardKey(const std::string &robot_prefix,
                                      const std::string &node_prefix,
                                      const std::string &key,
                                      const std::string &value);

    /**
     * @brief Build an XML snippet for CheckRobotStateAction.
     * @param node_prefix  Used to construct a unique name attribute.
     * @param robot_prefix A prefix for the robot's action servers
     * @param ready_key    (optional) Blackboard key for the "ready" output.
     * @param err_key      (optional) Blackboard key for the "err" output.
     * @param mode_key     (optional) Blackboard key for the "mode" output.
     * @param state_key    (optional) Blackboard key for the "state" output.
     * @param message_key  (optional) Blackboard key for the "message" output.
     * @return A string of the XML snippet.
     */
    std::string buildCheckRobotStateXML(const std::string &robot_prefix,
                                        const std::string &node_prefix,
                                        const std::string &ready_key = "",
                                        const std::string &err_key = "",
                                        const std::string &mode_key = "",
                                        const std::string &state_key = "",
                                        const std::string &message_key = "");

    /**
     * @brief Build an XML snippet for ResetRobotStateAction.
     * @param node_prefix  Used to construct a unique name attribute.
     * @param robot_prefix A prefix for the robot's action servers
     *  The Blackboard key for the "success" output is always "robot_state_success".
     * @return A string of the XML snippet.
     */
    std::string buildResetRobotStateXML(const std::string &robot_prefix,
                                        const std::string &node_prefix,
                                        const std::string &robot_model = "");

    std::string buildStopMotionXML(const std::string &robot_prefix,
                                   const std::string &node_prefix,
                                   double deceleration_time);

    // ----------------------------------------------------------------------------
    // Wrappers
    // ----------------------------------------------------------------------------

    /**
     * @brief Wrap multiple snippets in a <Sequence> with a given name.
     */
    std::string sequenceWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches);

    /**
     * @brief Wrap multiple snippets in a <Parallel> with a given name.
     */
    std::string parallelWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches,
                                   const int &success_threshold,
                                   const int &failure_threshold);

    /**
     * @brief Wrap multiple snippets in a <ReactiveSequence> with a given name.
     *
     * This wrapper is useful for creating tree branches that continuously monitor
     * the robot's state and can interrupt planning/execution branches if necessary.
     *
     * @param sequence_name A unique name for the ReactiveSequence.
     * @param branches      A vector of XML snippets representing the child nodes.
     * @return A string containing the generated XML snippet.
     */
    std::string reactiveWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches);

    /**
     * @brief Wrap multiple snippets in a <RepeatNode> node with a given name.
     *
     * This wrapper allows repeating its child node multiple times based on the specified
     * number of attempts. To repeat indefinitely, set num_cycles to -1.
     *
     * @param sequence_name A unique name for the RepeatNode node.
     * @param branches      A vector of XML snippets representing the child nodes.
     * @param num_cycles    Number of repeat attempts (-1 for infinite retries).
     * @return A string containing the generated XML snippet.
     */
    std::string repeatWrapperXML(const std::string &sequence_name,
                                 const std::vector<std::string> &branches,
                                 const int num_cycles = -1);

    /**
     * @brief Wrap multiple snippets in a <Fallback> with a given name.
     *
     * This wrapper is useful for creating tree branches that try each branch in
     * sequence one is successful.
     *
     * @param sequence_name A unique name for the Fallback.
     * @param branches      A vector of XML snippets representing the child nodes.
     * @return A string containing the generated XML snippet.
     */
    std::string fallbackWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches);

    /**
     * @brief Wrap a snippet in a top-level <root> with <BehaviorTree ID="...">
     *        so it can be loaded by BehaviorTreeFactory.
     */
    std::string mainTreeWrapperXML(const std::string &tree_id,
                                   const std::string &content);

    // ----------------------------------------------------------------------------
    // Helper functions
    // ----------------------------------------------------------------------------

    /**
     * @brief Create a geometry_msgs::msg::Pose easily.
     */
    geometry_msgs::msg::Pose createPose(double x, double y, double z,
                                        double qx, double qy, double qz, double qw);

    /**
     * @brief Returns a pose with quaterion built from rpy values.
     * @param x offset about X axis.
     * @param y offset about Y axis.
     * @param x offset about Z axis.
     * @param roll  rotation about X axis.
     * @param pitch rotation about Y axis.
     * @param yaw   rotation about Z axis.
     * @return geometry_msgs::msg::Pose corresponding to the values inserted
     */
    geometry_msgs::msg::Pose createPoseRPY(const double &x = 0.0,
                                           const double &y = 0.0,
                                           const double &z = 0.0,
                                           const double &roll = 0.0,
                                           const double &pitch = 0.0,
                                           const double &yaw = 0.0);

    /**
     * @brief Helper function to convert ObjectActionType enum to corresponding string.
     * @param type The ObjectActionType enum value.
     * @return A string representing the action node type.
     */
    std::string objectActionTypeToString(ObjectActionType type);

    /**
     * @brief Helper function to serialize a geometry_msgs::msg::Pose into a string.
     * @param pose The Pose message to serialize.
     * @return A string representation of the Pose.
     */
    std::string serializePose(const geometry_msgs::msg::Pose &pose);

    /**
     * @brief Helper function to serialize a std::vector<double> into a comma-separated string.
     * @param vec The vector to serialize.
     * @return A string representation of the vector.
     */
    std::string serializeVector(const std::vector<double> &vec);

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_TREE_HELPER_HPP
