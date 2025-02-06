#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP

#include <behaviortree_cpp_v3/decorator_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

namespace manymove_cpp_trees
{
    /**
     * @brief A custom retry node that keeps retrying indefinitely but
     *        if the "abort_mission" blackboard key is true, it halts the child and returns FAILURE;
     *        if the "stop_execution" key is true, it halts the child and returns RUNNING (i.e. it pauses execution).
     *        Otherwise, it ticks its child.
     */
    class RetryPauseAbortNode : public BT::DecoratorNode
    {
    public:
        RetryPauseAbortNode(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("stop_execution", false, "Pause execution when true"),
                BT::InputPort<bool>("collision_detected", false, "Stops current move when true, then retries planning"),
                BT::InputPort<bool>("abort_mission", false, "Abort mission when true")};
        }

        BT::NodeStatus tick() override;
        void halt() override;
    };

    /**
     * @class CheckBlackboardValue
     * @brief A simple condition node that checks if a blackboard key
     *        matches an expected integer value.
     */
    class CheckBlackboardValue : public BT::ConditionNode
    {
    public:
        /**
         * @brief Constructor
         * @param name The node's name in the XML
         * @param config The node's configuration (ports, blackboard, etc.)
         */
        CheckBlackboardValue(const std::string &name,
                             const BT::NodeConfiguration &config);

        /**
         * @brief Required BT ports: "key" (the blackboard key) and "value" (the expected integer).
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("key", "Name of the blackboard key to check"),
                BT::InputPort<int>("value", "Expected integer value"),
                BT::InputPort<std::string>("robot_prefix", "Optional robot namespace prefix, e.g. 'R_' or 'L_'."),
            };
        }

    protected:
        /**
         * @brief The main check. Returns SUCCESS if the blackboard's "key"
         *        equals the expected "value", otherwise FAILURE.
         */
        BT::NodeStatus tick() override;
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP
