#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/decorator_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace manymove_cpp_trees
{
    /**
     * @brief A custom retry node that keeps retrying indefinitely but
     *        if the "reset" blackboard key is true, it halts the child and returns FAILURE;
     *        if the "stop_execution" key is true, it halts the child and returns RUNNING (i.e. it pauses execution).
     *        Otherwise, it ticks its child.
     */
    class RetryPauseResetNode : public BT::DecoratorNode
    {
    public:
        RetryPauseResetNode(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("stop_execution", false, "Pause execution when true"),
                BT::InputPort<bool>("collision_detected", false, "Stops current move when true, then retries planning"),
                BT::InputPort<bool>("reset", false, "Reset branch when true")};
        }

        BT::NodeStatus tick() override;
        void halt() override;
    };

    /**
     * @class CheckBlackboardKeyValue
     * @brief A simple condition node that checks if a blackboard key
     *        matches an expected string value.
     */
    class CheckBlackboardKeyValue : public BT::ConditionNode
    {
    public:
        /**
         * @brief Constructor
         * @param name The node's name in the XML
         * @param config The node's configuration (ports, blackboard, etc.)
         */
        CheckBlackboardKeyValue(const std::string &name,
                                const BT::NodeConfiguration &config);

        /**
         * @brief Required BT ports: "key" (the blackboard key) and "value" (the expected value).
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("key", "Name of the blackboard key to check"),
                BT::InputPort<std::string>("value", "Expected value"),
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

    /**
     * @class SetBlackboardKeyValue
     * @brief A node that sets a blackboard key to a given string value.
     *
     * Usage example in XML:
     *   <SetBlackboardKeyValue name="SetKeyExample" key="some_key" value="foo"/>
     */
    class SetBlackboardKeyValue : public BT::SyncActionNode
    {
    public:
        // Constructor
        SetBlackboardKeyValue(const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config)
        {
        }

        // Required interface: which ports are needed/offered?
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("key", "Blackboard key to set"),
                BT::InputPort<std::string>("value", "Value to set (as a string)")};
        }

        // The main tick function; sets the blackboard key to the specified string
        BT::NodeStatus tick() override;
    };

    /**
     * @class WaitForKeyAction
     * @brief Periodically checks a blackboard key for a string value = expected_value.
     *        If it matches => SUCCESS, if timeout is reached => FAILURE.
     *        Timeout=0 => infinite wait. Poll_rate => how often to re-check.
     *
     * Ports:
     *   - "key" (string) : blackboard key name
     *   - "expected_value" (string) : the value we want
     *   - "timeout" (double) : seconds, 0 => infinite
     *   - "poll_rate" (double) : frequency in s
     */
    class WaitForKeyAction : public BT::StatefulActionNode
    {
    public:
        WaitForKeyAction(const std::string &name,
                         const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("key", "Blackboard key to read"),
                BT::InputPort<std::string>("expected_value", "Desired string value"),
                BT::InputPort<double>("timeout", 10.0, "Seconds before giving up (0 => infinite)"),
                BT::InputPort<double>("poll_rate", 0.25, "Check frequency (seconds)")};
        }

    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        // read from ports:
        std::string key_;
        std::string expected_value_;
        double timeout_;
        double poll_rate_;

        // time management
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;
        rclcpp::Time next_check_time_;

        bool condition_met_;
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_ACTION_NODES_LOGIC_HPP
