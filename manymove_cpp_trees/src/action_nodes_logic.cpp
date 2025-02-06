#include "manymove_cpp_trees/action_nodes_logic.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

namespace manymove_cpp_trees
{

    RetryPauseAbortNode::RetryPauseAbortNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::DecoratorNode(name, config)
    {
    }

    BT::NodeStatus RetryPauseAbortNode::tick()
    {
        // Read the two controlling blackboard keys:
        bool collision_detected = false;
        bool abort_mission = false;
        bool stop_execution = false;

        if (!child_node_)
            throw BT::RuntimeError("RetryPauseAbortNode: missing child");

        if ((!getInput("abort_mission", abort_mission)) || (!getInput("stop_execution", stop_execution)) || (!getInput("collision_detected", collision_detected)))
        {
            throw BT::RuntimeError("RetryPauseAbortNode: Missing required input [key]");
            return BT::NodeStatus::FAILURE;
        }

        // Priority 1: abort_mission is true: halt child and return FAILURE.
        if (abort_mission)
        {
            if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
                child_node_->halt();
            return BT::NodeStatus::FAILURE;
        }

        // Priority 2: stop_execution is true: halt child and return RUNNING (i.e. pause or restart the retry loop).
        if (stop_execution)
        {
            if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
                child_node_->halt();
            return BT::NodeStatus::RUNNING;
        }

        // Priority 3: collision_detected is true: halt child to stop motion but keep going.
        if (collision_detected)
        {
            // halt the child_node_ if running to stop movement, but keep going
            if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
                child_node_->halt();
            /**
             * If we get here there have been a collision detected. With the above call we make the
             * execution halt so the StopMotion will be called. Then the ExecuteTrajectory node will 
             * start and immediately fail: this is required because we can then jump to the planning node
             * of the Fallback child. If we reset the collision_detected here, the ExecuteTrajectory
             * node would wait forever for a valid trajectory, but there wouldn't be any PlanningAction
             * node running in parallel since we already did at least one execution and the planning chain must
             * at least be further than the current execution node (otherwise the execution node would still)
             * be waiting for a valid traj)
             */
        }

        BT::NodeStatus child_status = child_node_->executeTick();

        // If child returns FAILURE, then (like an infinite retry) we return RUNNING so that on the next tick it will be retried.
        if (child_status == BT::NodeStatus::FAILURE)
        {
            return BT::NodeStatus::RUNNING;
        }
        else if (child_status == BT::NodeStatus::SUCCESS)
        {
            // reset the collision_detected value
            config().blackboard->set("collision_detected", false);

            return BT::NodeStatus::SUCCESS;
        }
        else // if (child_status == RUNNING)
            return BT::NodeStatus::RUNNING;
    }

    void RetryPauseAbortNode::halt()
    {
        if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
            child_node_->halt();
        BT::DecoratorNode::halt();
    }

    // ---------------------------------------------------------------
    // CheckBlackboardValue Implementation
    // ---------------------------------------------------------------
    CheckBlackboardValue::CheckBlackboardValue(const std::string &name,
                                               const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
        // If you need to confirm the blackboard is present:
        if (!config.blackboard)
        {
            throw BT::RuntimeError("CheckBlackboardValue: no blackboard provided.");
        }
        // If you needed an rclcpp node for logging, you could do:
        // config.blackboard->get("node", node_);
        // but this condition node typically doesn't require an ROS node.
    }

    BT::NodeStatus CheckBlackboardValue::tick()
    {
        // 1) Extract input ports "key" and "value"
        std::string key;
        int expected_value;
        if (!getInput<std::string>("key", key))
        {
            throw BT::RuntimeError("CheckBlackboardValue: Missing required input [key]");
        }
        if (!getInput<int>("value", expected_value))
        {
            throw BT::RuntimeError("CheckBlackboardValue: Missing required input [value]");
        }

        // 2) Read the blackboard
        int actual_value = 0;
        if (!config().blackboard->get(key, actual_value))
        {
            // Key not found => we fail
            return BT::NodeStatus::FAILURE;
        }

        // 3) Compare the blackboard value to the expected value
        if (actual_value == expected_value)
        {
            // Condition satisfied => SUCCESS
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            // Condition not satisfied => FAILURE
            return BT::NodeStatus::FAILURE;
        }
    }

} // namespace manymove_cpp_trees
