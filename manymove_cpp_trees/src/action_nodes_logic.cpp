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
    // CheckBlackboardKeyValue Implementation
    // ---------------------------------------------------------------
    CheckBlackboardKeyValue::CheckBlackboardKeyValue(const std::string &name,
                                                     const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
        // If you need to confirm the blackboard is present:
        if (!config.blackboard)
        {
            throw BT::RuntimeError("CheckBlackboardKeyValue: no blackboard provided.");
        }
        // If you needed an rclcpp node for logging, you could do:
        // config.blackboard->get("node", node_);
        // but this condition node typically doesn't require an ROS node.
    }

    BT::NodeStatus CheckBlackboardKeyValue::tick()
    {
        // 1) Extract input ports "key" and "value"
        std::string key;
        std::string expected_value;
        if (!getInput<std::string>("key", key))
        {
            throw BT::RuntimeError("CheckBlackboardKeyValue: Missing required input [key]");
        }
        if (!getInput<std::string>("value", expected_value))
        {
            throw BT::RuntimeError("CheckBlackboardKeyValue: Missing required input [value]");
        }

        // 2) Read the blackboard
        std::string actual_value = "";
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

    // ---------------------------------------------------------------
    // SetBlackboardKeyValue Implementation
    // ---------------------------------------------------------------
    BT::NodeStatus SetBlackboardKeyValue::tick()
    {
        // 1) Read the "key" port
        std::string key;
        if (!getInput<std::string>("key", key))
        {
            throw BT::RuntimeError("SetBlackboardKeyValue: Missing required input port [key]");
        }

        // 2) Read the "value" port
        std::string value_str;
        if (!getInput<std::string>("value", value_str))
        {
            throw BT::RuntimeError("SetBlackboardKeyValue: Missing required input port [value]");
        }

        // 3) Set the key in the blackboard to the string
        config().blackboard->set(key, value_str);

        // 4) Return SUCCESS to indicate we’ve completed setting the key
        return BT::NodeStatus::SUCCESS;
    }

    // ---------------------------------------------------------
    // WaitForKeyAction
    // ---------------------------------------------------------
    WaitForKeyAction::WaitForKeyAction(const std::string &name,
                                       const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          condition_met_(false)
    {
        // If you need access to the node for time, etc.
        if (!config.blackboard)
        {
            throw BT::RuntimeError("WaitForKeyAction: no blackboard provided.");
        }
        config.blackboard->get("node", node_); // can be null if not found
    }

    BT::NodeStatus WaitForKeyAction::onStart()
    {
        condition_met_ = false;

        // Read ports
        if (!getInput<std::string>("key", key_))
        {
            RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyAction"),
                         "[%s] missing 'key' input", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput<std::string>("expected_value", expected_value_))
        {
            RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyAction"),
                         "[%s] missing 'expected_value' input", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        getInput<double>("timeout", timeout_);     // default=10
        getInput<double>("poll_rate", poll_rate_); // default=0.25

        // Mark timestamps
        if (!node_)
        {
            // fallback if no node in blackboard => we cannot do fancy timing
            RCLCPP_WARN(rclcpp::get_logger("WaitForKeyAction"),
                        "[%s] No rclcpp::Node found. We'll set times to 0 => single pass only.",
                        name().c_str());
            start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            next_check_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        else
        {
            start_time_ = node_->now();
            next_check_time_ = start_time_;
        }

        RCLCPP_INFO(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyAction"),
                    "[%s] WaitForKeyAction: key='%s', expected='%s', timeout=%.2f, poll_rate=%.2f",
                    name().c_str(), key_.c_str(), expected_value_.c_str(), timeout_, poll_rate_);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus WaitForKeyAction::onRunning()
    {
        if (condition_met_)
        {
            // already found => success
            return BT::NodeStatus::SUCCESS;
        }

        // If we have a node_, do time-based checks
        rclcpp::Time now(0, 0, RCL_ROS_TIME);
        if (node_)
        {
            now = node_->now();
        }

        // Check if it's time to re-check
        if (now < next_check_time_)
        {
            return BT::NodeStatus::RUNNING;
        }

        // do the actual check: read from blackboard
        std::string actual_value;
        bool found = config().blackboard->get(key_, actual_value);
        if (!found)
        {
            // key not found => no match
            actual_value = "";
        }

        if (actual_value == expected_value_)
        {
            condition_met_ = true;
            return BT::NodeStatus::SUCCESS;
        }

        // Not matched => check if we timed out
        if (timeout_ > 0.0 && node_)
        {
            double elapsed = (now - start_time_).seconds();
            if (elapsed >= timeout_)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[%s] Timeout after %.2f s => FAILURE. last_value='%s'",
                            name().c_str(), elapsed, actual_value.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // Otherwise schedule the next check in poll_rate_ s
        next_check_time_ = now + rclcpp::Duration::from_seconds(poll_rate_);
        return BT::NodeStatus::RUNNING;
    }

    void WaitForKeyAction::onHalted()
    {
        RCLCPP_WARN(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyAction"),
                    "[%s] Halted => reset state",
                    name().c_str());
        condition_met_ = false;
    }

} // namespace manymove_cpp_trees
