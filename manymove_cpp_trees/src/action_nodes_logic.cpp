#include "manymove_cpp_trees/action_nodes_logic.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

namespace manymove_cpp_trees
{

    RetryPauseResetNode::RetryPauseResetNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::DecoratorNode(name, config)
    {
    }

    BT::NodeStatus RetryPauseResetNode::tick()
    {
        // Read the two controlling blackboard keys:
        bool collision_detected = false;
        bool reset = false;
        bool stop_execution = false;
        std::string robot_prefix = "";

        if (!child_node_)
            throw BT::RuntimeError("RetryPauseResetNode: missing child");

        if ((!getInput("reset", reset)) ||
            (!getInput("stop_execution", stop_execution)) ||
            (!getInput("collision_detected", collision_detected)) ||
            (!getInput("robot_prefix", robot_prefix)))
        {
            throw BT::RuntimeError("RetryPauseResetNode: Missing required input key");
            return BT::NodeStatus::FAILURE;
        }

        // Priority 1: reset is true: halt child and return FAILURE.
        if (reset)
        {
            config().blackboard->set(robot_prefix + "reset", false);
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

    void RetryPauseResetNode::halt()
    {
        if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
            child_node_->halt();
        BT::DecoratorNode::halt();
    }

    // ---------------------------------------------------------------
    // CheckKeyBoolValue Implementation
    // ---------------------------------------------------------------

    CheckKeyBoolValue::CheckKeyBoolValue(const std::string &name,
                                         const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
        // If you need to confirm the blackboard is present:
        if (!config.blackboard)
        {
            throw BT::RuntimeError("CheckKeyBoolValue: no blackboard provided.");
        }
        // If you needed an rclcpp node for logging, you could do:
        // config.blackboard->get("node", node_);
        // but this condition node typically doesn't require a ROS node.
    }

    BT::NodeStatus CheckKeyBoolValue::tick()
    {
        // 1) Extract input ports "key" and "value"
        std::string key;
        std::string expected_value;
        if (!getInput<std::string>("key", key))
        {
            throw BT::RuntimeError("CheckKeyBoolValue: Missing required input [key]");
        }
        if (!getInput<std::string>("value", expected_value))
        {
            throw BT::RuntimeError("CheckKeyBoolValue: Missing required input [value]");
        }

        // 2) Read the blackboard
        bool actual_value;
        if (!config().blackboard->get(key, actual_value))
        {
            // Key not found => we fail
            return BT::NodeStatus::FAILURE;
        }

        // 3) Compare the blackboard value to the expected value
        if ((actual_value ? "true" : "false") == expected_value)
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
    // SetKeyBoolValue Implementation
    // ---------------------------------------------------------------

    BT::NodeStatus SetKeyBoolValue::tick()
    {
        // 1) Read the "key" port
        std::string key;
        if (!getInput<std::string>("key", key))
        {
            throw BT::RuntimeError("SetKeyBoolValue: Missing required input port [key]");
        }

        // 2) Read the "value" port
        bool value;
        if (!getInput<bool>("value", value))
        {
            throw BT::RuntimeError("SetKeyBoolValue: Missing required input port [value]");
        }

        // 3) Set the key in the blackboard to the string
        config().blackboard->set(key, value);

        // 4) Return SUCCESS to indicate we’ve completed setting the key
        return BT::NodeStatus::SUCCESS;
    }

    // ---------------------------------------------------------
    // WaitForKeyBool
    // ---------------------------------------------------------
    WaitForKeyBool::WaitForKeyBool(const std::string &name,
                                   const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          condition_met_(false)
    {
        // If you need access to the node for time, etc.
        if (!config.blackboard)
        {
            throw BT::RuntimeError("WaitForKeyBool: no blackboard provided.");
        }
        config.blackboard->get("node", node_);
    }

    BT::NodeStatus WaitForKeyBool::onStart()
    {
        condition_met_ = false;

        // Read ports
        if (!getInput<std::string>("key", key_))
        {
            RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyBool"),
                         "[%s] missing 'key' input", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput<bool>("expected_value", expected_value_))
        {
            RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyBool"),
                         "[%s] missing 'expected_value' input", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        getInput<double>("timeout", timeout_);
        getInput<double>("poll_rate", poll_rate_);

        // Mark timestamps
        if (!node_)
        {
            // fallback if no node in blackboard => we cannot do fancy timing
            RCLCPP_WARN(rclcpp::get_logger("WaitForKeyBool"),
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

        RCLCPP_INFO(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyBool"),
                    "[%s] WaitForKeyBool: key='%s', expected='%s', timeout=%.2f, poll_rate=%.2f",
                    name().c_str(), key_.c_str(), (expected_value_ ? "true" : "false"), timeout_, poll_rate_);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus WaitForKeyBool::onRunning()
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
        bool actual_value;
        bool found = config().blackboard->get(key_, actual_value);
        if (!found)
        {
            // key not found => no match
            throw BT::RuntimeError("WaitForKeyBool: no [key] input provided.");
        }

        RCLCPP_DEBUG(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyBool"),
                     "[%s] WaitForKeyBool: key='%s', expected='%s', actual='%s' timeout=%.2f, poll_rate=%.2f",
                     name().c_str(), key_.c_str(), (expected_value_ ? "true" : "false"), (actual_value ? "true" : "false"), timeout_, poll_rate_);

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
                            name().c_str(), elapsed, (actual_value ? "true" : "false"));
                return BT::NodeStatus::FAILURE;
            }
        }

        // Otherwise schedule the next check in poll_rate_ s
        next_check_time_ = now + rclcpp::Duration::from_seconds(poll_rate_);
        return BT::NodeStatus::RUNNING;
    }

    void WaitForKeyBool::onHalted()
    {
        RCLCPP_WARN(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyBool"),
                    "[%s] Halted => reset state",
                    name().c_str());
        condition_met_ = false;
    }

    // ===========================================================================
    // GetLinkPoseNode implementation
    // ===========================================================================
    namespace
    {
        constexpr double TF_TIMEOUT_SEC = 0.1;
    } // 100 ms

    GetLinkPoseNode::GetLinkPoseNode(const std::string &name,
                                     const BT::NodeConfiguration &cfg)
        : BT::SyncActionNode(name, cfg)
    {
        if (!cfg.blackboard || !cfg.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("GetLinkPoseNode: cannot retrieve rclcpp::Node "
                                   "from blackboard (key 'node').");
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    BT::NodeStatus GetLinkPoseNode::tick()
    {
        /* ── read mandatory / optional ports ─────────────────────────────── */
        std::string link_name;
        if (!getInput("link_name", link_name) || link_name.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] 'link_name' missing", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        std::string ref_frame;
        getInput("reference_frame", ref_frame);
        std::vector<double> pre, post;
        getInput("pre_transform_xyz_rpy", pre);
        getInput("post_transform_xyz_rpy", post);

        if ((!pre.empty() && pre.size() != 6) ||
            (!post.empty() && post.size() != 6))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[%s] pre/post vectors must contain exactly 6 elements", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        /* ── look-up TF transform link → ref_frame ───────────────────────── */
        geometry_msgs::msg::TransformStamped tf_link_to_ref;
        try
        {
            tf_link_to_ref = tf_buffer_->lookupTransform(
                ref_frame.empty() ? "world" : ref_frame, // target
                link_name,                               // source
                tf2::TimePointZero,
                tf2::durationFromSec(TF_TIMEOUT_SEC));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] TF error: %s", name().c_str(), ex.what());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[%s] RAW tf: {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f}",
                    name().c_str(),
                    tf_link_to_ref.transform.translation.x,
                    tf_link_to_ref.transform.translation.y,
                    tf_link_to_ref.transform.translation.z,
                    tf_link_to_ref.transform.rotation.x,
                    tf_link_to_ref.transform.rotation.y,
                    tf_link_to_ref.transform.rotation.z,
                    tf_link_to_ref.transform.rotation.w);

        /* ── build the 8-step combined transform ─────────────────────────── */
        tf2::Transform combined;
        combined.setIdentity();

        if (!pre.empty())
        {
            tf2::Quaternion q_pre;
            q_pre.setRPY(pre[3], pre[4], pre[5]);
            combined = tf2::Transform(q_pre) *
                       tf2::Transform(tf2::Quaternion::getIdentity(),
                                      tf2::Vector3(pre[0], pre[1], pre[2])) *
                       combined;
        }

        if (!post.empty())
        {
            tf2::Quaternion q_post;
            q_post.setRPY(post[3], post[4], post[5]);
            combined = tf2::Transform(q_post) *
                       tf2::Transform(tf2::Quaternion::getIdentity(),
                                      tf2::Vector3(post[0], post[1], post[2])) *
                       combined;
        }

        /* link orientation + translation */
        const auto &tr = tf_link_to_ref.transform;
        tf2::Quaternion q_link(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w);
        combined = tf2::Transform(tf2::Quaternion::getIdentity(),
                                  tf2::Vector3(tr.translation.x,
                                               tr.translation.y,
                                               tr.translation.z)) *
                   tf2::Transform(q_link) *
                   combined;

        /* ── extract final pose ──────────────────────────────────────────── */
        geometry_msgs::msg::Pose final_pose;
        final_pose.position.x = combined.getOrigin().x();
        final_pose.position.y = combined.getOrigin().y();
        final_pose.position.z = combined.getOrigin().z();

        tf2::Quaternion q_final = combined.getRotation();
        q_final.normalize();
        final_pose.orientation.x = q_final.x();
        final_pose.orientation.y = q_final.y();
        final_pose.orientation.z = q_final.z();
        final_pose.orientation.w = q_final.w();

        RCLCPP_INFO(node_->get_logger(), "GetLinkPoseNode final pose = {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f}",
                    final_pose.position.x,
                    final_pose.position.y,
                    final_pose.position.z,
                    final_pose.orientation.x,
                    final_pose.orientation.y,
                    final_pose.orientation.z,
                    final_pose.orientation.w);

        /* ── write to output & optionally to blackboard ──────────────────── */
        setOutput("pose", final_pose);

        std::string pose_key;
        if (getInput("pose_key", pose_key) && !pose_key.empty())
        {
            config().blackboard->set(pose_key, final_pose);
            RCLCPP_INFO(node_->get_logger(),
                        "GetLinkPoseNode pose written to = %s", pose_key.c_str());
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "GetLinkPoseNode: no pose_key provided, skipping BB write");
        }

        return BT::NodeStatus::SUCCESS;
    }

} // namespace manymove_cpp_trees
