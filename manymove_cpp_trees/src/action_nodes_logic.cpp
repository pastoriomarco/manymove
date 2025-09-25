#include "manymove_cpp_trees/action_nodes_logic.hpp"
#include "manymove_cpp_trees/hmi_utils.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

namespace manymove_cpp_trees
{

    RetryPauseResetNode::RetryPauseResetNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::DecoratorNode(name, config)
    {
    }

    BT::NodeStatus RetryPauseResetNode::tick()
    {
        // Read the controlling blackboard keys:
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

            // HMI message
            setHMIMessage(config().blackboard, robot_prefix, "RESET STATE", "red");

            return BT::NodeStatus::FAILURE;
        }

        // Priority 2: stop_execution is true: halt child and return RUNNING (i.e. pause or restart the retry loop).
        if (stop_execution)
        {
            if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
                child_node_->halt();

            // HMI message
            setHMIMessage(config().blackboard, robot_prefix, "WAITING FOR EXECUTION START", "yellow");

            return BT::NodeStatus::RUNNING;
        }

        // Priority 3: collision_detected is true: halt child to stop motion but keep going.
        if (collision_detected)
        {
            // halt the child_node_ if running to stop movement, but keep going
            if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
                child_node_->halt();

            // HMI message
            setHMIMessage(config().blackboard, robot_prefix, "COLLISION DETECTED", "red");

            // reset the collision_detected value
            config().blackboard->set("collision_detected", false);
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
    }

    BT::NodeStatus CheckKeyBoolValue::tick()
    {
        // 1) Extract input ports "key" and "value"
        std::string robot_prefix;
        std::string key;
        bool expected_value;
        bool hmi_message_logic;
        if (!getInput<std::string>("robot_prefix", robot_prefix))
        {
            throw BT::RuntimeError("CheckKeyBoolValue: Missing required input [robot_prefix]");
        }
        if (!getInput<std::string>("key", key))
        {
            throw BT::RuntimeError("CheckKeyBoolValue: Missing required input [key]");
        }
        if (!getInput<bool>("value", expected_value))
        {
            throw BT::RuntimeError("CheckKeyBoolValue: Missing required input [value]");
        }
        if (!getInput<bool>("hmi_message_logic", hmi_message_logic))
        {
            throw BT::RuntimeError("CheckKeyBoolValue: Missing required input [hmi_message_logic]");
        }

        // 2) Read the blackboard
        bool actual_value;
        if (!config().blackboard->get(key, actual_value))
        {
            // Key not found => we fail
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("CheckKeyBoolValue"), "Key: %s; Expected value: %s; Actual value: %s", key.c_str(), (expected_value ? "true" : "false"), (actual_value ? "true" : "false"));

        // 3) Compare the blackboard value to the expected value
        if (actual_value == expected_value)
        {
            // HMI message
            if (hmi_message_logic)
            {
                setHMIMessage(config().blackboard, robot_prefix, "KEY VALUE CHECK SUCCEEDED: " + key, "green");
            }

            // Condition satisfied => SUCCESS
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            // HMI message
            if (!hmi_message_logic)
            {
                setHMIMessage(config().blackboard, robot_prefix, "KEY VALUE CHECK FAILED: " + key, "red");
            }

            // Condition not satisfied => FAILURE
            return BT::NodeStatus::FAILURE;
        }
    }

    // ---------------------------------------------------------------
    // SetKeyBoolValue Implementation
    // ---------------------------------------------------------------

    BT::NodeStatus SetKeyBoolValue::tick()
    {
        std::string robot_prefix;
        if (!getInput<std::string>("robot_prefix", robot_prefix))
        {
            throw BT::RuntimeError("SetKeyBoolValue: Missing required input port [robot_prefix]");
        }

        // Read the "key" port
        std::string key;
        if (!getInput<std::string>("key", key))
        {
            throw BT::RuntimeError("SetKeyBoolValue: Missing required input port [key]");
        }

        // Read the "value" port
        bool value;
        if (!getInput<bool>("value", value))
        {
            throw BT::RuntimeError("SetKeyBoolValue: Missing required input port [value]");
        }

        // Set the key in the blackboard to the string
        config().blackboard->set(key, value);

        // HMI message
        setHMIMessage(config().blackboard, robot_prefix, "KEY " + key + " SET TO " + (value ? "true" : "false"), "green");

        // Return SUCCESS to indicate we’ve completed setting the key
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

        if (!getInput<std::string>("prefix", prefix_) || (prefix_ == ""))
        {
            prefix_ = "hmi_";
        }

        // Mark timestamps
        if (!node_)
        {
            // fallback if no node in blackboard => we cannot do usual timing
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
                    "[%s] WaitForKeyBool starting: key='%s', expected='%s', timeout=%.2f, poll_rate=%.2f",
                    name().c_str(), key_.c_str(), (expected_value_ ? "true" : "false"), timeout_, poll_rate_);

        // HMI message
        setHMIMessage(config().blackboard, prefix_, "[" + name() + "]" + "WAITING FOR KEY " + key_, "yellow");

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
        if (!config().blackboard->get(key_, actual_value))
        {
            // key not found => no match
            throw BT::RuntimeError("WaitForKeyBool: no [key] input provided.");
        }

        RCLCPP_DEBUG(node_ ? node_->get_logger() : rclcpp::get_logger("WaitForKeyBool"),
                     "[%s] WaitForKeyBool polling: key='%s', expected='%s', actual='%s' timeout=%.2f, poll_rate=%.2f",
                     name().c_str(), key_.c_str(), (expected_value_ ? "true" : "false"), (actual_value ? "true" : "false"), timeout_, poll_rate_);

        if (actual_value == expected_value_)
        {
            // HMI message
            setHMIMessage(config().blackboard, prefix_, "", "grey");

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

        // HMI message
        setHMIMessage(config().blackboard, prefix_, "[" + name() + "]: " + "WAITING FOR " + key_, "yellow");

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
    // GetLinkPoseAction implementation
    // ===========================================================================
    namespace
    {
        constexpr double TF_TIMEOUT_SEC = 0.1;
    } // 100 ms

    GetLinkPoseAction::GetLinkPoseAction(const std::string &name,
                                         const BT::NodeConfiguration &cfg)
        : BT::SyncActionNode(name, cfg)
    {
        if (!cfg.blackboard || !cfg.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("GetLinkPoseAction: cannot retrieve rclcpp::Node "
                                   "from blackboard (key 'node').");
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    BT::NodeStatus GetLinkPoseAction::tick()
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
                    "GetLinkPoseAction - [%s] - RAW tf: {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f}",
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

        RCLCPP_INFO(node_->get_logger(), "GetLinkPoseAction - [%s] - Final pose = {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f}",
                    name().c_str(),
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
                        "GetLinkPoseAction - [%s] - Pose written to %s", name().c_str(), pose_key.c_str());
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "GetLinkPoseAction - [%s] - No pose_key provided, skipping BB write", name().c_str());
        }

        return BT::NodeStatus::SUCCESS;
    }

    // =========================================================================
    // CheckPoseDistance implementation
    // =========================================================================

    CheckPoseDistance::CheckPoseDistance(const std::string &name,
                                         const BT::NodeConfiguration &cfg)
        : BT::ConditionNode(name, cfg)
    {
        if (!cfg.blackboard)
        {
            throw BT::RuntimeError("CheckPoseDistance: no blackboard provided.");
        }
        cfg.blackboard->get("node", node_);
    }

    BT::NodeStatus CheckPoseDistance::tick()
    {
        std::string reference_key, target_key;
        if (!getInput("reference_pose_key", reference_key))
        {
            throw BT::RuntimeError("CheckPoseDistance: missing input 'reference_pose_key'");
        }
        if (!getInput("target_pose_key", target_key))
        {
            throw BT::RuntimeError("CheckPoseDistance: missing input 'target_pose_key'");
        }

        double tol = 0.01;
        getInput("tolerance", tol);

        geometry_msgs::msg::Pose reference_pose, target_pose;
        if (!config().blackboard->get(reference_key, reference_pose))
        {
            RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("CheckPoseDistance"),
                         "[%s] key '%s' not found", name().c_str(), reference_key.c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!config().blackboard->get(target_key, target_pose))
        {
            RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("CheckPoseDistance"),
                         "[%s] key '%s' not found", name().c_str(), target_key.c_str());
            return BT::NodeStatus::FAILURE;
        }

        double dx = target_pose.position.x - reference_pose.position.x;
        double dy = target_pose.position.y - reference_pose.position.y;
        double dz = target_pose.position.z - reference_pose.position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        RCLCPP_INFO(node_ ? node_->get_logger() : rclcpp::get_logger("CheckPoseDistance"),
                    "[%s] Target pose: [%3f, %3f, %3f]", name().c_str(), target_pose.position.x, target_pose.position.y, target_pose.position.z);

        RCLCPP_INFO(node_ ? node_->get_logger() : rclcpp::get_logger("CheckPoseDistance"),
                    "[%s] Reference pose: [%3f, %3f, %3f]", name().c_str(), reference_pose.position.x, reference_pose.position.y, reference_pose.position.z);

        RCLCPP_INFO(node_ ? node_->get_logger() : rclcpp::get_logger("CheckPoseDistance"),
                    "[%s] distance=%.4f, tol=%.4f", name().c_str(), dist, tol);

        if (dist <= tol)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    // =========================================================================
    // CheckPoseBounds implementation
    // =========================================================================

    CheckPoseBounds::CheckPoseBounds(const std::string &name,
                                     const BT::NodeConfiguration &cfg)
        : BT::ConditionNode(name, cfg)
    {
        if (!cfg.blackboard)
        {
            throw BT::RuntimeError("CheckPoseBounds: no blackboard provided.");
        }
        cfg.blackboard->get("node", node_);
    }

    BT::NodeStatus CheckPoseBounds::tick()
    {
        std::string pose_key;
        if (!getInput("pose_key", pose_key))
        {
            throw BT::RuntimeError("CheckPoseBounds: missing input 'pose_key'");
        }

        std::vector<double> min_b, max_b;
        if (!getInput("min_bounds", min_b))
        {
            throw BT::RuntimeError("CheckPoseBounds: missing input 'min_bounds'");
        }
        if (!getInput("max_bounds", max_b))
        {
            throw BT::RuntimeError("CheckPoseBounds: missing input 'max_bounds'");
        }
        if (min_b.size() != 3 || max_b.size() != 3)
        {
            throw BT::RuntimeError("CheckPoseBounds: 'min_bounds' and 'max_bounds' must have 3 elements");
        }

        bool inclusive = true;
        getInput("inclusive", inclusive);

        geometry_msgs::msg::Pose pose;
        if (!config().blackboard->get(pose_key, pose))
        {
            RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("CheckPoseBounds"),
                         "[%s] key '%s' not found", name().c_str(), pose_key.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Normalize bounds in case user specified them swapped
        const double min_x = std::min(min_b[0], max_b[0]);
        const double max_x = std::max(min_b[0], max_b[0]);
        const double min_y = std::min(min_b[1], max_b[1]);
        const double max_y = std::max(min_b[1], max_b[1]);
        const double min_z = std::min(min_b[2], max_b[2]);
        const double max_z = std::max(min_b[2], max_b[2]);

        const double x = pose.position.x;
        const double y = pose.position.y;
        const double z = pose.position.z;

        bool inside;
        if (inclusive)
        {
            inside = (x >= min_x && x <= max_x) &&
                     (y >= min_y && y <= max_y) &&
                     (z >= min_z && z <= max_z);
        }
        else
        {
            inside = (x > min_x && x < max_x) &&
                     (y > min_y && y < max_y) &&
                     (z > min_z && z < max_z);
        }

        RCLCPP_INFO(node_ ? node_->get_logger() : rclcpp::get_logger("CheckPoseBounds"),
                    "[%s] Pose: [%.3f, %.3f, %.3f], Bounds X:[%.3f, %.3f] Y:[%.3f, %.3f] Z:[%.3f, %.3f] => %s",
                    name().c_str(),
                    x, y, z,
                    min_x, max_x,
                    min_y, max_y,
                    min_z, max_z,
                    inside ? "INSIDE" : "OUTSIDE");

        return inside ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

} // namespace manymove_cpp_trees
