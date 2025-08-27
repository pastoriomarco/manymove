#include "manymove_cpp_trees/action_nodes_isaac.hpp"

#include <chrono>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

namespace manymove_cpp_trees
{

    // ======================================================================
    // GetEntityPoseNode
    // ======================================================================
    GetEntityPoseNode::GetEntityPoseNode(const std::string &name,
                                         const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        if (!config.blackboard || !config.blackboard->get("node", node_) || !node_)
        {
            throw BT::RuntimeError("GetEntityPoseNode: missing 'node' in blackboard");
        }
        current_get_service_name_ = "/isaacsim/GetEntityState";
        get_client_ = node_->create_client<GetEntityState>(current_get_service_name_);
    }

    BT::NodeStatus GetEntityPoseNode::onStart()
    {
        // Read fixed ports (strings naming BB keys)
        std::string service_name = current_get_service_name_;
        getInput("service_name", service_name);

        if (!getInput("entity_path_key", entity_path_) || entity_path_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing input 'entity_path_key'", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput("pose_key", pose_key_) || pose_key_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing input 'pose_key'", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Read entity path from BB
        if (!config().blackboard->get(entity_path_, entity_path_) || entity_path_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] BB key '%s' not found or empty",
                         name().c_str(), entity_path_.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // (Re)create client if service name changed
        if (!get_client_ || service_name != current_get_service_name_)
        {
            current_get_service_name_ = service_name;
            get_client_ = node_->create_client<GetEntityState>(current_get_service_name_);
        }

        // If service not up yet, keep trying (node will be ticked again)
        if (!get_client_->wait_for_service(0s))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[%s] Waiting for service '%s'...",
                                 name().c_str(), current_get_service_name_.c_str());
            return BT::NodeStatus::RUNNING;
        }

        auto req = std::make_shared<GetEntityState::Request>();
        req->entity = entity_path_;

        auto pending = get_client_->async_send_request(req);
        future_ = pending.future.share();
        request_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus GetEntityPoseNode::onRunning()
    {
        if (!request_sent_)
        {
            // Shouldn't really happen, but be defensive.
            return BT::NodeStatus::FAILURE;
        }

        if (future_.wait_for(0s) != std::future_status::ready)
        {
            return BT::NodeStatus::RUNNING;
        }

        auto resp = future_.get();
        request_sent_ = false;

        if (!resp || resp->result.result != 1)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] GetEntityState error: %s",
                         name().c_str(),
                         resp ? resp->result.error_message.c_str() : "null response");
            return BT::NodeStatus::FAILURE;
        }

        const auto pose = resp->state.pose;
        config().blackboard->set(pose_key_, pose);
        setOutput("pose", pose);

        RCLCPP_INFO(node_->get_logger(),
                    "[%s] pose of '%s' -> BB['%s'] "
                    "(%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f)",
                    name().c_str(), entity_path_.c_str(), pose_key_.c_str(),
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        return BT::NodeStatus::SUCCESS;
    }

    void GetEntityPoseNode::onHalted()
    {
        // nothing to cancel for services; just reset flags
        request_sent_ = false;
    }

    // ======================================================================
    // SetEntityPoseNode
    // ======================================================================
    SetEntityPoseNode::SetEntityPoseNode(const std::string &name,
                                         const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        if (!config.blackboard || !config.blackboard->get("node", node_) || !node_)
        {
            throw BT::RuntimeError("SetEntityPoseNode: missing 'node' in blackboard");
        }
        current_set_service_name_ = "/isaacsim/SetEntityState";
        set_client_ = node_->create_client<SetEntityState>(current_set_service_name_);
    }

    BT::NodeStatus SetEntityPoseNode::onStart()
    {
        std::string service_name = current_set_service_name_;
        getInput("service_name", service_name);

        if (!getInput("entity_path_key", entity_path_) || entity_path_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing input 'entity_path_key'", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput("pose_key", pose_key_) || pose_key_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing input 'pose_key'", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Resolve values from BB
        if (!config().blackboard->get(entity_path_, entity_path_) || entity_path_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] BB key '%s' not found or empty",
                         name().c_str(), entity_path_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!config().blackboard->get(pose_key_, pose_))
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] BB key '%s' not found (Pose)",
                         name().c_str(), pose_key_.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // (Re)create client if service name changed
        if (!set_client_ || service_name != current_set_service_name_)
        {
            current_set_service_name_ = service_name;
            set_client_ = node_->create_client<SetEntityState>(current_set_service_name_);
        }

        if (!set_client_->wait_for_service(0s))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[%s] Waiting for service '%s'...",
                                 name().c_str(), current_set_service_name_.c_str());
            return BT::NodeStatus::RUNNING;
        }

        auto req = std::make_shared<SetEntityState::Request>();
        req->entity = entity_path_;
        req->state.header = std_msgs::msg::Header(); // default
        req->state.pose = pose_;
        req->state.twist = geometry_msgs::msg::Twist();        // zeros
        req->state.acceleration = geometry_msgs::msg::Accel(); // zeros

        auto pending = set_client_->async_send_request(req);
        future_ = pending.future.share();
        request_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus SetEntityPoseNode::onRunning()
    {
        if (!request_sent_)
        {
            return BT::NodeStatus::FAILURE;
        }

        if (future_.wait_for(0s) != std::future_status::ready)
        {
            return BT::NodeStatus::RUNNING;
        }

        auto resp = future_.get();
        request_sent_ = false;

        if (!resp || resp->result.result != 1)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] SetEntityState error: %s",
                         name().c_str(),
                         resp ? resp->result.error_message.c_str() : "null response");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[%s] pose set on '%s' from BB['%s'] "
                    "(%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f)",
                    name().c_str(), entity_path_.c_str(), pose_key_.c_str(),
                    pose_.position.x, pose_.position.y, pose_.position.z,
                    pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);

        return BT::NodeStatus::SUCCESS;
    }

    void SetEntityPoseNode::onHalted()
    {
        request_sent_ = false;
    }

} // namespace manymove_cpp_trees
