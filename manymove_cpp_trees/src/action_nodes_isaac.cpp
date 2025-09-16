#include "manymove_cpp_trees/action_nodes_isaac.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

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

    namespace
    {
    constexpr double kEpsilon = 1e-6;

    inline tf2::Vector3 projectOntoPlane(const tf2::Vector3 &vector,
                                         const tf2::Vector3 &normal)
    {
        return vector - (vector.dot(normal)) * normal;
    }

    inline tf2::Vector3 pickPerpendicularFallback(const tf2::Vector3 &axis)
    {
        const tf2::Vector3 world_y(0.0, 1.0, 0.0);
        tf2::Vector3 candidate = projectOntoPlane(world_y, axis);
        if (candidate.length2() > kEpsilon)
        {
            candidate.normalize();
            return candidate;
        }

        const tf2::Vector3 world_z(0.0, 0.0, 1.0);
        candidate = projectOntoPlane(world_z, axis);
        if (candidate.length2() > kEpsilon)
        {
            candidate.normalize();
            return candidate;
        }

        const tf2::Vector3 world_x(1.0, 0.0, 0.0);
        candidate = projectOntoPlane(world_x, axis);
        if (candidate.length2() > kEpsilon)
        {
            candidate.normalize();
            return candidate;
        }

        return tf2::Vector3(0.0, 0.0, 1.0);
    }

    } // namespace

    geometry_msgs::msg::Pose align_foundationpose_orientation(
        const geometry_msgs::msg::Pose &input_pose)
    {
        tf2::Quaternion source_q(input_pose.orientation.x,
                                  input_pose.orientation.y,
                                  input_pose.orientation.z,
                                  input_pose.orientation.w);
        if (source_q.length2() > 0.0)
        {
            source_q.normalize();
        }

        tf2::Matrix3x3 source_matrix(source_q);
        const tf2::Vector3 x_axis = source_matrix.getColumn(0).normalized();
        const tf2::Vector3 world_z(0.0, 0.0, 1.0);

        tf2::Vector3 projected_vertical = projectOntoPlane(world_z, x_axis);
        tf2::Vector3 new_z;
        if (projected_vertical.length2() > kEpsilon)
        {
            new_z = projected_vertical.normalized();
        }
        else
        {
            new_z = pickPerpendicularFallback(x_axis);
        }

        tf2::Vector3 new_y = new_z.cross(x_axis);
        if (new_y.length2() < kEpsilon)
        {
            tf2::Vector3 helper = pickPerpendicularFallback(x_axis);
            new_z = helper;
            new_y = new_z.cross(x_axis);
        }

        new_y.normalize();
        tf2::Vector3 corrected_z = x_axis.cross(new_y).normalized();

        tf2::Matrix3x3 corrected_matrix(
            x_axis.x(), new_y.x(), corrected_z.x(),
            x_axis.y(), new_y.y(), corrected_z.y(),
            x_axis.z(), new_y.z(), corrected_z.z());

        tf2::Quaternion corrected_q;
        corrected_matrix.getRotation(corrected_q);

        geometry_msgs::msg::Pose result = input_pose;
        result.orientation.x = corrected_q.x();
        result.orientation.y = corrected_q.y();
        result.orientation.z = corrected_q.z();
        result.orientation.w = corrected_q.w();
        return result;
    }

    FoundationPoseAlignmentNode::FoundationPoseAlignmentNode(const std::string &name,
                                                             const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        if (!config.blackboard || !config.blackboard->get("node", node_) || !node_)
        {
            throw BT::RuntimeError("FoundationPoseAlignmentNode: missing 'node' in blackboard");
        }

    }

    void FoundationPoseAlignmentNode::ensureSubscription(const std::string &topic)
    {
        if (subscription_ && topic == current_topic_
            && subscription_->get_topic_name() == topic)
        {
            return;
        }

        current_topic_ = topic;
        auto callback = [this](DetectionArray::SharedPtr msg)
        {
            detectionCallback(std::move(msg));
        };
        subscription_ = node_->create_subscription<DetectionArray>(
            topic, rclcpp::SensorDataQoS(), callback);
    }

    void FoundationPoseAlignmentNode::detectionCallback(const DetectionArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_detection_ = *msg;
        have_message_ = true;
        ++message_sequence_;
    }

    std::optional<FoundationPoseAlignmentNode::DetectionSelection>
    FoundationPoseAlignmentNode::pickDetection(const DetectionArray &array)
    {
        std::optional<DetectionSelection> best_selection;
        double best_score = -1.0;

        for (const auto &detection : array.detections)
        {
            for (const auto &result : detection.results)
            {
                const auto &hypothesis = result.hypothesis;
                if (!target_id_.empty() && hypothesis.class_id != target_id_)
                {
                    continue;
                }
                if (hypothesis.score < minimum_score_)
                {
                    continue;
                }
                if (!best_selection || hypothesis.score > best_score)
                {
                    best_selection = DetectionSelection{detection, result};
                    best_score = hypothesis.score;
                }
            }
        }

        return best_selection;
    }

    BT::NodeStatus FoundationPoseAlignmentNode::onStart()
    {
        std::string topic;
        if (!getInput("input_topic", topic))
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input 'input_topic'", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("pose_key", pose_key_) || pose_key_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input 'pose_key'", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        getInput("header_key", header_key_);
        getInput("target_id", target_id_);
        getInput("minimum_score", minimum_score_);
        getInput("timeout", timeout_seconds_);
        getInput("approach_pose_key", approach_pose_key_);
        getInput("object_pose_key", object_pose_key_);
        getInput("approach_offset", approach_offset_);

        store_pose_ = !pose_key_.empty();
        store_header_ = !header_key_.empty();
        store_approach_ = !approach_pose_key_.empty();
        store_object_pose_ = !object_pose_key_.empty();

        ensureSubscription(topic);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_processed_sequence_ = message_sequence_;
        }

        start_time_ = node_->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus FoundationPoseAlignmentNode::onRunning()
    {
        DetectionArray message_snapshot;
        bool has_new_message = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (have_message_ && message_sequence_ != last_processed_sequence_)
            {
                message_snapshot = latest_detection_;
                last_processed_sequence_ = message_sequence_;
                has_new_message = true;
            }
        }

        auto clock = node_->get_clock();

        if (!has_new_message)
        {
            if (timeout_seconds_ > 0.0)
            {
                const rclcpp::Duration elapsed = clock->now() - start_time_;
                if (elapsed.seconds() > timeout_seconds_)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "[%s] Timed out waiting for detections on '%s'",
                                name().c_str(), current_topic_.c_str());
                    return BT::NodeStatus::FAILURE;
                }
            }
            return BT::NodeStatus::RUNNING;
        }

        const auto selection = pickDetection(message_snapshot);
        if (!selection)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock, 5000,
                                 "[%s] No detection passed filters (target_id='%s', min_score=%.3f)",
                                 name().c_str(), target_id_.c_str(), minimum_score_);

            if (timeout_seconds_ > 0.0)
            {
                const rclcpp::Duration elapsed = clock->now() - start_time_;
                if (elapsed.seconds() > timeout_seconds_)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "[%s] Timed out waiting for a valid detection (target_id='%s', min_score=%.3f)",
                                name().c_str(), target_id_.c_str(), minimum_score_);
                    return BT::NodeStatus::FAILURE;
                }
            }
            return BT::NodeStatus::RUNNING;
        }

        geometry_msgs::msg::Pose corrected_pose = align_foundationpose_orientation(
            selection->result.pose.pose);

        if (store_pose_)
        {
            config().blackboard->set(pose_key_, corrected_pose);
        }
        if (store_object_pose_)
        {
            config().blackboard->set(object_pose_key_, corrected_pose);
        }
        setOutput("pose", corrected_pose);

        geometry_msgs::msg::Pose approach_pose = corrected_pose;
        bool compute_approach = std::abs(approach_offset_) > kEpsilon || store_approach_;
        if (compute_approach)
        {
            tf2::Quaternion q(corrected_pose.orientation.x,
                               corrected_pose.orientation.y,
                               corrected_pose.orientation.z,
                               corrected_pose.orientation.w);
            if (q.length2() > 0.0)
            {
                q.normalize();
            }
            tf2::Matrix3x3 rotation(q);
            tf2::Vector3 aligned_z = rotation.getColumn(2);
            if (aligned_z.length2() < kEpsilon)
            {
                aligned_z = tf2::Vector3(0.0, 0.0, 1.0);
            }
            tf2::Vector3 offset_vec = aligned_z.normalized() * approach_offset_;
            approach_pose.position.x += offset_vec.x();
            approach_pose.position.y += offset_vec.y();
            approach_pose.position.z += offset_vec.z();

            if (store_approach_)
            {
                config().blackboard->set(approach_pose_key_, approach_pose);
            }
            setOutput("approach_pose", approach_pose);
        }
        else
        {
            setOutput("approach_pose", approach_pose);
        }

        std_msgs::msg::Header header = selection->detection.header;
        if (header.frame_id.empty())
        {
            header = message_snapshot.header;
        }
        if (store_header_)
        {
            config().blackboard->set(header_key_, header);
        }
        setOutput("header", header);

        RCLCPP_INFO(node_->get_logger(),
                    "[%s] aligned pose published (%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f)%s",
                    name().c_str(),
                    corrected_pose.position.x, corrected_pose.position.y, corrected_pose.position.z,
                    corrected_pose.orientation.x, corrected_pose.orientation.y,
                    corrected_pose.orientation.z, corrected_pose.orientation.w,
                    compute_approach ? " with approach pose" : "");

        return BT::NodeStatus::SUCCESS;
    }

    void FoundationPoseAlignmentNode::onHalted()
    {
        // No specific action required; keep the last detection
    }

} // namespace manymove_cpp_trees
