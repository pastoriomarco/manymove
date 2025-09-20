#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP

#include <string>
#include <memory>
#include <future>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <simulation_interfaces/srv/get_entity_state.hpp>
#include <simulation_interfaces/srv/set_entity_state.hpp>

namespace manymove_cpp_trees
{

    // ======================================================================
    // GetEntityPoseNode (async, non-blocking)
    // ======================================================================
    class GetEntityPoseNode : public BT::StatefulActionNode
    {
    public:
        using GetEntityState = simulation_interfaces::srv::GetEntityState;
        using GetClient = rclcpp::Client<GetEntityState>;
        using GetFuture = GetClient::SharedFuture;

        explicit GetEntityPoseNode(const std::string &name,
                                   const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("service_name",
                                           "/isaacsim/GetEntityState",
                                           "GetEntityState service name"),
                // name of a BB key that stores the entity path string
                BT::InputPort<std::string>("entity_path_key",
                                           "Blackboard key holding the entity path string"),
                // name of a BB key where we should store the retrieved Pose
                BT::InputPort<std::string>("pose_key",
                                           "Blackboard key to write the retrieved Pose"),
                // optional direct output
                BT::OutputPort<geometry_msgs::msg::Pose>("pose", "Retrieved Pose")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<GetClient> get_client_;
        std::string current_get_service_name_;

        // per-run state
        bool request_sent_{false};
        std::string entity_path_;
        std::string pose_key_;
        GetFuture future_;
    };

    // ======================================================================
    // SetEntityPoseNode (async, non-blocking)
    // ======================================================================
    class SetEntityPoseNode : public BT::StatefulActionNode
    {
    public:
        using SetEntityState = simulation_interfaces::srv::SetEntityState;
        using SetClient = rclcpp::Client<SetEntityState>;
        using SetFuture = SetClient::SharedFuture;

        explicit SetEntityPoseNode(const std::string &name,
                                   const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("service_name",
                                           "/isaacsim/SetEntityState",
                                           "SetEntityState service name"),
                // name of a BB key that stores the entity path string
                BT::InputPort<std::string>("entity_path_key",
                                           "Blackboard key holding the entity path string"),
                // name of a BB key that stores the Pose to set
                BT::InputPort<std::string>("pose_key",
                                           "Blackboard key holding the Pose to set")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<SetClient> set_client_;
        std::string current_set_service_name_;

        // per-run state
        bool request_sent_{false};
        std::string entity_path_;
        std::string pose_key_;
        geometry_msgs::msg::Pose pose_;
        SetFuture future_;
    };

    /**
     * @brief Normalize the orientation of a pose for grasp alignment.
     *
     * The returned pose is a copy of @p input_pose with the position preserved and
     * the orientation (quaternion) potentially adjusted to form an orthonormal frame
     * consistent with world vertical. The world "down" direction is assumed to be (0, 0, -1).
     *
     * Behavior by @p force_z_vertical:
     *   - true  => Force the local Z axis to be exactly vertical (world down). The X axis is
     *              recomputed as the original X projected onto the horizontal plane; Y is Z × X.
     *              This changes both Z and X to guarantee Z is perfectly vertical.
     *   - false => Preserve the original X axis exactly. Choose Z to be as close as possible to
     *              world down while remaining orthogonal to X (rotate about X); Y is Z × X.
     *
     * Robust fallbacks are used if the input quaternion is degenerate or nearly aligned with
     * world axes; the output quaternion is normalized.
     *
     * @param input_pose       Source pose. Position is returned unchanged.
     * @param force_z_vertical If true, force Z to world vertical; otherwise keep X and make Z as
     *                         vertical as possible subject to X orthogonality.
     * @return Pose with adjusted orientation quaternion.
     */
    geometry_msgs::msg::Pose align_foundationpose_orientation(
        const geometry_msgs::msg::Pose &input_pose,
        bool force_z_vertical = false);

    class FoundationPoseAlignmentNode : public BT::StatefulActionNode
    {
    public:
        using DetectionArray = vision_msgs::msg::Detection3DArray;

        explicit FoundationPoseAlignmentNode(const std::string &name,
                                             const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("input_topic",
                                           "pose_estimation/output",
                                           "Detection3DArray topic from FoundationPose"),
                BT::InputPort<std::string>("pose_key",
                                           "Blackboard key to write the aligned pose"),
                BT::InputPort<std::string>("header_key", "",
                                           "Blackboard key to write the detection header"),
                BT::InputPort<std::string>("target_id", "",
                                           "Filter detections by class id (empty = any)"),
                BT::InputPort<double>("minimum_score", 0.0,
                                      "Minimum hypothesis score to accept"),
                BT::InputPort<double>("timeout", 1.0,
                                      "Seconds to wait for a valid detection (<=0: wait forever)"),
                BT::InputPort<double>("pick_offset", 0.0,
                                      "Offset along aligned +Z to adjust pick pose"),
                BT::InputPort<double>("approach_offset", 0.05,
                                      "Offset along aligned +Z to compute approach pose"),
                BT::InputPort<std::string>("approach_pose_key", "",
                                           "Blackboard key to write computed approach pose"),
                BT::InputPort<std::string>("object_pose_key", "",
                                           "Blackboard key to write the aligned pose for planning scene"),
                BT::InputPort<std::string>("planning_frame", "world",
                                           "Frame where the aligned pose should be expressed"),
                BT::InputPort<double>("transform_timeout", 0.1,
                                      "Timeout (s) when waiting for TF transform to the planning frame"),
                BT::InputPort<bool>("z_threshold_activation", false,
                                    "Enable enforcement of a minimum Z value for the pose"),
                BT::InputPort<double>("z_threshold", 0.0,
                                      "Minimum allowed Z value when the threshold is enabled"),
                BT::InputPort<bool>("normalize_pose", false,
                                    "If false, skip orientation normalization; if true, apply alignment"),
                BT::InputPort<bool>("force_z_vertical", false,
                                    "If true, align the pose so its Z axis is perfectly vertical"),
                BT::OutputPort<geometry_msgs::msg::Pose>("pose",
                                                         "Aligned pose output"),
                BT::OutputPort<geometry_msgs::msg::Pose>("approach_pose",
                                                         "Aligned approach pose output"),
                BT::OutputPort<std_msgs::msg::Header>("header",
                                                      "Header associated with the aligned detection")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        struct DetectionSelection
        {
            vision_msgs::msg::Detection3D detection;
            vision_msgs::msg::ObjectHypothesisWithPose result;
        };

        void ensureSubscription(const std::string &topic);
        void detectionCallback(const DetectionArray::SharedPtr msg);
        std::optional<DetectionSelection> pickDetection(const DetectionArray &array);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<DetectionArray>::SharedPtr subscription_;
        std::string current_topic_;

        std::mutex mutex_;
        DetectionArray latest_detection_;
        bool have_message_{false};
        uint64_t message_sequence_{0};
        uint64_t last_processed_sequence_{0};

        rclcpp::Time start_time_;
        double timeout_seconds_{0.0};
        double minimum_score_{0.0};
        std::string target_id_;
        std::string pose_key_;
        std::string header_key_;
        std::string approach_pose_key_;
        std::string object_pose_key_;
        double pick_offset_{0.0};
        double approach_offset_{0.0};
        bool z_threshold_activation_{false};
        double z_threshold_{0.0};
        bool normalize_pose_{false};
        bool force_z_vertical_{false};
        bool store_pose_{false};
        bool store_header_{false};
        bool store_approach_{false};
        bool store_object_pose_{false};
        std::string planning_frame_{"world"};
        double transform_timeout_{0.1};

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP
