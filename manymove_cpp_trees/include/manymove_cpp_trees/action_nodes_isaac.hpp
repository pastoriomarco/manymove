#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP

#include <string>
#include <memory>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <geometry_msgs/msg/pose.hpp>

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

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_ACTION_NODES_ISAAC_HPP
