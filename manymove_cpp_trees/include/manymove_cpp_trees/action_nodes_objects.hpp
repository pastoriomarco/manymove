#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_OBJECTS_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_OBJECTS_HPP

#include "manymove_cpp_trees/move.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <manymove_planner/action/plan_manipulator.hpp>
#include <manymove_planner/action/execute_trajectory.hpp>

#include <manymove_object_manager/action/add_collision_object.hpp>
#include <manymove_object_manager/action/remove_collision_object.hpp>
#include <manymove_object_manager/action/attach_detach_object.hpp>
#include <manymove_object_manager/action/check_object_exists.hpp>
#include <manymove_object_manager/action/get_object_pose.hpp>

#include "manymove_signals/action/set_output.hpp"
#include "manymove_signals/action/get_input.hpp"
#include "manymove_signals/action/check_robot_state.hpp"
#include "manymove_signals/action/reset_robot_state.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>

namespace manymove_cpp_trees
{
    /**
     * @class AddCollisionObjectAction
     * @brief A Behavior Tree node that adds a collision object to the planning scene using the AddCollisionObject action server.
     */
    class AddCollisionObjectAction : public BT::StatefulActionNode
    {
    public:
        using AddCollisionObject = manymove_object_manager::action::AddCollisionObject;
        using GoalHandleAddCollisionObject = rclcpp_action::ClientGoalHandle<AddCollisionObject>;

        /**
         * @brief Constructor for the AddCollisionObjectAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        AddCollisionObjectAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
                BT::InputPort<std::string>("shape", "Shape type (e.g., box, mesh)"),
                BT::InputPort<std::vector<double>>("dimensions", "Dimensions for primitive shapes"),
                BT::InputPort<geometry_msgs::msg::Pose>("pose", "Pose of the object"),
                BT::InputPort<std::string>("mesh_file", "", "Mesh file path (for mesh objects)"),
                BT::InputPort<double>("scale_mesh_x", 1.0, "Scale factor along X-axis (for mesh)"),
                BT::InputPort<double>("scale_mesh_y", 1.0, "Scale factor along Y-axis (for mesh)"),
                BT::InputPort<double>("scale_mesh_z", 1.0, "Scale factor along Z-axis (for mesh)")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleAddCollisionObject> goal_handle);
        void resultCallback(const GoalHandleAddCollisionObject::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<AddCollisionObject>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        AddCollisionObject::Result action_result_;
        std::string object_id_; ///< Unique object identifier
    };

    /**
     * @class RemoveCollisionObjectAction
     * @brief A Behavior Tree node that removes a collision object from the planning scene using the RemoveCollisionObject action server.
     */
    class RemoveCollisionObjectAction : public BT::StatefulActionNode
    {
    public:
        using RemoveCollisionObject = manymove_object_manager::action::RemoveCollisionObject;
        using GoalHandleRemoveCollisionObject = rclcpp_action::ClientGoalHandle<RemoveCollisionObject>;

        /**
         * @brief Constructor for the RemoveCollisionObjectAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        RemoveCollisionObjectAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Unique identifier for the object to remove")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleRemoveCollisionObject> goal_handle);
        void resultCallback(const GoalHandleRemoveCollisionObject::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<RemoveCollisionObject>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        RemoveCollisionObject::Result action_result_;
        std::string object_id_; ///< Unique object identifier
    };

    /**
     * @class AttachDetachObjectAction
     * @brief A Behavior Tree node that attaches or detaches a collision object to/from a robot link using the AttachDetachObject action server.
     */
    class AttachDetachObjectAction : public BT::StatefulActionNode
    {
    public:
        using AttachDetachObject = manymove_object_manager::action::AttachDetachObject;
        using GoalHandleAttachDetachObject = rclcpp_action::ClientGoalHandle<AttachDetachObject>;

        /**
         * @brief Constructor for the AttachDetachObjectAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        AttachDetachObjectAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
                BT::InputPort<std::string>("link_name", "Name of the link to attach/detach the object"),
                BT::InputPort<bool>("attach", true, "True to attach, False to detach")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleAttachDetachObject> goal_handle);
        void resultCallback(const GoalHandleAttachDetachObject::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<AttachDetachObject>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        AttachDetachObject::Result action_result_;
        std::string object_id_; ///< Unique object identifier
        std::string link_name_; ///< Link name to attach/detach
        bool attach_;           ///< True to attach, False to detach
    };

    /**
     * @class CheckObjectExistsAction
     * @brief A Behavior Tree node that checks if a collision object exists and whether it's attached using the CheckObjectExists action server.
     */
    class CheckObjectExistsAction : public BT::StatefulActionNode
    {
    public:
        using CheckObjectExists = manymove_object_manager::action::CheckObjectExists;
        using GoalHandleCheckObjectExists = rclcpp_action::ClientGoalHandle<CheckObjectExists>;

        /**
         * @brief Constructor for the CheckObjectExistsAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        CheckObjectExistsAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input and output ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
                BT::OutputPort<bool>("exists", "Indicates if the object exists"),
                BT::OutputPort<bool>("is_attached", "Indicates if the object is attached to a link"),
                BT::OutputPort<std::string>("link_name", "Name of the link the object is attached to, if any")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleCheckObjectExists> goal_handle);
        void resultCallback(const GoalHandleCheckObjectExists::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<CheckObjectExists>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        CheckObjectExists::Result action_result_;
        std::string object_id_; ///< Unique object identifier
    };

    /**
     * @class GetObjectPoseAction
     * @brief A Behavior Tree node that retrieves and modifies the pose of a collision object using the GetObjectPose action server.
     */
    class GetObjectPoseAction : public BT::StatefulActionNode
    {
    public:
        using GetObjectPose = manymove_object_manager::action::GetObjectPose;
        using GoalHandleGetObjectPose = rclcpp_action::ClientGoalHandle<GetObjectPose>;

        /**
         * @brief Constructor for the GetObjectPoseAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        GetObjectPoseAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input and output ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Identifier of the object"),
                BT::InputPort<std::vector<double>>("pre_transform_xyz_rpy", "Offset and rotation {x, y, z, roll, pitch, yaw}"),
                BT::InputPort<std::vector<double>>("post_transform_xyz_rpy", "Reference orientation {roll, pitch, yaw}"),
                BT::InputPort<std::string>("pose_key", "Blackboard key to store the retrieved pose"),
                BT::OutputPort<geometry_msgs::msg::Pose>("pose", "Pose after transformations")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleGetObjectPose> goal_handle);
        void resultCallback(const GoalHandleGetObjectPose::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<GetObjectPose>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        GetObjectPose::Result action_result_;
        std::string object_id_;                      ///< Unique object identifier
        std::vector<double> pre_transform_xyz_rpy_;  ///< Transformation offset and rotation
        std::vector<double> post_transform_xyz_rpy_; ///< Reference orientation
        std::string pose_key_;                       ///< Blackboard key to store the pose
    };

} // namespace manymove_cpp_trees

#endif
