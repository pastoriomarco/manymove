#ifndef OBJECT_MANAGER__OBJECT_MANAGER_HPP_
#define OBJECT_MANAGER__OBJECT_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <manymove_object_manager/action/add_collision_object.hpp>
#include <manymove_object_manager/action/remove_collision_object.hpp>
#include <manymove_object_manager/action/check_object_exists.hpp>
#include <manymove_object_manager/action/attach_detach_object.hpp>
#include <manymove_object_manager/action/get_object_pose.hpp>

#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <optional>

namespace manymove_object_manager
{
    /**
     * @class ObjectManagerNode
     * @brief A node to manage collision objects in a planning scene.
     *
     * This class provides action servers for adding, removing, checking the existence,
     * attaching/detaching collision objects, and retrieving object poses with optional
     * orientation alignment in a MoveIt-based planning scene. It publishes collision objects,
     * interacts with the /get_planning_scene service, and includes retry mechanisms for
     * confirming changes in the planning scene.
     */
    class ObjectManagerNode : public rclcpp::Node
    {
    public:
        // Alias declarations for actions and their goal handles
        using AddCollisionObject = manymove_object_manager::action::AddCollisionObject;
        using RemoveCollisionObject = manymove_object_manager::action::RemoveCollisionObject;
        using CheckObjectExists = manymove_object_manager::action::CheckObjectExists;
        using AttachDetachObject = manymove_object_manager::action::AttachDetachObject;
        using GetObjectPose = manymove_object_manager::action::GetObjectPose;

        using AddGoalHandle = rclcpp_action::ServerGoalHandle<AddCollisionObject>;
        using RemoveGoalHandle = rclcpp_action::ServerGoalHandle<RemoveCollisionObject>;
        using CheckGoalHandle = rclcpp_action::ServerGoalHandle<CheckObjectExists>;
        using AttachDetachGoalHandle = rclcpp_action::ServerGoalHandle<AttachDetachObject>;
        using GetObjectPoseGoalHandle = rclcpp_action::ServerGoalHandle<GetObjectPose>;

        /**
         * @brief Constructor for ObjectManagerNode.
         *
         * Initializes the node, declares parameters, creates publishers,
         * clients, and action servers for add, remove, check existence,
         * attach/detach, and get object pose operations.
         */
        ObjectManagerNode();

    private:
        // ------------------------
        // Action server callbacks
        // ------------------------

        // AddCollisionObject Action Server Callbacks

        /**
         * @brief Callback to handle a new goal request for AddCollisionObject.
         * @param uuid Unique identifier for the goal request.
         * @param goal The goal details to add a collision object.
         * @return A response indicating acceptance or rejection.
         */
        rclcpp_action::GoalResponse handleAddGoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const AddCollisionObject::Goal> goal);

        /**
         * @brief Callback to handle a cancel request for AddCollisionObject.
         * @param goal_handle The handle to the current goal.
         * @return A response indicating acceptance or rejection of the cancel request.
         */
        rclcpp_action::CancelResponse handleAddCancel(
            const std::shared_ptr<AddGoalHandle> goal_handle);

        /**
         * @brief Callback to handle execution of the add operation.
         * @param goal_handle The handle to the goal being executed.
         */
        void handleAddExecute(
            const std::shared_ptr<AddGoalHandle> goal_handle);

        // RemoveCollisionObject Action Server Callbacks

        /**
         * @brief Callback to handle a new goal request for RemoveCollisionObject.
         * @param uuid Unique identifier for the goal request.
         * @param goal The goal details to remove a collision object.
         * @return A response indicating acceptance or rejection.
         */
        rclcpp_action::GoalResponse handleRemoveGoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const RemoveCollisionObject::Goal> goal);

        /**
         * @brief Callback to handle a cancel request for RemoveCollisionObject.
         * @param goal_handle The handle to the current goal.
         * @return A response indicating acceptance or rejection of the cancel request.
         */
        rclcpp_action::CancelResponse handleRemoveCancel(
            const std::shared_ptr<RemoveGoalHandle> goal_handle);

        /**
         * @brief Callback to handle execution of the remove operation.
         * @param goal_handle The handle to the goal being executed.
         */
        void handleRemoveExecute(
            const std::shared_ptr<RemoveGoalHandle> goal_handle);

        // CheckObjectExists Action Server Callbacks

        /**
         * @brief Callback to handle a new goal request for CheckObjectExists.
         * @param uuid Unique identifier for the goal request.
         * @param goal The goal details to check object existence.
         * @return A response indicating acceptance or rejection.
         */
        rclcpp_action::GoalResponse handleCheckExistGoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CheckObjectExists::Goal> goal);

        /**
         * @brief Callback to handle a cancel request for CheckObjectExists.
         * @param goal_handle The handle to the current goal.
         * @return A response indicating acceptance or rejection of the cancel request.
         */
        rclcpp_action::CancelResponse handleCheckExistCancel(
            const std::shared_ptr<CheckGoalHandle> goal_handle);

        /**
         * @brief Callback to handle execution of the check existence operation.
         * @param goal_handle The handle to the goal being executed.
         */
        void handleCheckExistExecute(
            const std::shared_ptr<CheckGoalHandle> goal_handle);

        // AttachDetachObject Action Server Callbacks

        /**
         * @brief Callback to handle a new goal request for AttachDetachObject.
         * @param uuid Unique identifier for the goal request.
         * @param goal The goal details to attach or detach a collision object.
         * @return A response indicating acceptance or rejection.
         */
        rclcpp_action::GoalResponse handleAttachDetachGoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const AttachDetachObject::Goal> goal);

        /**
         * @brief Callback to handle a cancel request for AttachDetachObject.
         * @param goal_handle The handle to the current goal.
         * @return A response indicating acceptance or rejection of the cancel request.
         */
        rclcpp_action::CancelResponse handleAttachDetachCancel(
            const std::shared_ptr<AttachDetachGoalHandle> goal_handle);

        /**
         * @brief Callback to handle execution of the attach/detach operation.
         * @param goal_handle The handle to the goal being executed.
         */
        void handleAttachDetachExecute(
            const std::shared_ptr<AttachDetachGoalHandle> goal_handle);

        // GetObjectPose Action Server Callbacks

        /**
         * @brief Callback to handle a new goal request for GetObjectPose.
         * @param uuid Unique identifier for the goal request.
         * @param goal The goal details to retrieve and optionally align the object pose.
         * @return A response indicating acceptance or rejection.
         */
        rclcpp_action::GoalResponse handleGetObjectPoseGoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const GetObjectPose::Goal> goal);

        /**
         * @brief Callback to handle a cancel request for GetObjectPose.
         * @param goal_handle The handle to the current goal.
         * @return A response indicating acceptance or rejection of the cancel request.
         */
        rclcpp_action::CancelResponse handleGetObjectPoseCancel(
            const std::shared_ptr<GetObjectPoseGoalHandle> goal_handle);

        /**
         * @brief Callback to handle execution of the get object pose operation.
         * @param goal_handle The handle to the goal being executed.
         */
        void handleGetObjectPoseExecute(
            const std::shared_ptr<GetObjectPoseGoalHandle> goal_handle);

        // ------------------
        // Utility methods
        // ------------------

        /**
         * @brief Create a CollisionObject message from the provided parameters.
         * @param id The unique identifier for the object.
         * @param shape The shape type (box, cylinder, sphere, mesh).
         * @param dimensions Dimensions relevant to the shape type.
         * @param pose The pose of the collision object.
         * @param mesh_file Optional file path to a mesh resource (for mesh shape).
         * @param scale_mesh_x Optional value to scale the original mesh on X axis
         * @param scale_mesh_y Optional value to scale the original mesh on Y axis
         * @param scale_mesh_z Optional value to scale the original mesh on Z axis
         * @return A fully populated CollisionObject message.
         */
        moveit_msgs::msg::CollisionObject createCollisionObject(
            const std::string &id,
            const std::string &shape,
            const std::vector<double> &dimensions,
            const geometry_msgs::msg::Pose &pose,
            const std::string &mesh_file,
            const double scale_mesh_x = 1.0,
            const double scale_mesh_y = 1.0,
            const double scale_mesh_z = 1.0) const;

        /**
         * @brief Print the current planning scene, listing all collision and attached objects found.
         */
        void printPlanningScene();

        /**
         * @brief Validate the shape, dimensions, and optional mesh file for a collision object.
         * @param shape The shape type (box, cylinder, sphere, mesh).
         * @param dimensions The required dimension vector.
         * @param mesh_file Optional mesh file path if shape is 'mesh'.
         * @return True if valid, false otherwise.
         */
        bool validateShapeAndDimensions(
            const std::string &shape,
            const std::vector<double> &dimensions,
            const std::optional<std::string> &mesh_file) const;

        /**
         * @brief Check if an object exists in the planning scene.
         * @param id The identifier of the object to look for.
         * @return True if the object is found, false otherwise.
         */
        bool objectExists(const std::string &id);

        /**
         * @brief Check if an attached object exists in the planning scene.
         * @param id The identifier of the object to look for.
         * @param link_name The link where the object should be attached (empty for any link).
         * @return True if the object is found, false otherwise.
         */
        bool attachedObjectExists(const std::string &id, const std::string &link_name = "");

        /**
         * @brief Get the data of an object from the planning scene for further use.
         * @param object_id The identifier of the object for which to get the data.
         * @return A std::optional containing the CollisionObject if found, std::nullopt otherwise.
         */
        std::optional<moveit_msgs::msg::CollisionObject> getObjectDataById(const std::string &object_id);

        /**
         * @brief Get the link that an object is attached to, if any.
         * @param object_id The identifier of the object for which to get the link.
         * @return A std::optional containing the link name if found, std::nullopt otherwise.
         */
        std::optional<std::string> getAttachedObjectLinkById(const std::string &object_id);

        // ------------------
        // Member variables
        // ------------------
        rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_publisher_;                  ///< Publishes collision objects to the planning scene.
        rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attached_collision_object_publisher_; ///< Publishes attached collision objects to the planning scene.

        rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_; ///< Client for retrieving the planning scene.

        // Action servers for managing collision objects
        rclcpp_action::Server<AddCollisionObject>::SharedPtr add_object_action_server_;           ///< Action server for adding collision objects.
        rclcpp_action::Server<RemoveCollisionObject>::SharedPtr remove_object_action_server_;     ///< Action server for removing collision objects.
        rclcpp_action::Server<CheckObjectExists>::SharedPtr check_object_exists_action_server_;   ///< Action server for checking object existence.
        rclcpp_action::Server<AttachDetachObject>::SharedPtr attach_detach_object_action_server_; ///< Action server for attaching and detaching objects from specified link.
        rclcpp_action::Server<GetObjectPose>::SharedPtr get_object_pose_action_server_;           ///< Action server for retrieving and aligning object poses.

        std::string frame_id_; ///< Reference frame in which collision objects are defined.

        // Callback groups
        rclcpp::CallbackGroup::SharedPtr service_callback_group_; ///< Callback group for the /get_planning_scene service.
        rclcpp::CallbackGroup::SharedPtr action_callback_group_;  ///< Callback group for action servers.
    };
} // namespace manymove_object_manager

#endif // OBJECT_MANAGER__OBJECT_MANAGER_HPP_
