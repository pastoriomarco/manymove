#include "manymove_object_manager/object_manager.hpp"
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <thread>
#include <chrono>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

namespace manymove_object_manager
{

    ObjectManagerNode::ObjectManagerNode()
        : Node("object_manager_node")
    {
        // Declare and get the frame_id parameter
        this->declare_parameter<std::string>("frame_id", "world");
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Initialize publisher
        collision_object_publisher_ = this->create_publisher<moveit_msgs::msg::CollisionObject>(
            "/collision_object", 10);

        // Publisher for attached collision objects
        attached_collision_object_publisher_ = this->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
            "/attached_collision_object", 10);

        // Create a Reentrant callback group for the service client
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        get_planning_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/get_planning_scene", rmw_qos_profile_services_default, service_callback_group_);

        RCLCPP_INFO(this->get_logger(), "Waiting for /get_planning_scene service...");
        if (!get_planning_scene_client_->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "/get_planning_scene service not available. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "/get_planning_scene service is available.");

        // Call the function to print the current planning scene
        printPlanningScene();

        // Create a separate callback group for action servers
        action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Configure action server options
        auto serverOptions = rcl_action_server_get_default_options();
        serverOptions.result_service_qos = rmw_qos_profile_services_default;

        // Initialize action servers
        add_object_action_server_ = rclcpp_action::create_server<AddCollisionObject>(
            this,
            "add_collision_object",
            std::bind(&ObjectManagerNode::handleAddGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ObjectManagerNode::handleAddCancel, this, std::placeholders::_1),
            std::bind(&ObjectManagerNode::handleAddExecute, this, std::placeholders::_1),
            serverOptions,
            action_callback_group_);

        remove_object_action_server_ = rclcpp_action::create_server<RemoveCollisionObject>(
            this,
            "remove_collision_object",
            std::bind(&ObjectManagerNode::handleRemoveGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ObjectManagerNode::handleRemoveCancel, this, std::placeholders::_1),
            std::bind(&ObjectManagerNode::handleRemoveExecute, this, std::placeholders::_1),
            serverOptions,
            action_callback_group_);

        // Initialize CheckObjectExists action server (New Action Server)
        check_object_exists_action_server_ = rclcpp_action::create_server<CheckObjectExists>(
            this,
            "check_object_exists",
            std::bind(&ObjectManagerNode::handleCheckExistGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ObjectManagerNode::handleCheckExistCancel, this, std::placeholders::_1),
            std::bind(&ObjectManagerNode::handleCheckExistExecute, this, std::placeholders::_1),
            serverOptions,
            action_callback_group_);

        attach_detach_object_action_server_ = rclcpp_action::create_server<AttachDetachObject>(
            this,
            "attach_detach_object",
            std::bind(&ObjectManagerNode::handleAttachDetachGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ObjectManagerNode::handleAttachDetachCancel, this, std::placeholders::_1),
            std::bind(&ObjectManagerNode::handleAttachDetachExecute, this, std::placeholders::_1),
            serverOptions,
            action_callback_group_);

        get_object_pose_action_server_ = rclcpp_action::create_server<GetObjectPose>(
            this,
            "get_object_pose",
            std::bind(&ObjectManagerNode::handleGetObjectPoseGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ObjectManagerNode::handleGetObjectPoseCancel, this, std::placeholders::_1),
            std::bind(&ObjectManagerNode::handleGetObjectPoseExecute, this, std::placeholders::_1),
            serverOptions,
            action_callback_group_);

        RCLCPP_INFO(this->get_logger(),
                    "Object Manager Node initialized with frame_id: %s.", frame_id_.c_str());
    }

    // ----------------------------------------------------------------------------
    // Action Server Callbacks: CheckObjectExists
    // ----------------------------------------------------------------------------
    rclcpp_action::GoalResponse ObjectManagerNode::handleCheckExistGoal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const CheckObjectExists::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received request to check existence of object: %s",
                    goal->object_id.c_str());

        // Always accept the goal as it's a simple check
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ObjectManagerNode::handleCheckExistCancel(
        const std::shared_ptr<CheckGoalHandle> goal_handle)
    {
        (void)goal_handle; // Suppress unused parameter warning
        RCLCPP_INFO(this->get_logger(), "CheckExist operation canceled.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ObjectManagerNode::handleCheckExistExecute(
        const std::shared_ptr<CheckGoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<CheckObjectExists::Result>();

        // Check if the object exists
        bool collision_object_exists = objectExists(goal->object_id);
        bool attached_object_exists = attachedObjectExists(goal->object_id);
        bool exists = (collision_object_exists || attached_object_exists);

        result->exists = exists;
        result->is_attached = attached_object_exists;

        if (attached_object_exists)
        {
            result->link_name = getAttachedObjectLinkById(goal->object_id)->c_str();

            std::ostringstream result_message;
            result_message << "Object exists in the planning scene, attached to '" << result->link_name.c_str() << "'.";

            result->message = (result_message.str());

            RCLCPP_INFO(this->get_logger(),
                        "CheckExist: Object '%s' exists attached to link '%s'.",
                        goal->object_id.c_str(),
                        result->link_name.c_str());
        }
        else
        {
            result->link_name = "";
            result->message = exists ? "Object exists in the planning scene."
                                     : "Object not found in the planning scene.";

            RCLCPP_INFO(this->get_logger(),
                        "CheckExist: Object '%s' %s in the planning scene.",
                        goal->object_id.c_str(),
                        exists ? "exists" : "does NOT exist");
        }

        // Set the result and succeed the goal
        goal_handle->succeed(result);
    }

    void ObjectManagerNode::printPlanningScene()
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                         moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

        RCLCPP_INFO(this->get_logger(), "Requesting Planning Scene with components: %d", request->components.components);

        auto future_response = get_planning_scene_client_->async_send_request(request);

        // Spin the node manually to process service callbacks
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(5);

        while (std::chrono::steady_clock::now() - start_time < timeout)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            if (future_response.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
            {
                break;
            }
        }

        // Handle the service response
        if (future_response.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        {
            auto response = future_response.get();
            if (response)
            {
                RCLCPP_INFO(this->get_logger(), "Received Planning Scene:");
                const auto &collision_objects = response->scene.world.collision_objects;

                if (collision_objects.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "  No collision objects found in the planning scene.");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "  Found %zu collision object(s):", collision_objects.size());
                    for (const auto &obj : collision_objects)
                    {
                        RCLCPP_INFO(this->get_logger(), "    - ID: '%s'", obj.id.c_str());
                    }
                }

                const auto &attached_collision_object = response->scene.robot_state.attached_collision_objects;

                if (attached_collision_object.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "  No attached collision objects found in the planning scene.");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "  Found %zu attached collision object(s):", attached_collision_object.size());
                    for (const auto &attached_obj : attached_collision_object)
                    {
                        RCLCPP_INFO(this->get_logger(), "    - ID: '%s'", attached_obj.object.id.c_str());
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Received null response from /get_planning_scene.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for /get_planning_scene response.");
        }
    }

    // ----------------------------------------------------------------------------
    // Action Server Callbacks: AddCollisionObject
    // ----------------------------------------------------------------------------
    rclcpp_action::GoalResponse ObjectManagerNode::handleAddGoal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const AddCollisionObject::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to add object: %s", goal->id.c_str());

        if (!validateShapeAndDimensions(goal->shape, goal->dimensions, goal->mesh_file))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid shape/dimensions or missing mesh file for object.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        else if (objectExists(goal->id))
        {
            RCLCPP_WARN(this->get_logger(), "Object already in the planning scene.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        else if (attachedObjectExists(goal->id))
        {
            RCLCPP_WARN(this->get_logger(), "Object already in the planning scene, attached to %s.", getAttachedObjectLinkById(goal->id)->c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ObjectManagerNode::handleAddCancel(
        [[maybe_unused]] const std::shared_ptr<AddGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Add operation canceled.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ObjectManagerNode::handleAddExecute(
        const std::shared_ptr<AddGoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<AddCollisionObject::Result>();

        // Publish the collision object
        auto collision_object = createCollisionObject(goal->id, goal->shape, goal->dimensions, goal->pose, goal->mesh_file, goal->scale_mesh);
        collision_object_publisher_->publish(collision_object);
        RCLCPP_INFO(this->get_logger(),
                    "Published ADD operation for object '%s'.", goal->id.c_str());

        // Publish feedback
        auto feedback = std::make_shared<AddCollisionObject::Feedback>();
        feedback->status = "Collision object published.";
        goal_handle->publish_feedback(feedback);

        // Retry mechanism
        const int max_retries = 5;
        const int delay_ms = 50;
        bool exists = false;

        for (int attempt = 1; attempt <= max_retries; ++attempt)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(delay_ms * attempt));

            // Check if the object exists
            exists = objectExists(goal->id);
            if (exists)
            {
                result->success = true;
                result->message = "Object added successfully.";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(),
                            "Object '%s' added successfully after %d attempt(s).", goal->id.c_str(), attempt);
                return;
            }

            RCLCPP_WARN(this->get_logger(),
                        "Attempt %d: Object '%s' not found in planning scene yet.", attempt, goal->id.c_str());
        }

        // If we reach here, the object was not found
        result->success = false;
        result->message = "Failed to verify addition of object.";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to verify addition of object '%s' after %d attempts.", goal->id.c_str(), max_retries);
    }

    // ----------------------------------------------------------------------------
    // Action Server Callbacks: RemoveCollisionObject
    // ----------------------------------------------------------------------------
    rclcpp_action::GoalResponse ObjectManagerNode::handleRemoveGoal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const RemoveCollisionObject::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to remove object: %s", goal->id.c_str());

        // Check if the object is attached before trying to remove
        if (attachedObjectExists(goal->id))
        {
            RCLCPP_WARN(this->get_logger(), "Object '%s' is attached to link '%s', detach it before removing it.",
                        goal->id.c_str(),
                        getAttachedObjectLinkById(goal->id)->c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check if the object exists before trying to remove
        if ((!objectExists(goal->id)) && (!attachedObjectExists(goal->id)))
        {
            RCLCPP_WARN(this->get_logger(), "Object '%s' does not exist.", goal->id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ObjectManagerNode::handleRemoveCancel(
        const std::shared_ptr<RemoveGoalHandle> goal_handle)
    {
        (void)goal_handle; // Suppress unused parameter warning
        RCLCPP_INFO(this->get_logger(), "Remove operation canceled.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ObjectManagerNode::handleRemoveExecute(
        const std::shared_ptr<RemoveGoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<RemoveCollisionObject::Result>();

        // Create a CollisionObject message with REMOVE
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = goal->id;
        collision_object.header.frame_id = frame_id_;
        collision_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        // Publish the remove operation
        collision_object_publisher_->publish(collision_object);
        RCLCPP_INFO(this->get_logger(),
                    "Published REMOVE operation for object '%s'.", goal->id.c_str());

        // Publish feedback
        auto feedback = std::make_shared<RemoveCollisionObject::Feedback>();
        feedback->status = "Collision object remove command published.";
        goal_handle->publish_feedback(feedback);

        // Retry mechanism
        const int max_retries = 5;
        const int initial_delay_ms = 50;
        int delay_ms = initial_delay_ms;
        bool exists = true;

        for (int attempt = 1; attempt <= max_retries; ++attempt)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(delay_ms));

            // Check if the object still exists
            exists = objectExists(goal->id);
            if (!exists)
            {
                result->success = true;
                result->message = "Object removed successfully.";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(),
                            "Object '%s' removed successfully after %d attempt(s).", goal->id.c_str(), attempt);
                return;
            }

            RCLCPP_WARN(this->get_logger(),
                        "Attempt %d: Object '%s' still exists in planning scene.", attempt, goal->id.c_str());

            // Exponential backoff
            delay_ms *= 2;
        }

        // If we reach here, the object was not removed
        result->success = false;
        result->message = "Failed to verify removal of object.";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to verify removal of object '%s' after %d attempts.", goal->id.c_str(), max_retries);
    }

    // ----------------------------------------------------------------------------
    // Action Server Callbacks: AttachDetachObject
    // ----------------------------------------------------------------------------
    rclcpp_action::GoalResponse ObjectManagerNode::handleAttachDetachGoal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const AttachDetachObject::Goal> goal)
    {
        // I call once the search functions to avoid dalays
        auto attached_object_exists = attachedObjectExists(goal->object_id);
        auto collision_object_exists = objectExists(goal->object_id);

        RCLCPP_INFO(this->get_logger(),
                    "Received request to %s object: %s to link: %s",
                    goal->attach ? "attach" : "detach",
                    goal->object_id.c_str(),
                    goal->link_name.c_str());

        // Basic validation
        if (goal->object_id.empty() || (goal->attach && goal->link_name.empty()))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid object_id or link_name in the request.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check if object exists at all, if not it wouldn't make sense to continue either for attaching or detaching
        if ((!attached_object_exists && !collision_object_exists))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Object '%s' does not exist in the planning scene.",
                        goal->object_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check if the object exists for attach operation
        if (goal->attach && (attached_object_exists))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Cannot attach. Object '%s' already attached to link %s.",
                        goal->object_id.c_str(),
                        getAttachedObjectLinkById(goal->object_id)->c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check if the object is already attached somewhere
        if (goal->attach && (!collision_object_exists || attached_object_exists))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Cannot attach. Object '%s' already attached .",
                        goal->object_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check if the object is attached for detach operation
        if (!goal->attach && !attached_object_exists)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Cannot detach. Object '%s' is not attached to link '%s'.",
                        goal->object_id.c_str(),
                        goal->link_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ObjectManagerNode::handleAttachDetachCancel(
        [[maybe_unused]] const std::shared_ptr<AttachDetachGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "AttachDetach operation canceled.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ObjectManagerNode::handleAttachDetachExecute(
        const std::shared_ptr<AttachDetachGoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<AttachDetachObject::Result>();

        // Create a CollisionObject message for attaching/detaching
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.object.id = goal->object_id;
        attached_object.link_name = goal->link_name;
        attached_object.object.header.frame_id = frame_id_;

        std::optional<moveit_msgs::msg::CollisionObject> collision_object_opt;

        if (goal->attach)
        {
            attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
            attached_object.detach_posture = trajectory_msgs::msg::JointTrajectory();
         
            if (!goal->touch_links.empty())
            {
                attached_object.touch_links = goal->touch_links;
            }
        }
        else
        {
            attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

            // Retrieve object data from planning scene
            collision_object_opt = getObjectDataById(goal->object_id);
            if (!collision_object_opt)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to retrieve object '%s' data.",
                             goal->object_id.c_str());

                result->success = false;
                result->message = goal->attach ? "Failed to attach object." : "Failed to detach object.";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(),
                             "%s operation failed for object '%s' and link '%s'.",
                             goal->attach ? "Attach" : "Detach",
                             goal->object_id.c_str(),
                             goal->link_name.c_str());
                return;
            }
        }

        // Publish the attach/detach command
        attached_collision_object_publisher_->publish(attached_object);
        RCLCPP_INFO(this->get_logger(),
                    "%s operation published for object '%s' to link '%s'.",
                    goal->attach ? "Attach" : "Detach",
                    goal->object_id.c_str(),
                    goal->link_name.c_str());

        // Publish feedback
        auto feedback = std::make_shared<AttachDetachObject::Feedback>();
        feedback->status = goal->attach ? "Attach command published." : "Detach command published.";
        goal_handle->publish_feedback(feedback);

        // Retry mechanism to verify the operation
        const int max_retries = 5;
        const int delay_ms = 50;
        bool operation_successful = false;

        for (int attempt = 1; attempt <= max_retries; ++attempt)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(delay_ms * attempt));

            // Check the planning scene
            auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
            request->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

            auto future_response = get_planning_scene_client_->async_send_request(request);

            // Wait for the response with timeout
            if (future_response.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
            {
                auto response = future_response.get();
                if (response)
                {
                    bool found = attachedObjectExists(goal->object_id);

                    if (goal->attach && found)
                    {
                        RCLCPP_INFO(this->get_logger(),
                                    "Object '%s' successfully attached to link '%s'.",
                                    goal->object_id.c_str(),
                                    goal->link_name.c_str());
                        operation_successful = true;
                        break;
                    }
                    else if (!goal->attach && !found)
                    {
                        RCLCPP_INFO(this->get_logger(),
                                    "Object '%s' successfully detached from link '%s'.",
                                    goal->object_id.c_str(),
                                    goal->link_name.c_str());

                        // If detaching, ensure the object is back in the planning scene
                        if (!objectExists(goal->object_id))
                        {
                            RCLCPP_INFO(this->get_logger(),
                                        "Object '%s' not found in the planning scene after detaching. Adding it back.",
                                        goal->object_id.c_str());

                            const auto &collision_object = collision_object_opt.value();

                            // Publisher for collision objects
                            collision_object_publisher_->publish(collision_object);
                            RCLCPP_INFO(this->get_logger(),
                                        "Re-added object '%s' to the planning scene.", goal->object_id.c_str());

                            // Wait briefly to allow the planning scene to update
                            rclcpp::sleep_for(std::chrono::milliseconds(500));

                            // Verify the object is added
                            if (objectExists(goal->object_id))
                            {
                                RCLCPP_INFO(this->get_logger(),
                                            "Object '%s' re-added to the planning scene successfully.",
                                            goal->object_id.c_str());
                                operation_successful = true;
                                break;
                            }
                            else
                            {
                                RCLCPP_WARN(this->get_logger(),
                                            "Failed to re-add object '%s' to the planning scene.",
                                            goal->object_id.c_str());
                            }
                        }
                        else
                        {
                            operation_successful = true;
                            break;
                        }
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Received null response from /get_planning_scene.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for /get_planning_scene response.");
            }

            RCLCPP_WARN(this->get_logger(),
                        "Attempt %d: Operation not confirmed yet.", attempt);
        }

        if (operation_successful)
        {
            result->success = true;
            result->message = goal->attach ? "Object attached successfully." : "Object detached successfully.";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(),
                        "%s operation completed successfully for object '%s' and link '%s'.",
                        goal->attach ? "Attach" : "Detach",
                        goal->object_id.c_str(),
                        goal->link_name.c_str());
        }
        else
        {
            result->success = false;
            result->message = goal->attach ? "Failed to attach object." : "Failed to detach object.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(),
                         "%s operation failed for object '%s' and link '%s'.",
                         goal->attach ? "Attach" : "Detach",
                         goal->object_id.c_str(),
                         goal->link_name.c_str());
        }
    }

    rclcpp_action::GoalResponse ObjectManagerNode::handleGetObjectPoseGoal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const GetObjectPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received GetObjectPose goal for object: %s, reference link: %s", goal->object_id.c_str(), goal->link_name.c_str());

        if (goal->object_id.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Invalid object_id: cannot be empty.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ObjectManagerNode::handleGetObjectPoseCancel(
        [[maybe_unused]] const std::shared_ptr<GetObjectPoseGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "GetObjectPose operation canceled.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ObjectManagerNode::handleGetObjectPoseExecute(
        const std::shared_ptr<GetObjectPoseGoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<GetObjectPose::Result>();

        // 1. Retrieve the CollisionObject from the planning scene
        auto collision_object_opt = getObjectDataById(goal->object_id);
        if (!collision_object_opt)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to retrieve object '%s' from planning scene.",
                         goal->object_id.c_str());
            result->success = false;
            result->message = "Object not found in planning scene.";
            goal_handle->abort(result);
            return;
        }

        // 2. The object's pose is stored in either "world" (frame_id_) or
        //    the attached link's frame if attached
        geometry_msgs::msg::Pose object_pose = collision_object_opt->pose;

        bool is_attached = attachedObjectExists(goal->object_id);
        std::string object_frame = frame_id_; // assume 'world' by default
        if (is_attached)
        {
            auto maybe_link = getAttachedObjectLinkById(goal->object_id);
            if (!maybe_link)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to get attached link for object '%s'.",
                             goal->object_id.c_str());
                result->success = false;
                result->message = "Failed to determine attached link.";
                goal_handle->abort(result);
                return;
            }
            // If attached, the object_pose is in this link's frame
            object_frame = maybe_link.value();
        }

        // 3. If needed, transform from 'object_frame' => 'goal->link_name'
        //    only if the user specifies a different link_name
        if (!goal->link_name.empty() && goal->link_name != object_frame)
        {
            tf2_ros::Buffer tfBuffer(this->get_clock());
            tf2_ros::TransformListener tfListener(tfBuffer, this, true);

            try
            {
                geometry_msgs::msg::TransformStamped transformStamped =
                    tfBuffer.lookupTransform(
                        goal->link_name, // target frame
                        object_frame,    // source frame
                        tf2::TimePointZero,
                        tf2::durationFromSec(1.0));

                // Apply that transform
                geometry_msgs::msg::Pose transformed_pose;
                tf2::doTransform(object_pose, transformed_pose, transformStamped);
                object_pose = transformed_pose;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to lookup transform from '%s' to '%s': %s",
                             object_frame.c_str(), goal->link_name.c_str(), ex.what());
                result->success = false;
                result->message = "Transform error.";
                goal_handle->abort(result);
                return;
            }
        }

        // 4. Now we apply the pre-/post- transformations to the pose in the final frame

        try
        {
            // Validate arrays
            if (goal->pre_transform_xyz_rpy.size() != 6)
            {
                throw std::invalid_argument(
                    "pre_transform_xyz_rpy must have exactly 6 elements (x,y,z,roll,pitch,yaw).");
            }
            if (goal->post_transform_xyz_rpy.size() != 6)
            {
                throw std::invalid_argument(
                    "post_transform_xyz_rpy must have exactly 6 elements (x,y,z,roll,pitch,yaw).");
            }

            // Steps 1..8 from the original snippet
            // ------------------------------------
            // Step 1: Start with identity
            tf2::Transform combined_transform;
            combined_transform.setIdentity();

            // Step 2: Rotation from pre_transform_xyz_rpy (roll, pitch, yaw)
            tf2::Quaternion pre_rot;
            pre_rot.setRPY(goal->pre_transform_xyz_rpy[3],
                           goal->pre_transform_xyz_rpy[4],
                           goal->pre_transform_xyz_rpy[5]);
            tf2::Transform pre_rot_tf;
            pre_rot_tf.setRotation(pre_rot);
            pre_rot_tf.setOrigin(tf2::Vector3(0, 0, 0));
            combined_transform = pre_rot_tf * combined_transform;

            // Step 3: Translation from pre_transform_xyz_rpy (x, y, z)
            tf2::Vector3 pre_trans(goal->pre_transform_xyz_rpy[0],
                                   goal->pre_transform_xyz_rpy[1],
                                   goal->pre_transform_xyz_rpy[2]);
            tf2::Transform pre_trans_tf;
            pre_trans_tf.setIdentity();
            pre_trans_tf.setOrigin(pre_trans);
            combined_transform = pre_trans_tf * combined_transform;

            // Step 4: Rotation from post_transform_xyz_rpy (roll, pitch, yaw)
            tf2::Quaternion post_rot;
            post_rot.setRPY(goal->post_transform_xyz_rpy[3],
                            goal->post_transform_xyz_rpy[4],
                            goal->post_transform_xyz_rpy[5]);
            tf2::Transform post_rot_tf;
            post_rot_tf.setRotation(post_rot);
            post_rot_tf.setOrigin(tf2::Vector3(0, 0, 0));
            combined_transform = post_rot_tf * combined_transform;

            // Step 7: Translation from post_transform_xyz_rpy (x, y, z)
            tf2::Vector3 post_trans(goal->post_transform_xyz_rpy[0],
                                    goal->post_transform_xyz_rpy[1],
                                    goal->post_transform_xyz_rpy[2]);
            tf2::Transform post_trans_tf;
            post_trans_tf.setIdentity();
            post_trans_tf.setOrigin(post_trans);
            combined_transform = post_trans_tf * combined_transform;

            // Step 5: Rotation from the object's original orientation
            tf2::Quaternion obj_quat(
                object_pose.orientation.x,
                object_pose.orientation.y,
                object_pose.orientation.z,
                object_pose.orientation.w);
            tf2::Transform obj_rot_tf;
            obj_rot_tf.setRotation(obj_quat);
            obj_rot_tf.setOrigin(tf2::Vector3(0, 0, 0));
            combined_transform = obj_rot_tf * combined_transform;

            // Step 6: Translation from the object's original position
            tf2::Vector3 obj_trans(
                object_pose.position.x,
                object_pose.position.y,
                object_pose.position.z);
            tf2::Transform obj_trans_tf;
            obj_trans_tf.setIdentity();
            obj_trans_tf.setOrigin(obj_trans);
            combined_transform = obj_trans_tf * combined_transform;

            // Step 8: Extract final pose
            geometry_msgs::msg::Pose final_pose;
            final_pose.position.x = combined_transform.getOrigin().x();
            final_pose.position.y = combined_transform.getOrigin().y();
            final_pose.position.z = combined_transform.getOrigin().z();

            tf2::Quaternion final_quat = combined_transform.getRotation();
            final_quat.normalize();
            final_pose.orientation.x = final_quat.x();
            final_pose.orientation.y = final_quat.y();
            final_pose.orientation.z = final_quat.z();
            final_pose.orientation.w = final_quat.w();

            // 5. Set and return result
            result->pose = final_pose;
            result->success = true;
            result->message = "Pose updated successfully with updated transformation logic.";
            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(),
                        "Object '%s' updated successfully. Final pose: "
                        "position (%.3f, %.3f, %.3f), orientation (%.3f, %.3f, %.3f, %.3f)",
                        goal->object_id.c_str(),
                        final_pose.position.x, final_pose.position.y, final_pose.position.z,
                        final_pose.orientation.x, final_pose.orientation.y,
                        final_pose.orientation.z, final_pose.orientation.w);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            result->success = false;
            result->message = e.what();
            goal_handle->abort(result);
        }
    }

    // ----------------------------------------------------------------------------
    // Utility Function: objectExists
    // ----------------------------------------------------------------------------
    bool ObjectManagerNode::objectExists(const std::string &id)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;

        RCLCPP_INFO(this->get_logger(), "Checking if object '%s' exists in the planning scene...", id.c_str());

        auto future_response = get_planning_scene_client_->async_send_request(request);

        // Wait for the future to complete without additional spinning
        if (future_response.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            auto response = future_response.get();
            if (response)
            {
                for (const auto &obj : response->scene.world.collision_objects)
                {
                    if (obj.id == id)
                    {
                        RCLCPP_INFO(this->get_logger(), "Object '%s' found in the planning scene.", id.c_str());
                        return true;
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Received null response from /get_planning_scene.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for /get_planning_scene response.");
        }

        RCLCPP_INFO(this->get_logger(), "Object '%s' not found in the planning scene.", id.c_str());
        return false;
    }

    // ----------------------------------------------------------------------------
    // Utility Function: objectExists
    // ----------------------------------------------------------------------------
    bool ObjectManagerNode::attachedObjectExists(const std::string &id, const std::string &link_name)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

        if (link_name.size() > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Checking if object '%s' is attached to link '%s'...", id.c_str(), link_name.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Checking if object '%s' is attached to any link...", id.c_str());
        }

        auto future_response = get_planning_scene_client_->async_send_request(request);

        if (future_response.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            auto response = future_response.get();
            if (response)
            {
                for (const auto &attached_obj : response->scene.robot_state.attached_collision_objects)
                {
                    if (attached_obj.object.id == id)
                    {
                        if (link_name.empty() || attached_obj.link_name == link_name)
                        {
                            RCLCPP_INFO(this->get_logger(), "Object '%s' is attached to link '%s'.", id.c_str(), attached_obj.link_name.c_str());
                            return true;
                        }
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Received null response from /get_planning_scene.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for /get_planning_scene response.");
        }

        RCLCPP_WARN(this->get_logger(), "Object '%s' is not attached to link '%s'.", id.c_str(), link_name.empty() ? "any link" : link_name.c_str());
        return false;
    }

    // ----------------------------------------------------------------------------
    // Utility Function: getObjectDataById
    // ----------------------------------------------------------------------------
    std::optional<moveit_msgs::msg::CollisionObject> ObjectManagerNode::getObjectDataById(const std::string &object_id)
    {
        // Request planning scene world data
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                         moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

        auto future_response = get_planning_scene_client_->async_send_request(request);

        // Wait for the service response
        if (future_response.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            auto response = future_response.get();
            if (response)
            {
                for (const auto &collision_object : response->scene.world.collision_objects)
                {
                    if (collision_object.id == object_id)
                    {
                        // Object found, return it
                        RCLCPP_INFO(this->get_logger(), "Object '%s' data retrieved from planning scene.", object_id.c_str());
                        return collision_object;
                    }
                }
                for (const auto &attached_collision_object : response->scene.robot_state.attached_collision_objects)
                {
                    if (attached_collision_object.object.id == object_id)
                    {
                        // Object found, return it
                        RCLCPP_INFO(this->get_logger(), "Object '%s' data retrieved from attached objects in planning scene.", object_id.c_str());
                        return attached_collision_object.object;
                    }
                }
                RCLCPP_WARN(this->get_logger(), "Object '%s' not found in planning scene.", object_id.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Null response received from /get_planning_scene.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for /get_planning_scene response.");
        }

        // Return empty if not found
        return std::nullopt;
    }

    // ----------------------------------------------------------------------------
    // Utility Function: getAttachedObjectLinkById
    // ----------------------------------------------------------------------------
    std::optional<std::string> ObjectManagerNode::getAttachedObjectLinkById(const std::string &object_id)
    {
        // Request planning scene world data
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

        auto future_response = get_planning_scene_client_->async_send_request(request);

        // Wait for the service response
        if (future_response.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            auto response = future_response.get();
            if (response)
            {
                for (const auto &attached_collision_object : response->scene.robot_state.attached_collision_objects)
                {
                    if (attached_collision_object.object.id == object_id)
                    {
                        // Object found, return it
                        RCLCPP_INFO(this->get_logger(), "Object '%s' link retrieved from attached objects in planning scene.", object_id.c_str());
                        return attached_collision_object.link_name;
                    }
                }
                RCLCPP_WARN(this->get_logger(), "Link '%s' not found in planning scene.", object_id.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Null response received from /get_planning_scene.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for /get_planning_scene response.");
        }

        // Return empty if not found
        return std::nullopt;
    }

    // ----------------------------------------------------------------------------
    // Utility Function: createCollisionObject
    // ----------------------------------------------------------------------------
    moveit_msgs::msg::CollisionObject ObjectManagerNode::createCollisionObject(
        const std::string &id,
        const std::string &shape,
        const std::vector<double> &dimensions,
        const geometry_msgs::msg::Pose &pose,
        const std::string &mesh_file,
        const std::vector<double> scale_mesh) const
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id_;
        collision_object.id = id;

        if (shape == "mesh")
        {
            // Attempt to load the mesh from the specified resource
            shapes::Mesh *mesh = shapes::createMeshFromResource(mesh_file);
            if (!mesh)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from file: %s", mesh_file.c_str());
                return collision_object; // Exit early if mesh loading fails
            }

            try
            {
                // Apply scaling to the mesh vertices
                for (unsigned int i = 0; i < mesh->vertex_count; ++i)
                {
                    mesh->vertices[3 * i + 0] *= scale_mesh[0]; // Scale X
                    mesh->vertices[3 * i + 1] *= scale_mesh[1]; // Scale Y
                    mesh->vertices[3 * i + 2] *= scale_mesh[2]; // Scale Z
                }
                // Convert the scaled mesh into a shape_msgs::Mesh
                shape_msgs::msg::Mesh mesh_msg;
                shapes::ShapeMsg shape_msg;

                // Convert the shapes::Mesh into a ShapeMsg (a Boost variant)
                shapes::constructMsgFromShape(mesh, shape_msg);

                // Extract the shape_msgs::msg::Mesh from the ShapeMsg
                if (auto mesh_ptr = boost::get<shape_msgs::msg::Mesh>(&shape_msg))
                {
                    // Successfully extracted the mesh
                    mesh_msg = *mesh_ptr;
                    collision_object.meshes.push_back(mesh_msg);
                    collision_object.mesh_poses.push_back(pose);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to extract mesh from ShapeMsg for object '%s'.", id.c_str());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception while processing mesh: %s", e.what());
            }

            // Clean up dynamically allocated mesh to prevent memory leaks
            delete mesh;
        }
        else
        {
            shape_msgs::msg::SolidPrimitive primitive;
            if (shape == "box")
            {
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                if (dimensions.size() >= 3)
                {
                    primitive.dimensions[0] = dimensions[0];
                    primitive.dimensions[1] = dimensions[1];
                    primitive.dimensions[2] = dimensions[2];
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Insufficient dimensions for box. Using default size.");
                    primitive.dimensions[0] = 0.1;
                    primitive.dimensions[1] = 0.1;
                    primitive.dimensions[2] = 0.1;
                }
            }
            else if (shape == "cylinder")
            {
                primitive.type = primitive.CYLINDER;
                primitive.dimensions.resize(2);
                if (dimensions.size() >= 2)
                {
                    primitive.dimensions[0] = dimensions[0]; // height
                    primitive.dimensions[1] = dimensions[1]; // radius
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Insufficient dimensions for cylinder. Using default size.");
                    primitive.dimensions[0] = 0.1;
                    primitive.dimensions[1] = 0.05;
                }
            }
            else if (shape == "sphere")
            {
                primitive.type = primitive.SPHERE;
                primitive.dimensions.resize(1);
                if (dimensions.size() >= 1)
                {
                    primitive.dimensions[0] = dimensions[0]; // radius
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Insufficient dimensions for sphere. Using default size.");
                    primitive.dimensions[0] = 0.05;
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "Unsupported shape type: %s. Defaulting to box.", shape.c_str());
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[0] = 0.1;
                primitive.dimensions[1] = 0.1;
                primitive.dimensions[2] = 0.1;
            }

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);
            collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        }

        return collision_object;
    }

    // ----------------------------------------------------------------------------
    // Utility Function: validateShapeAndDimensions
    // ----------------------------------------------------------------------------
    bool ObjectManagerNode::validateShapeAndDimensions(
        const std::string &shape,
        const std::vector<double> &dimensions,
        const std::optional<std::string> &mesh_file) const
    {
        if (shape == "box" && dimensions.size() == 3)
            return true;
        if (shape == "cylinder" && dimensions.size() == 2)
            return true;
        if (shape == "sphere" && dimensions.size() == 1)
            return true;
        if (shape == "mesh" && mesh_file.has_value())
            return true;

        RCLCPP_WARN(this->get_logger(),
                    "Invalid shape '%s' or missing required dimensions/mesh file.", shape.c_str());
        RCLCPP_WARN(this->get_logger(),
                    "Dimensions size '%li'.", dimensions.size());
        return false;
    }

} // namespace manymove_object_manager

// ----------------------------------------------------------------------------
// Main Function
// ----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<manymove_object_manager::ObjectManagerNode>();

    // Use a MultiThreadedExecutor with multiple threads
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // 4 threads
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
