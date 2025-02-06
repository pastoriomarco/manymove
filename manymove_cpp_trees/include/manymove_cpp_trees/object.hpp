#ifndef MANYMOVE_CPP_TREES_OBJECT_HPP
#define MANYMOVE_CPP_TREES_OBJECT_HPP

#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace manymove_cpp_trees
{
    /**
     * @enum ObjectActionType
     * @brief Enumerates the types of actions that can be performed on objects.
     */
    enum class ObjectActionType
    {
        ADD,     ///< Add a new object to the planning scene.
        REMOVE,  ///< Remove an existing object from the planning scene.
        ATTACH,  ///< Attach an object to a specific robot link.
        DETACH,  ///< Detach an object from a specific robot link.
        CHECK,   ///< Check if an object exists and its attachment status.
        GET_POSE ///< Retrieve and possibly modify the pose of an object.
    };

    /**
     * @struct ObjectAction
     * @brief Represents an action to be performed on a collision object.
     */
    struct ObjectAction
    {
        ObjectActionType type; ///< The type of object action.
        std::string object_id; ///< Unique identifier for the object.

        // Parameters for ADD action
        std::string shape;              ///< Shape type (e.g., box, mesh).
        std::vector<double> dimensions; ///< Dimensions for primitive shapes (e.g., width, height, depth).
        geometry_msgs::msg::Pose pose;  ///< Pose of the object in the planning scene.
        std::string mesh_file;          ///< Path to the mesh file (required if shape is mesh).
        double scale_mesh_x = 1.0;      ///< Scale factor along the X-axis for mesh objects.
        double scale_mesh_y = 1.0;      ///< Scale factor along the Y-axis for mesh objects.
        double scale_mesh_z = 1.0;      ///< Scale factor along the Z-axis for mesh objects.

        // Parameters for ATTACH and DETACH actions
        std::string link_name; ///< Name of the robot link to attach/detach the object.
        bool attach = true;    ///< Flag indicating whether to attach (true) or detach (false) the object.

        // Parameters for GET_POSE action
        std::vector<double> pre_transform_xyz_rpy;         ///< Linear transform in x, y and z and rotation in roll, pitch, yaw of the pose of the object
        std::vector<double> post_transform_xyz_rpy; ///< Reference orientation for the pose transform of the pose
        std::string pose_key;                          ///< Blackboard key to store the retrieved pose (used only for GET_POSE).

        /**
         * @brief Default constructor.
         */
        ObjectAction() = default;

        /**
         * @brief Parameterized constructor for ADD action.
         * @param obj_id Unique identifier for the object.
         * @param shp Shape type.
         * @param dims Dimensions for the shape.
         * @param ps Pose of the object.
         * @param mesh Path to mesh file.
         * @param sx Scale factor X.
         * @param sy Scale factor Y.
         * @param sz Scale factor Z.
         */
        ObjectAction(const std::string &obj_id, const std::string &shp,
                     const std::vector<double> &dims, const geometry_msgs::msg::Pose &ps,
                     const std::string &mesh = "", double sx = 1.0, double sy = 1.0, double sz = 1.0)
            : type(ObjectActionType::ADD), object_id(obj_id), shape(shp),
              dimensions(dims), pose(ps), mesh_file(mesh),
              scale_mesh_x(sx), scale_mesh_y(sy), scale_mesh_z(sz) {}

        /**
         * @brief Parameterized constructor for ATTACH/DETACH action.
         * @param obj_id Unique identifier for the object.
         * @param link Link name.
         * @param att Attach flag (true for attach, false for detach).
         */
        ObjectAction(const std::string &obj_id, const std::string &link, bool att)
            : type(att ? ObjectActionType::ATTACH : ObjectActionType::DETACH),
              object_id(obj_id), link_name(link), attach(att) {}

        /**
         * @brief Parameterized constructor for CHECK action.
         * @param obj_id Unique identifier for the object.
         */
        ObjectAction(const std::string &obj_id)
            : type(ObjectActionType::CHECK), object_id(obj_id) {}

        /**
         * @brief Parameterized constructor for GET_POSE action.
         * @param obj_id Unique identifier for the object.
         * @param xyz_rpy Offset values in format {x, y, z, roll, pitch, yaw}.
         * @param rpy Reference orientation in format {roll, pitch, yaw}.
         * @param key Blackboard key to store the retrieved pose.
         */
        ObjectAction(const std::string &obj_id, const std::string &key, const std::vector<double> &pre_xyz_rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, const std::vector<double> &post_xyz_rpy = {0.0, 0.0, 0.0})
            : type(ObjectActionType::GET_POSE), object_id(obj_id),
              pose_key(key),
              pre_transform_xyz_rpy(pre_xyz_rpy), post_transform_xyz_rpy(post_xyz_rpy) {}
    };

    /**
     * @brief Helper function to create an ObjectAction for adding a box-shaped object.
     * @param object_id Unique identifier for the object.
     * @param dimensions Dimensions of the box (width, height, depth).
     * @param pose Pose of the object.
     * @param object_type Type of primitive object, possible values: box, cylinder, sphere.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createAddPrimitiveObject(const std::string &object_id,
                                                 const std::string &object_type,
                                                 const std::vector<double> &dimensions,
                                                 const geometry_msgs::msg::Pose &pose)
    {
        return ObjectAction(object_id, object_type, dimensions, pose);
    }

    /**
     * @brief Helper function to create an ObjectAction for adding a mesh-shaped object.
     * @param object_id Unique identifier for the object.
     * @param pose Pose of the object.
     * @param mesh_file Path to the mesh file.
     * @param scale_x Scale factor along the X-axis.
     * @param scale_y Scale factor along the Y-axis.
     * @param scale_z Scale factor along the Z-axis.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createAddMeshObject(const std::string &object_id,
                                            const geometry_msgs::msg::Pose &pose,
                                            const std::string &mesh_file,
                                            double scale_x = 1.0, double scale_y = 1.0, double scale_z = 1.0)
    {
        return ObjectAction(object_id, "mesh", {}, pose, mesh_file, scale_x, scale_y, scale_z);
    }

    /**
     * @brief Helper function to create an ObjectAction for attaching an object.
     * @param object_id Unique identifier for the object.
     * @param link_name Name of the robot link to attach the object to.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createAttachObject(const std::string &object_id, const std::string &link_name)
    {
        return ObjectAction(object_id, link_name, true);
    }

    /**
     * @brief Helper function to create an ObjectAction for detaching an object.
     * @param object_id Unique identifier for the object.
     * @param link_name Name of the robot link to detach the object from.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createDetachObject(const std::string &object_id, const std::string &link_name)
    {
        return ObjectAction(object_id, link_name, false);
    }

    /**
     * @brief Helper function to create an ObjectAction for checking object existence.
     * @param object_id Unique identifier for the object.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createCheckObjectExists(const std::string &object_id)
    {
        return ObjectAction(object_id);
    }

    /**
     * @brief Helper function to create an ObjectAction for removing an object.
     * @param object_id Unique identifier for the object.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createRemoveObject(const std::string &object_id)
    {
        ObjectAction action;
        action.type = ObjectActionType::REMOVE;
        action.object_id = object_id;
        return action;
    }

    /**
     * @brief Helper function to create an ObjectAction for getting and modifying object pose.
     * @param object_id Unique identifier for the object.
     * @param key Blackboard key to store the retrieved pose.
     * @param xyz_rpy Transformation offsets in format {x, y, z, roll, pitch, yaw}.
     * @param rpy Reference orientation in format {roll, pitch, yaw}.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createGetObjectPose(const std::string &object_id,
                                            const std::string &key,
                                            const std::vector<double> &pre_xyz_rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                            const std::vector<double> &post_xyz_rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
    {
        return ObjectAction(object_id, key, pre_xyz_rpy, post_xyz_rpy);
    }

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_OBJECT_HPP
