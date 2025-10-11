// Copyright 2025 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
    ADD,         ///< Add a new object to the planning scene.
    REMOVE,      ///< Remove an existing object from the planning scene.
    ATTACH,      ///< Attach an object to a specific robot link.
    DETACH,      ///< Detach an object from a specific robot link.
    CHECK,       ///< Check if an object exists and its attachment status.
    GET_POSE     ///< Retrieve and possibly modify the pose of an object.
  };

/**
 * @struct ObjectAction
 * @brief Represents an action to be performed on a collision object.
 */
  struct ObjectAction
  {
    ObjectActionType type;     ///< The type of object action.
    std::string object_id_key_st;     ///< Unique identifier for the object.
    // Parameters for ADD action
    std::string shape_key_st;           ///< Shape type (e.g., box, mesh).
    std::string dimensions_key_d_a;     ///< Dimensions for primitive shapes (e.g., width, height,
                                        // depth).
    std::string pose_key;               ///< Blackboard key to store the retrieved pose (used only
                                        // for GET_POSE).
    std::string mesh_file_key_st;       ///< Path to the mesh file (required if shape is mesh).
    std::string scale_key_d_a;          ///< Blackboard key to store the retrieved scale (used only
                                        // for GET_POSE).
    // Parameters for ATTACH and DETACH actions
    std::string link_name_key_st;         ///< Name of the robot link to attach/detach the object.
    std::string touch_links_key_st_a;     ///< (Optional) List of robot links to exclude from
                                          // collision checking.
    // Parameters for GET_POSE action (also uses the link_name for relative)
    std::string pre_transform_xyz_rpy_key_d_a;      ///< Linear transform in x, y and z and rotation
                                                    // in roll, pitch, yaw of the pose of the object
    std::string post_transform_xyz_rpy_key_d_a;     ///< Reference orientation for the pose
                                                    // transform of the pose

    /**
     * @brief Default constructor.
     */
    ObjectAction() = default;
  };

/**
 * @brief Helper function to create an ObjectAction for adding a objects.
 * @param object_id_key_st Unique identifier for the object.
 * @param dimensions_key Dimensions of the box (width, height, depth).
 * @param pose_key Pose of the object.
 * @param object_shape_key Type of primitive object, possible values: box, cylinder, sphere, mesh.
 * @param mesh_file_key Path to the mesh file.
 * @param scale_key Scale factors along the X, Y and Z axis.
 * @return Configured ObjectAction.
 */
  inline ObjectAction createAddObject(
    const std::string& object_id_key_st,
    const std::string& object_shape_key,
    const std::string& dimensions_key,
    const std::string& pose_key,
    const std::string& scale_key,
    const std::string& mesh_file_key)
  {
    ObjectAction action;
    action.type = ObjectActionType::ADD;
    action.object_id_key_st = object_id_key_st;
    action.shape_key_st = object_shape_key;
    action.dimensions_key_d_a = dimensions_key;
    action.pose_key = pose_key;
    action.mesh_file_key_st = mesh_file_key;
    action.scale_key_d_a = scale_key;
    return action;
  }

/**
 * @brief Helper function to create an ObjectAction for attaching an object.
 * @param object_id_key_st Unique identifier for the object.
 * @param link_name Name of the robot link to attach the object to.
 * @return Configured ObjectAction.
 */
  inline ObjectAction createAttachObject(
    const std::string& object_id_key_st,
    const std::string& link_name_key,
    const std::string& touch_links_key)
  {
    ObjectAction action;
    action.type = ObjectActionType::ATTACH;
    action.object_id_key_st = object_id_key_st;
    action.link_name_key_st = link_name_key;
    action.touch_links_key_st_a = touch_links_key;
    return action;
  }

/**
 * @brief Helper function to create an ObjectAction for detaching an object.
 * @param object_id_key_st Unique identifier for the object.
 * @param link_name_key Name of the robot link to detach the object from.
 * @return Configured ObjectAction.
 */
  inline ObjectAction createDetachObject(
    const std::string& object_id_key_st,
    const std::string& link_name_key)
  {
    ObjectAction action;
    action.type = ObjectActionType::DETACH;
    action.object_id_key_st = object_id_key_st;
    action.link_name_key_st = link_name_key;
    return action;
  }

/**
 * @brief Helper function to create an ObjectAction for checking object existence.
 * @param object_id_key_st Unique identifier for the object.
 * @return Configured ObjectAction.
 */
  inline ObjectAction createCheckObjectExists(const std::string& object_id_key_st)
  {
    ObjectAction action;
    action.type = ObjectActionType::CHECK;
    action.object_id_key_st = object_id_key_st;
    return action;
  }

/**
 * @brief Helper function to create an ObjectAction for removing an object.
 * @param object_id_key_st Unique identifier for the object.
 * @return Configured ObjectAction.
 */
  inline ObjectAction createRemoveObject(const std::string& object_id_key_st)
  {
    ObjectAction action;
    action.type = ObjectActionType::REMOVE;
    action.object_id_key_st = object_id_key_st;
    return action;
  }

/**
 * @brief Helper function to create an ObjectAction for getting and modifying object pose.
 * @param object_id_key_st Unique identifier for the object.
 * @param pose_key Blackboard key to store the retrieved pose.
 * @param pre_xyz_rpy_key Transformation offsets in format {x, y, z, roll, pitch, yaw}.
 * @param post_xyz_rpy_key Secon transformation offsets in format {x, y, z, roll, pitch, yaw}.
 * @return Configured ObjectAction.
 */
  inline ObjectAction createGetObjectPose(
    const std::string& object_id_key_st,
    const std::string& pose_key,
    const std::string& link_name_key,
    const std::string& pre_xyz_rpy_key,
    const std::string& post_xyz_rpy_key)
  {
    ObjectAction action;
    action.type = ObjectActionType::GET_POSE;
    action.object_id_key_st = object_id_key_st;
    action.pose_key = pose_key;
    action.link_name_key_st = link_name_key;
    action.pre_transform_xyz_rpy_key_d_a = pre_xyz_rpy_key;
    action.post_transform_xyz_rpy_key_d_a = post_xyz_rpy_key;
    return action;
  }

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_OBJECT_HPP
