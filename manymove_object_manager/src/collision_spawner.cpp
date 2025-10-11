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

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
 # include <tf2/LinearMath/Quaternion.hpp>
#else
 # include <tf2/LinearMath/Quaternion.h>
#endif

#include <manymove_msgs/action/add_collision_object.hpp>
#include <manymove_msgs/action/check_object_exists.hpp>
#include <manymove_msgs/action/remove_collision_object.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

class CollisionSpawner : public rclcpp::Node
{
  public:
  struct ObjectSpec
  {
    std::string name;
    std::string type;
    std::vector<double> dimensions;
    bool has_pose;
    geometry_msgs::msg::Pose pose;
    std::optional<std::string> mesh_file;
    std::vector<double> scale_mesh;
  };

  using AddCollisionObject = manymove_msgs::action::AddCollisionObject;
  using RemoveCollisionObject = manymove_msgs::action::RemoveCollisionObject;
  using CheckObjectExists = manymove_msgs::action::CheckObjectExists;

  CollisionSpawner()
    : Node("collision_spawner"), rng_(std::random_device
  {
  }
                                        ())
  {
    // Declare parameters
    this->declare_parameter<std::string>
      ("frame_id",
      "world");
    frame_id_ = this->get_parameter
                  ("frame_id").as_string
                  ();

    this->declare_parameter<std::string>
      ("config_file",
      "");
    config_file_ = this->get_parameter
                     ("config_file").as_string
                     ();

    if (config_file_.empty
          ()) {
      RCLCPP_ERROR
        (this->get_logger
          (),
        "Parameter 'config_file' is empty. Cannot proceed.");
      rclcpp::shutdown
        ();
      return;
    }

    // Load YAML
    if (!loadObjectsFromYAML
          (config_file_)) {
      RCLCPP_ERROR
        (this->get_logger
          (),
        "Failed to load objects from YAML file: %s",
        config_file_.c_str
          ());
      rclcpp::shutdown
        ();
      return;
    }
    if (objects_to_spawn_.empty
          ()) {
      RCLCPP_WARN
        (this->get_logger
          (),
        "No valid objects to spawn.");
      return;
    }

    // Action Clients
    add_object_action_client_ = rclcpp_action::create_client<AddCollisionObject>
                                  (this,
                                  "add_collision_object");
    remove_object_action_client_ = rclcpp_action::create_client<RemoveCollisionObject>
                                     (this,
                                     "remove_collision_object");
    check_object_exists_action_client_ = rclcpp_action::create_client<CheckObjectExists>
                                           (this,
                                           "check_object_exists");

    RCLCPP_INFO
      (this->get_logger
        (),
      "Waiting for action servers...");
    if (!add_object_action_client_->wait_for_action_server
          (std::chrono::seconds
            (10))) {
      RCLCPP_ERROR
        (this->get_logger
          (),
        "AddCollisionObject action server not available. Exiting.");
      rclcpp::shutdown
        ();
      return;
    }
    if (!remove_object_action_client_->wait_for_action_server
          (std::chrono::seconds
            (10))) {
      RCLCPP_ERROR
        (this->get_logger
          (),
        "RemoveCollisionObject action server not available. Exiting.");
      rclcpp::shutdown
        ();
      return;
    }
    if (!check_object_exists_action_client_->wait_for_action_server
          (std::chrono::seconds
            (10))) {
      RCLCPP_WARN
      (
        this->get_logger
          (),
        "CheckObjectExists action server not available. Proceeding without existence checks.");
    }
    else {
      RCLCPP_INFO
        (this->get_logger
          (),
        "CheckObjectExists action server is available.");
    }

    // Remove existing spawner objects
    removeExistingSpawnerObjects
      ();

    RCLCPP_INFO
      (this->get_logger
        (),
      "Finished removing existing spawner objects. Spawning new objects...");
    spawnObjects
      ();

    RCLCPP_INFO
      (this->get_logger
        (),
      "Finished adding spawner objects. Shutting down...");

    // Schedule shutdown
    shutdown_timer_ = this->create_wall_timer
                        (std::chrono::seconds
                          (1),
                        [this]()
    {
      RCLCPP_INFO
        (this->get_logger
          (),
        "Shutting down node.");
      rclcpp::shutdown
        ();
    });
  }

  private:
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  bool loadObjectsFromYAML(const std::string & file_path)
  {
    try {
      YAML::Node config = YAML::LoadFile
                            (file_path);
      if (!config["objects"]) {
        RCLCPP_ERROR
          (this->get_logger
            (),
          "YAML file does not contain 'objects' key.");
        return false;
      }

      for (const auto & obj_node : config["objects"]) {
        ObjectSpec obj_spec;
        if (!obj_node["name"] || !obj_node["type"]) {
          RCLCPP_WARN
            (this->get_logger
              (),
            "Object spec missing 'name' or 'type'. Skipping.");
          continue;
        }

        obj_spec.name = obj_node["name"].as<std::string>
                          ();
        obj_spec.type = obj_node["type"].as<std::string>
                          ();

        if (obj_spec.type == "mesh") {
          if (!obj_node["mesh_file"]) {
            RCLCPP_WARN
              (this->get_logger
                (),
              "Mesh object '%s' missing 'mesh_file'. Skipping.",
              obj_spec.name.c_str
                ());
            continue;
          }
          obj_spec.mesh_file = obj_node["mesh_file"].as<std::string>
                                 ();

          // Parse scaling values for mesh
          if (obj_node["scale"]) {
            obj_spec.scale_mesh = obj_node["scale"].as<std::vector<double> >
                                    ();
          }
          else {
            obj_spec.scale_mesh =
            {
              1.0, 1.0, 1.0
            };
          }
        }
        else {
          if (!obj_node["dimensions"]) {
            RCLCPP_WARN
              (this->get_logger
                (),
              "Object '%s' missing 'dimensions'. Skipping.",
              obj_spec.name.c_str
                ());
            continue;
          }
          obj_spec.dimensions = obj_node["dimensions"].as<std::vector<double> >
                                  ();
        }

        if (obj_node["pose"]) {
          if (obj_node["pose"]["position"] && obj_node["pose"]["orientation"]) {
            obj_spec.pose.position.x = obj_node["pose"]["position"]["x"].as<double>
                                         ();
            obj_spec.pose.position.y = obj_node["pose"]["position"]["y"].as<double>
                                         ();
            obj_spec.pose.position.z = obj_node["pose"]["position"]["z"].as<double>
                                         ();

            double roll = obj_node["pose"]["orientation"]["roll"].as<double>
                            ();
            double pitch = obj_node["pose"]["orientation"]["pitch"].as<double>
                             ();
            double yaw = obj_node["pose"]["orientation"]["yaw"].as<double>
                           ();
            obj_spec.pose.orientation = rpyToQuaternion
                                          (roll,
                                          pitch,
                                          yaw);

            obj_spec.has_pose = true;
          }
          else {
            RCLCPP_WARN
              (this->get_logger
                (),
              "Object '%s' has incomplete 'pose'. Will be placed randomly.",
              obj_spec.name.c_str
                ());
            obj_spec.has_pose = false;
          }
        }
        else {
          obj_spec.has_pose = false;
        }

        objects_to_spawn_.push_back
          (obj_spec);
      }
      return true;
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR
        (this->get_logger
          (),
        "YAML Exception: %s",
        e.what
          ());
      return false;
    }
    catch (const std::exception & e) {
      RCLCPP_ERROR
        (this->get_logger
          (),
        "Exception while loading YAML file: %s",
        e.what
          ());
      return false;
    }
  }

  void removeExistingSpawnerObjects()
  {
    RCLCPP_INFO
      (this->get_logger
        (),
      "Removing existing spawner objects...");
    for (const auto & obj : objects_to_spawn_) {
      auto object_id = obj.name + "_spawner";

      // 1. Check if it exists
      if (!isObjectPresent
            (object_id)) {
        RCLCPP_INFO
          (this->get_logger
            (),
          "Object '%s' does not exist. Skipping removal.",
          object_id.c_str
            ());
        continue;
      }

      // 2. Proceed with remove
      sendRemoveGoal
        (object_id);
    }
  }

  void sendRemoveGoal(const std::string & object_id)
  {
    auto goal = RemoveCollisionObject::Goal
                  ();
    goal.id = object_id;

    RCLCPP_INFO
      (this->get_logger
        (),
      "Sending remove goal for object '%s'.",
      object_id.c_str
        ());

    auto send_goal_future = remove_object_action_client_->async_send_goal
                              (goal);

    // Wait for the goal to be accepted
    if (rclcpp::spin_until_future_complete
          (this->get_node_base_interface
            (),
          send_goal_future,
          std::chrono::seconds
            (5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN
        (this->get_logger
          (),
        "Timeout while sending remove goal for object '%s'.",
        object_id.c_str
          ());
      return;
    }

    auto goal_handle = send_goal_future.get
                         ();
    if (!goal_handle) {
      RCLCPP_ERROR
        (this->get_logger
          (),
        "Remove goal was rejected for object '%s'.",
        object_id.c_str
          ());
      return;
    }

    // Wait for the result
    auto get_result_future = remove_object_action_client_->async_get_result
                               (goal_handle);

    if (rclcpp::spin_until_future_complete
          (this->get_node_base_interface
            (),
          get_result_future,
          std::chrono::seconds
            (5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN
        (this->get_logger
          (),
        "Timeout while waiting for remove result for object '%s'.",
        object_id.c_str
          ());
      return;
    }

    auto result = get_result_future.get
                    ();
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO
        (this->get_logger
          (),
        "Successfully removed object '%s'.",
        object_id.c_str
          ());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN
        (this->get_logger
          (),
        "Remove action was aborted for object '%s'.",
        object_id.c_str
          ());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN
        (this->get_logger
          (),
        "Remove action was canceled for object '%s'.",
        object_id.c_str
          ());
      break;
    default:
      RCLCPP_WARN
        (this->get_logger
          (),
        "Unknown result code for remove action of object '%s'.",
        object_id.c_str
          ());
      break;
    }
  }

  void spawnObjects()
  {
    for (const auto & obj : objects_to_spawn_) {
      sendAddGoal
        (obj);
    }
  }

  void sendAddGoal(const ObjectSpec & obj_spec)
  {
    auto goal = AddCollisionObject::Goal
                  ();
    goal.id = obj_spec.name + "_spawner";
    goal.shape = obj_spec.type;
    goal.dimensions = obj_spec.type == "mesh" ? std::vector<double>
                        () : obj_spec.dimensions;
    goal.pose = obj_spec.has_pose ? obj_spec.pose : generateRandomPose
                  ();

    if (obj_spec.type == "mesh") {
      goal.mesh_file = obj_spec.mesh_file.value
                         ();
      goal.scale_mesh = obj_spec.scale_mesh;
    }

    RCLCPP_INFO
      (this->get_logger
        (),
      "Sending add goal for object '%s'.",
      obj_spec.name.c_str
        ());

    auto send_goal_future = add_object_action_client_->async_send_goal
                              (goal);

    // Wait for the goal to be accepted
    if (rclcpp::spin_until_future_complete
          (this->get_node_base_interface
            (),
          send_goal_future,
          std::chrono::seconds
            (5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN
        (this->get_logger
          (),
        "Timeout while sending add goal for object '%s'.",
        obj_spec.name.c_str
          ());
      return;
    }

    auto goal_handle = send_goal_future.get
                         ();
    if (!goal_handle) {
      RCLCPP_ERROR
        (this->get_logger
          (),
        "Add goal was rejected for object '%s'.",
        obj_spec.name.c_str
          ());
      return;
    }

    // Wait for the result
    auto get_result_future = add_object_action_client_->async_get_result
                               (goal_handle);

    if (rclcpp::spin_until_future_complete
          (this->get_node_base_interface
            (),
          get_result_future,
          std::chrono::seconds
            (5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN
        (this->get_logger
          (),
        "Timeout while waiting for add result for object '%s'.",
        obj_spec.name.c_str
          ());
      return;
    }

    auto result = get_result_future.get
                    ();
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO
        (this->get_logger
          (),
        "Successfully added object '%s'.",
        obj_spec.name.c_str
          ());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN
        (this->get_logger
          (),
        "Add action was aborted for object '%s'.",
        obj_spec.name.c_str
          ());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN
        (this->get_logger
          (),
        "Add action was canceled for object '%s'.",
        obj_spec.name.c_str
          ());
      break;
    default:
      RCLCPP_WARN
        (this->get_logger
          (),
        "Unknown result code for add action of object '%s'.",
        obj_spec.name.c_str
          ());
      break;
    }
  }

// ----------------------------------------------------------------------------
// New Utility Function: isObjectPresent
// ----------------------------------------------------------------------------
  bool isObjectPresent(const std::string & object_id)
  {
    if (!check_object_exists_action_client_->action_server_is_ready
          ()) {
      RCLCPP_WARN
        (this->get_logger
          (),
        "CheckObjectExists action server not ready. Assuming object '%s' does not exist.",
        object_id.c_str
          ());
      return false;
    }

    auto goal_msg = CheckObjectExists::Goal
                      ();
    goal_msg.object_id = object_id;

    RCLCPP_INFO
      (this->get_logger
        (),
      "Sending CheckObjectExists goal for object '%s'.",
      object_id.c_str
        ());

    auto send_goal_future = check_object_exists_action_client_->async_send_goal
                              (goal_msg);

    // Wait for the goal to be accepted
    if (rclcpp::spin_until_future_complete
          (this->get_node_base_interface
            (),
          send_goal_future,
          std::chrono::seconds
            (3)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN
      (
        this->get_logger
          (),
        "Timeout while sending CheckObjectExists goal for object '%s'. Assuming it does not exist.",
        object_id.c_str
          ());
      return false;
    }

    auto goal_handle = send_goal_future.get
                         ();
    if (!goal_handle) {
      RCLCPP_ERROR
      (
        this->get_logger
          (),
        "CheckObjectExists goal was rejected for object '%s'. Assuming it does not exist.",
        object_id.c_str
          ());
      return false;
    }

    // Wait for the result
    auto get_result_future = check_object_exists_action_client_->async_get_result
                               (goal_handle);

    if (rclcpp::spin_until_future_complete
          (this->get_node_base_interface
            (),
          get_result_future,
          std::chrono::seconds
            (3)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN
      (
        this->get_logger
          (),
        "Timeout while waiting for CheckObjectExists result for object '%s'. Assuming it does not exist.",
        object_id.c_str
          ());
      return false;
    }

    auto result = get_result_future.get
                    ().result;
    RCLCPP_INFO
      (this->get_logger
        (),
      "CheckObjectExists result for object '%s': exists=%s",
      object_id.c_str
        (),
      result->exists ? "true" : "false");
    return result->exists;
  }

// ----------------------------------------------------------------------------
// Existing Utility Function: generateRandomPose
// ----------------------------------------------------------------------------
  geometry_msgs::msg::Pose generateRandomPose()
  {
    geometry_msgs::msg::Pose pose;
    std::uniform_real_distribution<double> dist_x(0.15, 0.3);
    std::uniform_real_distribution<double> dist_y(-0.25, 0.25);
    std::uniform_real_distribution<double> dist_z(0.05, 0.25);
    std::uniform_real_distribution<double> dist_angle(0, 2 * M_PI);

    pose.position.x = dist_x
                        (rng_);
    pose.position.y = dist_y
                        (rng_);
    pose.position.z = dist_z
                        (rng_);

    pose.orientation = rpyToQuaternion
                         (dist_angle
                           (rng_),
                         dist_angle
                           (rng_),
                         dist_angle
                           (rng_));
    return pose;
  }

// ----------------------------------------------------------------------------
// Existing Utility Function: rpyToQuaternion
// ----------------------------------------------------------------------------
  geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY
      (roll,
      pitch,
      yaw);
    q.normalize
      ();
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x
                ();
    q_msg.y = q.y
                ();
    q_msg.z = q.z
                ();
    q_msg.w = q.w
                ();
    return q_msg;
  }

  private:
  std::vector<ObjectSpec> objects_to_spawn_;
  rclcpp_action::Client<AddCollisionObject>::SharedPtr add_object_action_client_;
  rclcpp_action::Client<RemoveCollisionObject>::SharedPtr remove_object_action_client_;
  rclcpp_action::Client<CheckObjectExists>::SharedPtr check_object_exists_action_client_;

  std::mt19937 rng_;
  std::string frame_id_;
  std::string config_file_;
};

int main(int argc, char ** argv)
{
  rclcpp::init
    (argc,
    argv);
  auto node = std::make_shared<CollisionSpawner>
                ();
  rclcpp::spin
    (node);
  rclcpp::shutdown
    ();
  return 0;
}
