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

#ifndef MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
#define MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/blackboard.h>
#include <std_msgs/msg/string.hpp>
#include <manymove_msgs/srv/set_blackboard_values.hpp>
#include "manymove_cpp_trees/tree_helper.hpp"
#include "manymove_cpp_trees/blackboard_utils.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>

namespace manymove_cpp_trees
{

/**
 * @brief A node that provides a single "update_blackboard" service
 *        (using manymove_msgs::srv::SetBlackboardValues) and publishes
 *        the status of keys every 250ms.
 *
 * All data is handled as JSON. The publishBlackboardStatus() method iterates
 * over a vector of BlackboardEntry structures to build the JSON message.
 */
  class HMIServiceNode : public rclcpp::Node
  {
public:
    // Constructor: pass the complete vector of keys.
    explicit HMIServiceNode(const std::string& node_name,
                            BT::Blackboard::Ptr blackboard,
                            std::vector<BlackboardEntry> keys = {});

private:
    BT::Blackboard::Ptr blackboard_;
    std::vector<BlackboardEntry> keys_;

    // The update_blackboard service.
    rclcpp::Service<manymove_msgs::srv::SetBlackboardValues>::SharedPtr update_blackboard_srv_;

    // Publisher for blackboard status.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // Timer to publish status every 250ms.
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Service callback.
    void handleUpdateBlackboard(
      const std::shared_ptr<manymove_msgs::srv::SetBlackboardValues::Request> request,
      std::shared_ptr<manymove_msgs::srv::SetBlackboardValues::Response> response);

    std::string serializePoseRPY(const geometry_msgs::msg::Pose& pose);

    // Timer callback.
    void publishBlackboardStatus();

    // Minimal JSON parse for "double_array" (e.g. "[0.01, 0.01, 0.25]").
    std::vector<double> parseJsonDoubleArray(const std::string& json_str);

    // Minimal JSON parse for pose (e.g.
    // {"x":0.1,"y":0.2,"z":0.3,"roll":1.57,"pitch":0.0,"yaw":0.0}).
    geometry_msgs::msg::Pose parseJsonPose(const std::string& json_str);
  };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
