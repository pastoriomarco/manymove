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

#ifndef MANYMOVE_CPP_TREES_BLACKBOARD_UTILS_HPP
#define MANYMOVE_CPP_TREES_BLACKBOARD_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/blackboard.h>
#include <string>
#include <vector>

namespace manymove_cpp_trees
{
// Structure representing a blackboard key.
  struct BlackboardEntry
  {
    std::string key;
    std::string value_type;     // e.g. "bool", "double", "string", "pose", etc.
  };

  template<class T>
  void defineVariableKey(
    const rclcpp::Node::SharedPtr& node_ptr,
    BT::Blackboard::Ptr blackboard,
    std::vector<BlackboardEntry>& keys,
    const std::string& key_name,
    const std::string& key_type,
    const T& value)
  {
    // Set the blackboard key with initial value
    blackboard->set(key_name, value);

    // These keys need to be published for the HMI, adding them here:
    keys.push_back({key_name, key_type});

    RCLCPP_INFO(node_ptr->get_logger(),
                "%s key added to 'blackboard_status' service.",
                key_name.c_str());
  }
}

#endif // MANYMOVE_CPP_TREES_BLACKBOARD_UTILS_HPP
