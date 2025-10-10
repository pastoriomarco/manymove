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
  std::string value_type;       // e.g. "bool", "double", "string", "pose", etc.
};

template<class T>
void defineVariableKey(
  const rclcpp::Node::SharedPtr & node_ptr,
  BT::Blackboard::Ptr blackboard,
  std::vector<BlackboardEntry> & keys,
  const std::string & key_name,
  const std::string & key_type,
  const T & value)
{
  // Set the blackboard key with initial value
  blackboard->set(key_name, value);

  // These keys need to be published for the HMI, adding them here:
  keys.push_back({key_name, key_type});

  RCLCPP_INFO(
    node_ptr->get_logger(),
    "%s key added to 'blackboard_status' service.",
    key_name.c_str());
}
}

#endif // MANYMOVE_CPP_TREES_BLACKBOARD_UTILS_HPP
