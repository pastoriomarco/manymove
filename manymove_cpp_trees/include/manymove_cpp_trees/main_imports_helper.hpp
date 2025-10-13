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

#ifndef MANYMOVE_CPP_TREES__MAIN_IMPORTS_HELPER_HPP_
#define MANYMOVE_CPP_TREES__MAIN_IMPORTS_HELPER_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/decorators/force_failure_node.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "manymove_cpp_trees/action_nodes_logic.hpp"
#include "manymove_cpp_trees/action_nodes_objects.hpp"
#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include "manymove_cpp_trees/action_nodes_signals.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"
#include "manymove_cpp_trees/hmi_service_node.hpp"
#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/object.hpp"
#include "manymove_cpp_trees/robot.hpp"
#include "manymove_cpp_trees/tree_helper.hpp"
#include "manymove_msgs/action/check_robot_state.hpp"
#include "manymove_msgs/action/get_input.hpp"
#include "manymove_msgs/action/reset_robot_state.hpp"
#include "manymove_msgs/action/set_output.hpp"
#include "manymove_cpp_trees/action_nodes_isaac.hpp"

using geometry_msgs::msg::Pose;

using manymove_cpp_trees::defineRobotParams;
using manymove_cpp_trees::defineVariableKey;
using manymove_cpp_trees::defineMovementConfigs;
using manymove_cpp_trees::buildMoveXML;
using manymove_cpp_trees::sequenceWrapperXML;
using manymove_cpp_trees::parallelWrapperXML;
using manymove_cpp_trees::fallbackWrapperXML;
using manymove_cpp_trees::repeatSequenceWrapperXML;
using manymove_cpp_trees::retrySequenceWrapperXML;
using manymove_cpp_trees::BlackboardEntry;
using manymove_cpp_trees::Move;
using manymove_cpp_trees::RobotParams;
using manymove_cpp_trees::createPose;
using manymove_cpp_trees::createPoseRPY;
using manymove_cpp_trees::buildObjectActionXML;
using manymove_cpp_trees::createCheckObjectExists;
using manymove_cpp_trees::createAddObject;
using manymove_cpp_trees::createAttachObject;
using manymove_cpp_trees::createDetachObject;
using manymove_cpp_trees::createRemoveObject;
using manymove_cpp_trees::createGetObjectPose;
using manymove_cpp_trees::ObjectSnippets;
using manymove_cpp_trees::createObjectSnippets;
using manymove_cpp_trees::buildSetOutputXML;
using manymove_cpp_trees::buildWaitForInput;
using manymove_cpp_trees::buildCheckRobotStateXML;
using manymove_cpp_trees::buildResetRobotStateXML;
using manymove_cpp_trees::buildWaitForObject;
using manymove_cpp_trees::buildWaitForKeyBool;
using manymove_cpp_trees::buildCheckKeyBool;
using manymove_cpp_trees::buildSetKeyBool;
using manymove_cpp_trees::buildFoundationPoseSequenceXML;
using manymove_cpp_trees::buildCopyPoseXML;
using manymove_cpp_trees::buildCheckPoseDistanceXML;
using manymove_cpp_trees::mainTreeWrapperXML;
using manymove_cpp_trees::registerAllNodeTypes;

#endif  // MANYMOVE_CPP_TREES__MAIN_IMPORTS_HELPER_HPP_
