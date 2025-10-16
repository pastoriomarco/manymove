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

#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/blackboard.h>

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "manymove_cpp_trees/tree_helper.hpp"
#include "manymove_cpp_trees/move.hpp"

using manymove_cpp_trees::BlackboardEntry;
using manymove_cpp_trees::ObjectSnippets;
using manymove_cpp_trees::createPose;

TEST(TreeHelper, CreateObjectSnippets_PrimitiveValidation)
{
  auto bb = BT::Blackboard::create();
  std::vector<BlackboardEntry> keys;
  geometry_msgs::msg::Pose pose = createPose(0, 0, 0, 0, 0, 0, 1);

  // valid box
  ObjectSnippets s = manymove_cpp_trees::createObjectSnippets(
    bb, keys, "box1", "box", pose, {0.1, 0.2, 0.3}, "", {1.0, 1.0, 1.0}, "tool0", "[fl,fr]");
  EXPECT_FALSE(s.add_xml.empty());
  EXPECT_NE(s.add_xml.find("AddCollisionObjectAction"), std::string::npos);
  EXPECT_NE(s.remove_xml.find("RemoveCollisionObjectAction"), std::string::npos);
  EXPECT_NE(s.attach_xml.find("AttachDetachObjectAction"), std::string::npos);
  EXPECT_NE(s.detach_xml.find("AttachDetachObjectAction"), std::string::npos);

  // duplicate name should throw
  EXPECT_THROW(
    manymove_cpp_trees::createObjectSnippets(
      bb, keys, "box1", "box", pose, {0.1, 0.2, 0.3}, "", {1, 1, 1}),
    BT::RuntimeError);

  // wrong dims
  EXPECT_THROW(
    manymove_cpp_trees::createObjectSnippets(
      bb, keys, "box2", "box", pose, {0.1, 0.2}, "", {1, 1, 1}),
    BT::RuntimeError);

  // negative scale
  EXPECT_THROW(
    manymove_cpp_trees::createObjectSnippets(
      bb, keys, "box3", "box", pose, {0.1, 0.2, 0.3}, "", {-1.0, 1.0, 1.0}),
    BT::RuntimeError);
}

TEST(TreeHelper, CreateObjectSnippets_MeshValidation)
{
  auto bb = BT::Blackboard::create();
  std::vector<BlackboardEntry> keys;
  geometry_msgs::msg::Pose pose = createPose(0, 0, 0, 0, 0, 0, 1);

  // missing mesh file
  EXPECT_THROW(
    manymove_cpp_trees::createObjectSnippets(
      bb, keys, "mesh1", "mesh", pose, {}, "", {1, 1, 1}),
    BT::RuntimeError);
}

TEST(TreeHelper, BuildObjectActionXMLVariants)
{
  manymove_cpp_trees::ObjectAction add = manymove_cpp_trees::createAddObject(
    "obj_id", "shape_key", "dims_key", "pose_key", "scale_key", "mesh_key");
  std::string add_xml = manymove_cpp_trees::buildObjectActionXML("prefix", add);
  EXPECT_NE(add_xml.find("AddCollisionObjectAction"), std::string::npos);
  EXPECT_NE(add_xml.find("object_id=\"{obj_id}\""), std::string::npos);

  manymove_cpp_trees::ObjectAction rm = manymove_cpp_trees::createRemoveObject("obj_id");
  std::string rm_xml = manymove_cpp_trees::buildObjectActionXML("prefix", rm);
  EXPECT_NE(rm_xml.find("RemoveCollisionObjectAction"), std::string::npos);

  manymove_cpp_trees::ObjectAction att =
    manymove_cpp_trees::createAttachObject("obj_id", "link_key", "touch_key");
  std::string att_xml = manymove_cpp_trees::buildObjectActionXML("prefix", att);
  EXPECT_NE(att_xml.find("AttachDetachObjectAction"), std::string::npos);
  EXPECT_NE(att_xml.find("attach=\"true\""), std::string::npos);

  manymove_cpp_trees::ObjectAction det =
    manymove_cpp_trees::createDetachObject("obj_id", "link_key");
  std::string det_xml = manymove_cpp_trees::buildObjectActionXML("prefix", det);
  EXPECT_NE(det_xml.find("AttachDetachObjectAction"), std::string::npos);
  EXPECT_NE(det_xml.find("attach=\"false\""), std::string::npos);

  manymove_cpp_trees::ObjectAction gp = manymove_cpp_trees::createGetObjectPose(
    "obj", "pose_key", "link_key", "pre_key", "post_key");
  std::string gp_xml = manymove_cpp_trees::buildObjectActionXML("prefix", gp);
  EXPECT_NE(gp_xml.find("GetObjectPoseAction"), std::string::npos);
  EXPECT_NE(gp_xml.find("pose_key=\"pose_key\""), std::string::npos);
}

TEST(TreeHelper, BuildMoveXMLSeedsBlackboardAndNodes)
{
  auto bb = BT::Blackboard::create();

  manymove_msgs::msg::MovementConfig cfg;
  cfg.velocity_scaling_factor = 1.0;
  cfg.acceleration_scaling_factor = 1.0;
  cfg.max_cartesian_speed = 0.1;

  manymove_cpp_trees::Move m("R_", "tool0", "joint", cfg, "", {0.1, 0.2, 0.3});
  std::string xml = manymove_cpp_trees::buildMoveXML(
    "R_", "nodeA", std::vector<manymove_cpp_trees::Move>{m}, bb, /*reset_trajs=*/ true, 1);

  // contains a MoveManipulatorAction and a ResetTrajectories
  EXPECT_NE(xml.find("MoveManipulatorAction"), std::string::npos);
  EXPECT_NE(xml.find("ResetTrajectories"), std::string::npos);

  // BlackBoard contains move_0 and trajectory_0
  std::shared_ptr<manymove_cpp_trees::Move> m0;
  EXPECT_TRUE(bb->get("move_0", m0));
  moveit_msgs::msg::RobotTrajectory traj0;
  EXPECT_TRUE(bb->get("trajectory_0", traj0));
}
