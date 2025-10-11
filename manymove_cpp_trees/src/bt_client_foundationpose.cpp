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

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/decorators/force_failure_node.h>

#include "manymove_cpp_trees/action_nodes_objects.hpp"
#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include "manymove_cpp_trees/action_nodes_signals.hpp"
#include "manymove_cpp_trees/action_nodes_logic.hpp"
#include "manymove_cpp_trees/action_nodes_isaac.hpp"
#include "manymove_cpp_trees/robot.hpp"
#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/object.hpp"
#include "manymove_cpp_trees/tree_helper.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"
#include "manymove_cpp_trees/hmi_service_node.hpp"

#include "manymove_msgs/action/set_output.hpp"
#include "manymove_msgs/action/get_input.hpp"
#include "manymove_msgs/action/check_robot_state.hpp"
#include "manymove_msgs/action/reset_robot_state.hpp"

#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>

using geometry_msgs::msg::Pose;
using namespace manymove_cpp_trees;

int main(int argc, char ** argv)
{
  // ----------------------------------------------------------------------------
  // 0. Preparing the node, blackboard and robot params
  // ----------------------------------------------------------------------------

  rclcpp::init
    (argc,
    argv);

  auto node = rclcpp::Node::make_shared
                ("bt_client_foundationpose");
  RCLCPP_INFO
    (node->get_logger
      (),
    "BT Client FoundationPose Node started (Programmatic XML).");

  // Create a blackboard and set "node"
  auto blackboard = BT::Blackboard::create
                      ();
  blackboard->set
    ("node",
    node);
  RCLCPP_INFO
    (node->get_logger
      (),
    "Blackboard: set('node', <rclcpp::Node>)");

  // Create the keys variable for HMI
  std::vector<manymove_cpp_trees::BlackboardEntry> keys;

  // Define all params and blackboard keys for the robot:
  RobotParams rp = defineRobotParams
                     (node,
                     blackboard,
                     keys);
  auto move_configs = defineMovementConfigs
                        ();

  // UTILITY KEYS

  blackboard->set
    ("world_frame_key",
    "world");
  blackboard->set
    ("identity_transform_key",
    std::vector<double>
  {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  });
  blackboard->set
    ("tcp_frame_name_key",
    "link_tcp");

  blackboard->set
    ("touch_links_key",
    rp.contact_links);

  // ----------------------------------------------------------------------------
  // 1. Create the scene
  // ----------------------------------------------------------------------------

  // Objects in the scene:
  // This is the new unified helper function to create all the snippets to handle any kind of
  // objects
  ObjectSnippets ground = createObjectSnippets
                            (blackboard,
                            keys,
                            "ground",                          /* object name */
                            "box",                          /* shape */
                            createPoseRPY
                              (0.0,
                              0.0,
                              -0.051,
                              0.0,
                              0.0,
                              0.0),                                                                  /*
                                                                                                      *
                                                                                                      *
                                                                                                      *
                                                                                                      *
                                                                                                      * pose
                                                                                                      *
                                                                                                      *
                                                                                                      *
                                                                                                      *
                                                                                                      * of
                                                                                                      *
                                                                                                      *
                                                                                                      *
                                                                                                      *
                                                                                                      * the
                                                                                                      *
                                                                                                      *
                                                                                                      *
                                                                                                      *
                                                                                                      * object
                                                                                                      *
                                                                                                      **/
  {
    1.0, 1.0, 0.1
  },                                                                  /* primitive dimensions */
                            "",                          /* mesh file path */
  {
    1.0, 1.0, 1.0
  },                                                                  /* scale */
                            "",                          /* link name to attach/detach */
  {
  }                                                     /* contact links to attach/detach */
                            );

  ObjectSnippets wall = createObjectSnippets
                          (blackboard,
                          keys,
                          "wall",
                          "box",
                          createPoseRPY
                            (0.0,
                            -0.15,
                            0.1,
                            0.0,
                            0.0,
                            0.0),
  {
    1.0, 0.02, 0.2
  });

  ObjectSnippets graspable = createObjectSnippets
                               (blackboard,
                               keys,
                               "graspable",
                               "box",
                               Pose
                                 (),
  {
    0.06, 0.015, 0.015
  },
                               "",
  {
    1.0, 1.0, 1.0
  },
                               "tcp_frame_name_key",
                               "touch_links_key");

  // blackboard->set("pick_target_key", Pose());
  defineVariableKey<Pose>
    (node,
    blackboard,
    keys,
    "pick_target_key",
    "pose",
    Pose
      ());

  // blackboard->set("approach_pick_target_key", Pose());
  defineVariableKey<Pose>
    (node,
    blackboard,
    keys,
    "approach_pick_target_key",
    "pose",
    Pose
      ());

  blackboard->set
    ("foundationpose_header_key",
    std_msgs::msg::Header
      ());
  blackboard->set
    ("graspable_spawn_pose_key",
    Pose
      ());
  Pose disposed_pose = createPose
                         (0.0,
                         0.0,
                         -0.5,
                         0.0,
                         0.0,
                         0.0,
                         1.0);
  blackboard->set
    ("graspable_disposal_pose_key",
    disposed_pose);

  // Drop poses to place the object, these are not overwritten later, so we hardcode them
  // Here we create the drop pose first, then we set it in the blackboard key
  Pose drop_target = createPose
                       (0.2,
                       0.0,
                       0.2,
                       1.0,
                       0.0,
                       0.0,
                       0.0);
  blackboard->set
    ("drop_target_key",
    drop_target);

  // The approach move from the drop pose is cartesian, we set an offset in the direction of the
  // move (here, Z)
  Pose approach_drop_target = drop_target;
  approach_drop_target.position.z += 0.02;
  blackboard->set
    ("approach_drop_target_key",
    approach_drop_target);

  // Adjusting only the scaling factors for default moves
  auto & max_move = move_configs["max_move"];
  max_move.velocity_scaling_factor = 0.7;
  max_move.acceleration_scaling_factor = 0.35;
  max_move.max_cartesian_speed = 0.25;

  auto & mid_move = move_configs["mid_move"];
  mid_move.velocity_scaling_factor = 0.4;
  mid_move.acceleration_scaling_factor = 0.2;
  mid_move.max_cartesian_speed = 0.25;

  auto & slow_move = move_configs["slow_move"];
  slow_move.velocity_scaling_factor = 0.2;
  slow_move.acceleration_scaling_factor = 0.1;
  slow_move.max_cartesian_speed = 0.1;

  auto & cartesian_max_move = move_configs["cartesian_max_move"];
  cartesian_max_move.velocity_scaling_factor = 0.8;
  cartesian_max_move.acceleration_scaling_factor = 0.4;
  cartesian_max_move.max_cartesian_speed = 0.45;

  auto & cartesian_mid_move = move_configs["cartesian_mid_move"];
  cartesian_mid_move.velocity_scaling_factor = 0.4;
  cartesian_mid_move.acceleration_scaling_factor = 0.2;
  cartesian_mid_move.max_cartesian_speed = 0.25;

  auto & cartesian_slow_move = move_configs["cartesian_slow_move"];
  cartesian_slow_move.velocity_scaling_factor = 0.2;
  cartesian_slow_move.acceleration_scaling_factor = 0.1;
  cartesian_slow_move.max_cartesian_speed = 0.1;

  // We define the joint targets we need for the joint moves as vectors of doubles.
  // Be careful that the number of values must match the number of DOF of the robot (here, 6 DOF)
  std::vector<double> joint_rest =
  {
    0.0, -0.785, 0.785, 0.0, 1.57, 0.0
  };

  // Compose the TCP name:
  std::string tcp_frame_name = rp.prefix + rp.tcp_frame;

  std::vector<Move> rest_position =
  {
    {
      rp.prefix, tcp_frame_name, "joint", max_move, "", joint_rest
    },
  };

  // Sequences for Pick/Drop/Homing
  std::vector<Move> pick_sequence =
  {
    {
      rp.prefix, tcp_frame_name, "pose", mid_move, "approach_pick_target_key"
    },
    {
      rp.prefix, tcp_frame_name, "cartesian", cartesian_slow_move, "pick_target_key"
    },
  };

  std::vector<Move> drop_sequence =
  {
    {
      rp.prefix, tcp_frame_name, "cartesian", cartesian_mid_move, "approach_pick_target_key"
    },
    {
      rp.prefix, tcp_frame_name, "pose", max_move, "approach_drop_target_key"
    },
    {
      rp.prefix, tcp_frame_name, "cartesian", cartesian_slow_move, "drop_target_key"
    },
  };

  std::vector<Move> home_position =
  {
    {
      rp.prefix, tcp_frame_name, "pose", mid_move, "approach_drop_target_key"
    },
    {
      rp.prefix, tcp_frame_name, "named", max_move, "",
      {
      }, "home"
    },
  };

  // Build move sequence blocks
  std::string to_rest_xml = buildMoveXML
                              (rp.prefix,
                              rp.prefix + "toRest",
                              rest_position,
                              blackboard,
                              false,
                              3);

  std::string pick_object_xml = buildMoveXML
                                  (rp.prefix,
                                  rp.prefix + "pick",
                                  pick_sequence,
                                  blackboard,
                                  false,
                                  3);

  std::string drop_object_xml = buildMoveXML
                                  (rp.prefix,
                                  rp.prefix + "drop",
                                  drop_sequence,
                                  blackboard,
                                  false,
                                  3);

  std::string to_home_xml = buildMoveXML
                              (rp.prefix,
                              rp.prefix + "home",
                              home_position,
                              blackboard,
                              false,
                              3);

  // ----------------------------------------------------------------------------
  // 4. Build object pose retrieval elements
  // ----------------------------------------------------------------------------

  blackboard->set<bool>
    ("pick_pose_validity_key",
    false);

  std::string check_pick_pose_validity_xml = buildCheckKeyBool
                                               (rp.prefix,
                                               "CheckPickPoseValidity",
                                               "pick_pose_validity_key",
                                               true);
  std::string wait_for_pick_pose_valid_xml = buildWaitForKeyBool
                                               (rp.prefix,
                                               "WaitForPickPoseValid",
                                               "pick_pose_validity_key",
                                               true);
  std::string wait_for_pick_pose_invalid_xml = buildWaitForKeyBool
                                                 (rp.prefix,
                                                 "WaitForPickPoseInvalid",
                                                 "pick_pose_validity_key",
                                                 false);
  std::string set_pick_pose_valid_xml = buildSetKeyBool
                                          (rp.prefix,
                                          "SetPickPoseValid",
                                          "pick_pose_validity_key",
                                          true);
  std::string set_pick_pose_invalid_xml = buildSetKeyBool
                                            (rp.prefix,
                                            "SetPickPoseInvalid",
                                            "pick_pose_validity_key",
                                            false);

  blackboard->set<bool>
    ("graspable_created_key",
    false);

  std::string check_graspable_created_xml = buildCheckKeyBool
                                              (rp.prefix,
                                              "CheckGraspableCreated",
                                              "graspable_created_key",
                                              true);
  std::string wait_for_graspable_created_xml = buildWaitForKeyBool
                                                 (rp.prefix,
                                                 "WaitForGraspableCreated",
                                                 "graspable_created_key",
                                                 true);
  std::string wait_for_graspable_absent_xml = buildWaitForKeyBool
                                                (rp.prefix,
                                                "WaitForGraspableAbsent",
                                                "graspable_created_key",
                                                false);
  std::string set_graspable_created_xml = buildSetKeyBool
                                            (rp.prefix,
                                            "SetGraspableCreated",
                                            "graspable_created_key",
                                            true);
  std::string set_graspable_deleted_xml = buildSetKeyBool
                                            (rp.prefix,
                                            "SetGraspableDeleted",
                                            "graspable_created_key",
                                            false);

  // Let's build the full sequence in logically separated blocks:
  std::string spawn_fixed_objects_xml = sequenceWrapperXML
                                          ("SpawnFixedObjects",
  {
    ground.init_xml,
    wall.init_xml
  });

  std::vector<double> pick_transform =
  {
    -0.015, 0.0, 0.01, 0.0, 0.0, 0.0
  };
  std::vector<double> approach_transform =
  {
    -0.015, 0.0, -0.08, 0.0, 0.0, 0.0
  };
  // Bounds vector [min_x, min_y, min_z, max_x, max_y, max_z] for validating FoundationPose pose
  // Adjust these as needed for your workspace
  std::vector<double> bounds =
  {
    -0.20, -0.40, 0.00, 0.20, -0.10, 0.20
  };

  std::string foundation_pose_sequence_xml = buildFoundationPoseSequenceXML
                                               ("UpdateFoundationPose",
                                               "/output",
                                               pick_transform,
                                               approach_transform,
                                               0.0,
                                               30.0,
                                               "pick_target_key",
                                               "approach_pick_target_key",
                                               "foundationpose_header_key",
                                               "graspable_pose_key",
                                               true,
                                               0.011,
                                               true,
                                               true,
                                               true,
                                               bounds);

  blackboard->set<Pose>
    ("previous_graspable_pose_key",
    Pose
      ());
  std::string copy_current_object_pose_xml = buildCopyPoseXML
                                               (rp.prefix,
                                               "PreviousObjFPPose",
                                               "graspable_pose_key",
                                               "previous_graspable_pose_key");
  std::string check_previous_pose_distance_xml = buildCheckPoseDistanceXML
                                                 (
    "CheckPreviousPoseDistance",
    "graspable_pose_key",
    "previous_graspable_pose_key",
    0.01);

  // Setting commands for gripper open/close
  std::string move_gripper_close_xml =
    "<PublishJointStateAction topic=\"" + rp.gripper_action_server +
    "\" joint_names=\"[right_finger_joint]\" joint_positions=\"[0.0]\" joint_efforts=\"[-5.0]\" />";
  move_gripper_close_xml +=
    "<Delay delay_msec=\"1000\">" +
    std::string
      ("<PublishJointStateAction topic=\"") + rp.gripper_action_server +
    "\" joint_names=\"[right_finger_joint]\" joint_positions=\"[0.0]\" joint_efforts=\"[-2.0]\" />"
    +
    "</Delay>";

  std::string move_gripper_open_xml =
    "<PublishJointStateAction topic=\"" + rp.gripper_action_server +
    "\" joint_names=\"[right_finger_joint]\" joint_positions=\"[0.0081]\" joint_efforts=\"[5.0]\" />";
  move_gripper_open_xml +=
    "<Delay delay_msec=\"1000\">" +
    std::string
      ("<PublishJointStateAction topic=\"") + rp.gripper_action_server +
    "\" joint_names=\"[right_finger_joint]\" joint_positions=\"[0.0081]\" joint_efforts=\"[2.0]\" />"
    +
    "</Delay>";

  std::string wait_for_robot_start_execution_xml = buildWaitForKeyBool
                                                     ("",
                                                     "robot_start_execution",
                                                     rp.prefix + "stop_execution",
                                                     false);

  std::string retry_forever_foundationpose_xml = retrySequenceWrapperXML
                                                   ("GetFoundationposeOutput",
  {
    copy_current_object_pose_xml,
    foundation_pose_sequence_xml,
    std::string
      ("<Inverter>") + check_previous_pose_distance_xml + std::string
      ("</Inverter>"),
    set_pick_pose_valid_xml,
  },
                                                   -1);

  std::string spawn_graspable_objects_xml = repeatSequenceWrapperXML
                                              ("SpawnGraspableObjects",
  {
    wait_for_pick_pose_invalid_xml,
    retry_forever_foundationpose_xml,
    wait_for_graspable_absent_xml,
    graspable.add_xml,
    set_graspable_created_xml,
  });

  // Define some semantically relevant sequences for gripper actions
  std::string close_gripper_xml = sequenceWrapperXML
                                    ("CloseGripper",
  {
    move_gripper_close_xml,
    graspable.attach_xml
  });
  std::string open_gripper_xml = sequenceWrapperXML
                                   ("OpenGripper",
  {
    move_gripper_open_xml,
    graspable.detach_xml
  });

  // Let's combine the moves and the gripper actions to pick up and drop down the object
  std::string pick_sequence_xml = sequenceWrapperXML
                                    ("PickSequence",
  {
    pick_object_xml,
    close_gripper_xml
  });
  std::string drop_sequence_xml = sequenceWrapperXML
                                    ("DropSequence",
  {
    drop_object_xml,
    open_gripper_xml
  });

  std::string reset_graspable_objects_xml = sequenceWrapperXML
                                              ("reset_graspable_objects",
  {
    open_gripper_xml,
    graspable.remove_xml,
    set_graspable_deleted_xml,
    set_pick_pose_invalid_xml,
  });

  // ----------------------------------------------------------------------------
  // 5. Assembling the tree
  // ----------------------------------------------------------------------------

  std::string startup_sequence_xml = sequenceWrapperXML
                                       ("StartUpSequence",
  {
    spawn_fixed_objects_xml,
    wait_for_robot_start_execution_xml,
    reset_graspable_objects_xml,
    to_rest_xml
  });

  // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other
  // children
  std::string robot_cycle_wrapper_xml = repeatSequenceWrapperXML
                                          ("RobotCycle",
  {
    // spawn_graspable_objects_xml, //< Acquire detection and spawn planning object
    move_gripper_open_xml,                //< Ensure gripper open before pick
    wait_for_pick_pose_valid_xml,             //<
    pick_sequence_xml,                    //< Pick sequence
    set_pick_pose_invalid_xml,             //<
    drop_sequence_xml,                    //< Drop sequence
    graspable.remove_xml,                 //< Remove from planning scene
    set_graspable_deleted_xml,             //<
  },                                      //<
                                          -1);                              //< num_cycles=-1 for
                                                                            // infinite

  std::string parallel_sequences_xml = parallelWrapperXML
                                         ("ParallelWrapper",
  {
    spawn_graspable_objects_xml,
    robot_cycle_wrapper_xml
  },
                                         2,
                                         2);

  std::string retry_forever_wrapper_xml = retrySequenceWrapperXML
                                            ("ResetHandler",
  {
    startup_sequence_xml,
    parallel_sequences_xml
  },
                                            -1);

  // GlobalMasterSequence with RepeatForever as child to set BehaviorTree ID and root
  // main_tree_to_execute in the XML
  std::string master_body = sequenceWrapperXML
                              ("GlobalMasterSequence",
  {
    retry_forever_wrapper_xml
  });

  // Create the MasterTree
  std::string final_tree_xml = mainTreeWrapperXML
                                 ("MasterTree",
                                 master_body);

  // ----------------------------------------------------------------------------
  // 6. Setting up the overall cycle
  // ----------------------------------------------------------------------------

  RCLCPP_INFO
    (node->get_logger
      (),
    "=== Programmatically Generated Tree XML ===\n%s",
    final_tree_xml.c_str
      ());

  // Register node types
  BT::BehaviorTreeFactory factory;
  registerAllNodeTypes
    (factory);

  // Create the tree from final_tree_xml
  BT::Tree tree;
  try {
    tree = factory.createTreeFromText
             (final_tree_xml,
             blackboard);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR
      (node->get_logger
        (),
      "Failed to create tree: %s",
      ex.what
        ());
    return 1;
  }

  // ZMQ publisher (optional, to visualize in Groot)
  BT::PublisherZMQ publisher(tree);

  // Create the HMI Service Node and pass the same blackboard ***
  auto hmi_node = std::make_shared<manymove_cpp_trees::HMIServiceNode>
                    ("hmi_service_node",
                    blackboard,
                    keys);
  RCLCPP_INFO
    (node->get_logger
      (),
    "HMI Service Node instantiated.");

  // Create a MultiThreadedExecutor so that both nodes can be spun concurrently.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node
    (node);
  executor.add_node
    (hmi_node);

  // Tick the tree in a loop.
  rclcpp::Rate rate(100);
  while (rclcpp::ok
           ()) {
    executor.spin_some
      ();
    BT::NodeStatus status = tree.tickRoot
                              ();

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO
        (node->get_logger
          (),
        "BT ended SUCCESS.");
      break;
    }
    else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR
        (node->get_logger
          (),
        "BT ended FAILURE.");
      break;
    }
    rate.sleep
      ();
  }

  tree.rootNode
    ()->halt
    ();
  rclcpp::shutdown
    ();
  return 0;
}
