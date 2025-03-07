#include "manymove_cpp_trees/tree_helper.hpp"
#include <sstream>
#include <rclcpp/rclcpp.hpp>

// A static global counter to ensure unique move IDs across the entire tree
static int g_global_move_id = 0;

namespace manymove_cpp_trees
{
    // ----------------------------------------------------------------------------
    // Builder functions to build xml tree snippets programmatically
    // ----------------------------------------------------------------------------

    std::string buildParallelPlanExecuteXML(const std::string &robot_prefix,
                                            const std::string &node_prefix,
                                            const std::vector<Move> &moves,
                                            BT::Blackboard::Ptr blackboard)
    {
        std::ostringstream xml;

        // The first ID used in this block
        int blockStartID = g_global_move_id;

        // Collect move_ids
        std::vector<int> move_ids;
        move_ids.reserve(moves.size());

        // Planning Sequence
        std::ostringstream planning_seq;
        planning_seq << "    <Sequence name=\"PlanningSequence_" << node_prefix << "_" << blockStartID << "\">\n";

        // Execution Sequence
        std::ostringstream execution_seq;
        execution_seq << "    <Sequence name=\"ExecutionSequence_" << node_prefix << "_" << blockStartID << "\">\n";

        bool first_move = true;

        for (const auto &move : moves)
        {
            int this_move_id = g_global_move_id; // unique ID for this move
            move_ids.push_back(this_move_id);

            // If it's non-empty but doesn't match, log or throw
            if (move.robot_prefix != robot_prefix)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bt_client_node"),
                             "buildParallelPlanExecuteXML: Move has prefix=%s, but user gave robot_prefix=%s: INVALID MOVE.",
                             move.robot_prefix.c_str(), robot_prefix.c_str());
                return "<INVALID TREE: YOU TRIED TO ASSIGN A MOVE TO CREATED WITH ANOTHER ROBOT PREFIX>";
            }

            // Populate the blackboard with the move
            std::string key = "move_" + std::to_string(this_move_id);
            blackboard->set(key, std::make_shared<Move>(move));
            RCLCPP_INFO(rclcpp::get_logger("bt_client_node"),
                        "BB set: %s", key.c_str());

            std::ostringstream partial_planning_seq;

            // create the planning action
            partial_planning_seq << "      <PlanningAction"
                                 << " name=\"PlanMove_" << this_move_id << "\""
                                 << " move_id=\"" << this_move_id << "\""
                                 << " robot_prefix=\"" << robot_prefix << "\""
                                 << " planned_move_id=\"{planned_move_id_" << this_move_id << "}\""
                                 << " trajectory=\"{trajectory_" << this_move_id << "}\""
                                 << " sequential_from_previous=\"" << (first_move ? "false" : "true") << "\""
                                 << " planning_validity=\"{validity_" << this_move_id << "}\""
                                 << " pose_key=\"" << move.pose_key << "\""
                                 << "/>\n";

            // add the planning action to the planning sequence
            planning_seq << partial_planning_seq.str();

            // create the execution action, with a stop-safe retry mechanism and with the planning action as fallback
            execution_seq << "    <RetryPauseAbortNode name=\"StopSafe_Retry_" << this_move_id << "\""
                          << " collision_detected=\"{" << robot_prefix << "collision_detected}\""
                          << " stop_execution=\"{" << robot_prefix << "stop_execution}\""
                          << " abort_mission=\"{" << robot_prefix << "abort_mission}\">\n"
                          << "     <Fallback>\n"
                          << "      <ExecuteTrajectory"
                          << " name=\"ExecMove_" << this_move_id << "\""
                          << " robot_prefix=\"" << robot_prefix << "\""
                          << " planned_move_id=\"{planned_move_id_" << this_move_id << "}\""
                          << " trajectory=\"{trajectory_" << this_move_id << "}\""
                          << " planning_validity=\"{validity_" << this_move_id << "}\""
                          << " invalidate_traj_on_exec=\"true\" "
                          << " collision_detected=\"{" << robot_prefix << "collision_detected}\""
                          << " execution_resumed=\"{" << robot_prefix << "execution_resumed}\""
                          << "/>\n"
                          << "      <ForceFailure>"
                          << "      <PlanningAction"
                          << " name=\"PlanMove_" << this_move_id << "\""
                          << " move_id=\"" << this_move_id << "\""
                          << " robot_prefix=\"" << robot_prefix << "\""
                          << " planned_move_id=\"{planned_move_id_" << this_move_id << "}\""
                          << " trajectory=\"{trajectory_" << this_move_id << "}\""
                          << " sequential_from_previous=\"false\""
                          << " planning_validity=\"{validity_" << this_move_id << "}\""
                          << " pose_key=\"" << move.pose_key << "\""
                          << "/>\n"
                          << "      </ForceFailure>\n"
                          << "     </Fallback>"
                          << "    </RetryPauseAbortNode>\n";

            // increment the global ID for the next move
            g_global_move_id++;
            first_move = false;
        }

        planning_seq << "    </Sequence>\n";
        execution_seq << "    </Sequence>\n";

        // Parallel node
        std::ostringstream parallel_node;
        parallel_node
            << "  <Parallel name=\"ParallelPlanExecute_" << node_prefix << "_" << blockStartID
            << "\" success_threshold=\"2\" failure_threshold=\"1\">\n"
            << planning_seq.str()
            << execution_seq.str()
            << "  </Parallel>\n";

        // ResetTrajectories node
        std::ostringstream reset_node;
        reset_node << "  <ResetTrajectories move_ids=\"";
        for (size_t i = 0; i < move_ids.size(); i++)
        {
            reset_node << move_ids[i];
            if (i != move_ids.size() - 1)
                reset_node << ",";
        }
        reset_node << "\"/>\n";

        // Insert ResetTrajectories first
        xml << reset_node.str();

        // Insert Parallel nodes
        xml << parallel_node.str();

        return xml.str();
    }

    std::string buildSequentialPlanExecuteXML(const std::string &robot_prefix,
                                              const std::string &node_prefix,
                                              const std::vector<Move> &moves,
                                              BT::Blackboard::Ptr blackboard,
                                              bool reset_trajs)
    {
        std::ostringstream xml;

        // The first ID used in this block
        int blockStartID = g_global_move_id;

        // Collect move_ids
        std::vector<int> move_ids;
        move_ids.reserve(moves.size());

        // Execution Sequence with planning fallback
        std::ostringstream execution_seq;
        execution_seq << "    <Sequence name=\"ExecutionSequence_" << node_prefix << "_" << blockStartID << "\">\n";

        for (const auto &move : moves)
        {
            int this_move_id = g_global_move_id; // unique ID for this move
            move_ids.push_back(this_move_id);

            // If it's non-empty but doesn't match, log or throw
            if (move.robot_prefix != robot_prefix)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bt_client_node"),
                             "buildSequentialPlanExecuteXML: Move has prefix=%s, but user gave robot_prefix=%s: INVALID MOVE.",
                             move.robot_prefix.c_str(), robot_prefix.c_str());
                return "<INVALID TREE: YOU TRIED TO ASSIGN A MOVE TO CREATED WITH ANOTHER ROBOT PREFIX>";
            }

            // Populate the blackboard with the move
            std::string key = "move_" + std::to_string(this_move_id);
            blackboard->set(key, std::make_shared<Move>(move));
            RCLCPP_INFO(rclcpp::get_logger("bt_client_node"),
                        "BB set: %s", key.c_str());

            // create the execution action, with a stop-safe retry mechanism and with the planning action as fallback
            execution_seq << "    <RetryPauseAbortNode name=\"StopSafe_Retry_" << this_move_id << "\""
                          << " collision_detected=\"{" << robot_prefix << "collision_detected}\""
                          << " stop_execution=\"{" << robot_prefix << "stop_execution}\""
                          << " abort_mission=\"{" << robot_prefix << "abort_mission}\">\n"
                          << "     <Fallback>\n"
                          << "      <ExecuteTrajectory"
                          << " name=\"ExecMove_" << this_move_id << "\""
                          << " robot_prefix=\"" << robot_prefix << "\""
                          << " planned_move_id=\"{planned_move_id_" << this_move_id << "}\""
                          << " trajectory=\"{trajectory_" << this_move_id << "}\""
                          << " planning_validity=\"{validity_" << this_move_id << "}\""
                          << " invalidate_traj_on_exec=\"" << (reset_trajs ? "true" : "false") << "\" "
                          << " collision_detected=\"{" << robot_prefix << "collision_detected}\""
                          << " execution_resumed=\"{" << robot_prefix << "execution_resumed}\""
                          << "/>\n"
                          << "      <ForceFailure>"
                          << "      <PlanningAction"
                          << " name=\"PlanMove_" << this_move_id << "\""
                          << " move_id=\"" << this_move_id << "\""
                          << " robot_prefix=\"" << robot_prefix << "\""
                          << " planned_move_id=\"{planned_move_id_" << this_move_id << "}\""
                          << " trajectory=\"{trajectory_" << this_move_id << "}\""
                          << " sequential_from_previous=\"false\""
                          << " planning_validity=\"{validity_" << this_move_id << "}\""
                          << " pose_key=\"" << move.pose_key << "\""
                          << "/>\n"
                          << "      </ForceFailure>\n"
                          << "     </Fallback>"
                          << "    </RetryPauseAbortNode>\n";

            // increment the global ID for the next move
            g_global_move_id++;
        }

        execution_seq << "    </Sequence>\n";

        return execution_seq.str();
    }

    std::string buildObjectActionXML(const std::string &node_prefix, const ObjectAction &action)
    {
        // Generate a unique node name using the node_prefix and object_id
        std::string node_name = node_prefix + "_" + action.object_id + "_" + objectActionTypeToString(action.type);

        // Start constructing the XML node
        std::ostringstream xml;
        xml << "<" << objectActionTypeToString(action.type) << " ";
        xml << "name=\"" << node_name << "\" ";
        xml << "object_id=\"" << action.object_id << "\" ";

        // Handle different action types
        switch (action.type)
        {
        case ObjectActionType::ADD:
        {
            // Shape attribute
            xml << "shape=\"" << action.shape << "\" ";

            if (action.shape != "mesh")
            {
                // Serialize dimensions
                std::string dimensions_str = serializeVector(action.dimensions);
                xml << "dimensions=\"" << dimensions_str << "\" ";
            }
            else
            {
                // Mesh-specific attributes
                xml << "mesh_file=\"" << action.mesh_file << "\" ";
                xml << "scale_mesh_x=\"" << action.scale_mesh_x << "\" ";
                xml << "scale_mesh_y=\"" << action.scale_mesh_y << "\" ";
                xml << "scale_mesh_z=\"" << action.scale_mesh_z << "\" ";
            }

            if (!action.pose_key.empty())
            {
                // When a key is provided, use it as a reference.
                xml << "pose=\"{" << action.pose_key << "}\" ";
            }
            else
            {
                std::string pose_str = serializePose(action.pose);
                xml << "pose=\"" << pose_str << "\" ";
            }
            break;
        }
        case ObjectActionType::REMOVE:
        {
            // No additional attributes needed
            break;
        }
        case ObjectActionType::ATTACH:
        case ObjectActionType::DETACH:
        {
            // Link name and attach flag
            xml << "link_name=\"" << action.link_name << "\" ";
            xml << "attach=\"" << (action.attach ? "true" : "false") << "\" ";

            // If touch_links is not empty, serialize it as a comma-separated list.
            if (!action.touch_links.empty())
            {
                xml << "touch_links=\"" << BT::convertToString(action.touch_links) << "\" ";
            }
            break;
        }
        case ObjectActionType::CHECK:
        {
            // No additional attributes needed
            break;
        }
        case ObjectActionType::GET_POSE:
        {
            // Serialize pre_transform_xyz_rpy and post_transform_xyz_rpy
            std::string transform_str = serializeVector(action.pre_transform_xyz_rpy);
            std::string reference_orient_str = serializeVector(action.post_transform_xyz_rpy);
            xml << "pre_transform_xyz_rpy=\"" << transform_str << "\" ";
            xml << "post_transform_xyz_rpy=\"" << reference_orient_str << "\" ";
            xml << "link_name=\"" << action.link_name << "\" ";

            // Serialize pose_key if it's not empty
            if (!action.pose_key.empty())
            {
                xml << "pose_key=\"" << action.pose_key << "\" ";
            }
            break;
        }
        default:
            throw std::invalid_argument("Unsupported ObjectActionType in buildObjectActionXML");
        }

        // Close the XML node
        xml << "/>";

        return xml.str();
    }

    std::string buildSetOutputXML(const std::string &robot_prefix,
                                  const std::string &node_prefix,
                                  const std::string &io_type,
                                  int ionum,
                                  int value)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_SetOutput";

        std::ostringstream xml;
        xml << "<SetOutputAction "
            << "name=\"" << node_name << "\" "
            << "io_type=\"" << io_type << "\" "
            << "ionum=\"" << ionum << "\" "
            << "robot_prefix=\"" << robot_prefix << "\" "
            << "value=\"" << ((value == 0) ? "0" : "1") << "\"";

        // If the user wants the success output on blackboard
        xml << " success=\"{" << io_type << "_" << ionum << "_success" << "}\"";

        xml << "/>";
        return xml.str();
    }

    std::string buildGetInputXML(const std::string &robot_prefix,
                                 const std::string &node_prefix,
                                 const std::string &io_type,
                                 int ionum)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_GetInput";

        std::ostringstream xml;
        xml << "<GetInputAction "
            << "name=\"" << node_name << "\" "
            << "io_type=\"" << io_type << "\" "
            << "ionum=\"" << ionum << "\""
            << "robot_prefix=\"" << robot_prefix << "\" ";

        // If user wants the read value on the blackboard
        xml << " value=\"{" << io_type << "_" << ionum << "}\"";

        // If user wants the success output on the blackboard
        xml << " success=\"{" << io_type << "_" << ionum << "_success" << "}\"";

        xml << "/>";
        return xml.str();
    }

    std::string buildCheckInputXML(const std::string &robot_prefix,
                                   const std::string &node_prefix,
                                   const std::string &io_type,
                                   int ionum,
                                   int value)
    {
        // Construct a node name
        std::string node_name = robot_prefix + node_prefix + "_CheckInput";

        // The value can be 0 or 1, so we trim anything different from 0 or 1. If it's not 0, then it is 1.
        int value_to_check = (value == 0 ? 0 : 1);

        // Build GetInputAction
        std::string check_input_xml = buildGetInputXML(robot_prefix, node_name, io_type, ionum);

        // Build the CheckBlackboardKeyValue node
        std::ostringstream inner_xml;
        inner_xml << "<Condition ID=\"CheckBlackboardKeyValue\""
                  << " key=\"" << io_type << "_" << ionum << "\""
                  << " value=\"" << value_to_check << "\" />";

        // Wrap in a Sequence
        std::ostringstream sequence_xml;
        sequence_xml << sequenceWrapperXML(node_name + "_Sequence", {check_input_xml, inner_xml.str()});

        return sequence_xml.str();
    }

    std::string buildWaitForInput(const std::string &robot_prefix,
                                  const std::string &node_prefix,
                                  const std::string &io_type,
                                  int ionum,
                                  int desired_value,
                                  int timeout_ms,
                                  int poll_rate_ms)
    {
        // Construct an XML snippet for <WaitForInputAction... />
        std::string node_name = robot_prefix + node_prefix + "_WaitForInput";
        double timeout_sec = (timeout_ms <= 0) ? 0.0 : (double)timeout_ms / 1000.0;
        double poll_sec = (poll_rate_ms <= 0) ? 0.25 : (double)poll_rate_ms / 1000.0;

        std::ostringstream xml;
        xml << "<WaitForInputAction"
            << " name=\"" << node_name << "\""
            << " robot_prefix=\"" << robot_prefix << "\""
            << " io_type=\"" << io_type << "\""
            << " ionum=\"" << ionum << "\""
            << " desired_value=\"" << desired_value << "\""
            << " timeout=\"" << timeout_sec << "\""
            << " poll_rate=\"" << poll_sec << "\""
            << "/>";
        return xml.str();
    }

    // std::string buildWaitForObject(const std::string &robot_prefix,
    //                                const std::string &node_prefix,
    //                                const std::string &object_id,
    //                                bool exists,
    //                                int timeout_ms)
    // {
    //     // Construct a node name
    //     std::string node_name = robot_prefix + node_prefix + "_WaitForObject";

    //     // Build GetInputAction
    //     std::string check_obj_xml = buildObjectActionXML(node_name, createCheckObjectExists(object_id));

    //     /// TODO: hardcoded dalay, evaluate if it should be set by user or not:
    //     int delay_ms = 250;

    //     // Check here for details about <Delay> : https://github.com/BehaviorTree/BehaviorTree.CPP/issues/413
    //     std::ostringstream delay_and_fail_xml;
    //     delay_and_fail_xml << "<Delay delay_msec=\"" << delay_ms << "\">\n"
    //                        << "<AlwaysFailure />" << "\n"
    //                        << "</Delay>" << "\n";

    //     std::string fallback_check_or_delay_xml = fallbackWrapperXML((node_name + "_Fallback"), {check_obj_xml, delay_and_fail_xml.str()});

    //     std::ostringstream wait_xml;

    //     // Tree modified after finding this issue:
    //     // https://github.com/BehaviorTree/BehaviorTree.CPP/issues/395
    //     // wait_xml << "<RetryUntilSuccessful name=\"" << node_name << "_Retry\" max_attempts=\"-1\">\n"
    //     //       << fallback_check_or_delay << "\n"
    //     //       << "</RetryUntilSuccessful>";

    //     wait_xml << "<Inverter>\n"
    //              << "<KeepRunningUntilFailure>\n"
    //              << (exists ? "<Inverter>\n" : "\n")
    //              << fallback_check_or_delay_xml << "\n"
    //              << (exists ? "</Inverter>\n" : "\n")
    //              << "</KeepRunningUntilFailure>\n"
    //              << "</Inverter>\n";

    //     if (timeout_ms > 0)
    //     {
    //         std::ostringstream timeout_xml;
    //         timeout_xml << "<Timeout msec=\"" << timeout_ms << "\">\n"
    //                     << wait_xml.str()
    //                     << "</Timeout>";

    //         return sequenceWrapperXML(node_name + "_WaitForObjectTimeout", {timeout_xml.str()});
    //     }

    //     return sequenceWrapperXML(node_name + "_WaitForObject", {wait_xml.str()});
    // }

    std::string buildWaitForObject(const std::string &robot_prefix,
                                   const std::string &node_prefix,
                                   const std::string &object_id,
                                   bool exists,
                                   int timeout_ms,
                                   int poll_rate_ms)
    {
        // Construct a unique name for the node
        // e.g. "R_pickup_WaitForObject"
        std::string node_name = robot_prefix + node_prefix + "_WaitForObject";

        // Now produce a single <WaitForObjectAction> node:
        //   <WaitForObjectAction name="node_name"
        //       object_id="object_id"
        //       exists="true/false"
        //       timeout="X"
        //       poll_rate="0.25" />

        std::ostringstream xml;
        xml << "<WaitForObjectAction"
            << " name=\"" << node_name << "\""
            << " object_id=\"" << object_id << "\""
            << " exists=\"" << (exists ? "true" : "false") << "\""
            << " timeout=\"" << ((timeout_ms > 0) ? (static_cast<double>(timeout_ms) / 1000.0) : 0.0) << "\""
            << " poll_rate=\"" << ((poll_rate_ms > 0) ? (static_cast<double>(poll_rate_ms) / 1000.0) : 0.0) << "\""
            << "/>";

        return xml.str();
    }

    // std::string buildWaitForKey(const std::string &robot_prefix,
    //                             const std::string &node_prefix,
    //                             const std::string &key_id,
    //                             const std::string &expected_value,
    //                             int timeout_ms)
    // {
    //     // Construct a unique node name.
    //     std::string node_name = robot_prefix + node_prefix + "_WaitForKey";

    //     // Create a condition that compares the blackboard key to the expected value.
    //     std::ostringstream condition;
    //     condition << "<Condition ID=\"CheckBlackboardKeyValue\""
    //               << " key=\"" << key_id << "\""
    //               << " value=\"" << expected_value << "\" />";

    //     /// TODO: hardcoded dalay, evaluate if it should be set by user or not:
    //     int delay_ms = 250;

    //     // Check here for details about <Delay> : https://github.com/BehaviorTree/BehaviorTree.CPP/issues/413
    //     std::ostringstream delay_and_fail_xml;
    //     delay_and_fail_xml << "<Delay delay_msec=\"" << delay_ms << "\">\n"
    //                        << "<AlwaysFailure />" << "\n"
    //                        << "</Delay>" << "\n";

    //     std::string fallback_check_or_delay_xml = fallbackWrapperXML((node_name + "_Fallback"), {condition.str(), delay_and_fail_xml.str()});

    //     std::ostringstream wait_xml;

    //     // Tree modified after finding this issue:
    //     // https://github.com/BehaviorTree/BehaviorTree.CPP/issues/395
    //     // wait_xml << "<RetryUntilSuccessful name=\"" << node_name << "_Retry\" max_attempts=\"-1\">\n"
    //     //       << fallback_check_or_delay << "\n"
    //     //       << "</RetryUntilSuccessful>";

    //     wait_xml << "<Inverter>\n"
    //              << "<KeepRunningUntilFailure>\n"
    //              << "<Inverter>\n"
    //              << fallback_check_or_delay_xml << "\n"
    //              << "</Inverter>\n"
    //              << "</KeepRunningUntilFailure>\n"
    //              << "</Inverter>\n";

    //     if (timeout_ms > 0)
    //     {
    //         std::ostringstream timeout_xml;
    //         timeout_xml << "<Timeout msec=\"" << timeout_ms << "\">\n"
    //                     << wait_xml.str()
    //                     << "</Timeout>";

    //         return sequenceWrapperXML(node_name + "_WaitForKeyTimeout", {timeout_xml.str()});
    //     }

    //     return sequenceWrapperXML(node_name + "_WaitForKey", {wait_xml.str()});
    // }

    std::string buildWaitForKey(const std::string &robot_prefix,
                                const std::string &node_prefix,
                                const std::string &key_id,
                                const std::string &expected_value,
                                int timeout_ms,
                                int poll_rate_ms)
    {
        // Similar approach for <WaitForKeyAction... />
        std::string node_name = robot_prefix + node_prefix + "_WaitForKey";
        double timeout_sec = (timeout_ms <= 0) ? 0.0 : (double)timeout_ms / 1000.0;
        double poll_sec = (poll_rate_ms <= 0) ? 0.25 : (double)poll_rate_ms / 1000.0;

        std::ostringstream xml;
        xml << "<WaitForKeyAction"
            << " name=\"" << node_name << "\""
            << " key=\"" << key_id << "\""
            << " expected_value=\"" << expected_value << "\""
            << " timeout=\"" << timeout_sec << "\""
            << " poll_rate=\"" << poll_sec << "\""
            << "/>";
        return xml.str();
    }

    std::string buildSetBlackboardKey(const std::string &robot_prefix,
                                      const std::string &node_prefix,
                                      const std::string &key,
                                      const std::string &value)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_SetKey";

        // Build the XML snippet
        std::ostringstream xml;
        xml << "<SetBlackboardKeyValue "
            << "name=\"" << node_name << "\" "
            << "key=\"" << key << "\" "
            << "value=\"" << value << "\"/>";

        return xml.str();
    }

    std::string buildCheckRobotStateXML(const std::string &robot_prefix,
                                        const std::string &node_prefix,
                                        const std::string &ready_key,
                                        const std::string &err_key,
                                        const std::string &mode_key,
                                        const std::string &state_key,
                                        const std::string &message_key)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_CheckRobotState";

        std::ostringstream xml;
        xml << "<CheckRobotStateAction "
            << "name=\"" << node_name << "\""
            << "robot_prefix=\"" << robot_prefix << "\" ";

        // Optional outputs
        if (!ready_key.empty())
        {
            xml << " ready=\"{" << ready_key << "}\"";
        }
        if (!err_key.empty())
        {
            xml << " err=\"{" << err_key << "}\"";
        }
        if (!mode_key.empty())
        {
            xml << " mode=\"{" << mode_key << "}\"";
        }
        if (!state_key.empty())
        {
            xml << " state=\"{" << state_key << "}\"";
        }
        if (!message_key.empty())
        {
            xml << " message=\"{" << message_key << "}\"";
        }

        xml << "/>";
        return xml.str();
    }

    std::string buildResetRobotStateXML(const std::string &robot_prefix,
                                        const std::string &node_prefix,
                                        const std::string &robot_model)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_ResetRobotState";

        std::ostringstream xml;
        xml << "<ResetRobotStateAction "
            << "name=\"" << node_name << "\""
            << "robot_prefix=\"" << robot_prefix << "\" "
            << "robot_model=\"" << robot_model << "\" ";

        // Output
        xml << " success=\"{" << "robot_state_success" << "}\"";

        xml << "/>";

        return sequenceWrapperXML(
            node_name + "_WaitTimeout",
            {xml.str(), buildStopMotionXML(robot_prefix, node_prefix, 0.25)});

        return xml.str();
    }

    std::string buildStopMotionXML(const std::string &robot_prefix,
                                   const std::string &node_prefix,
                                   double deceleration_time)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_StopMotion";

        std::ostringstream xml;
        xml << "<StopMotionAction "
            << "name=\"" << node_name << "\" "
            << "robot_prefix=\"" << robot_prefix << "\" "
            << "deceleration_time=\"" << deceleration_time << "\" ";

        // Output
        // xml << " success=\"{" << "stop_motion_success" << "}\"";

        xml << "/>";
        return xml.str();
    }

    // ----------------------------------------------------------------------------
    // Wrappers
    // ----------------------------------------------------------------------------

    std::string sequenceWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches)
    {
        std::ostringstream xml;
        xml << "  <Sequence name=\"" << sequence_name << "\">\n";
        for (auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "  </Sequence>\n";
        return xml.str();
    }

    std::string parallelWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches,
                                   const int &success_threshold,
                                   const int &failure_threshold)
    {
        std::ostringstream xml;
        xml << "  <Parallel name=\"" << sequence_name << "\""
            << " success_threshold=\"" << success_threshold << "\">"
            << " failure_threshold=\"" << failure_threshold << "c\">\n";
        for (auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "  </Parallel>\n";
        return xml.str();
    }

    std::string reactiveWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches)
    {
        std::ostringstream xml;
        xml << "  <ReactiveSequence name=\"" << sequence_name << "\">\n";
        for (auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "  </ReactiveSequence>\n";
        return xml.str();
    }

    std::string repeatWrapperXML(const std::string &sequence_name,
                                 const std::vector<std::string> &branches,
                                 const int num_cycles)
    {
        std::ostringstream xml;
        xml << "  <Repeat name=\"" << sequence_name << "\" num_cycles=\"" << num_cycles << "\">\n";
        xml << "    <Sequence name=\"" << sequence_name << "_sequence\">\n";
        for (const auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "    </Sequence>\n";
        xml << "  </Repeat>\n";
        return xml.str();
    }

    std::string fallbackWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches)
    {
        std::ostringstream xml;
        xml << "  <Fallback name=\"" << sequence_name << "\">\n";
        for (auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "  </Fallback>\n";
        return xml.str();
    }

    std::string mainTreeWrapperXML(const std::string &tree_id,
                                   const std::string &content)
    {
        std::ostringstream xml;
        xml << R"(<?xml version="1.0" encoding="UTF-8"?>)" << "\n";
        xml << "<root main_tree_to_execute=\"" << tree_id << "\">\n";
        xml << "  <BehaviorTree ID=\"" << tree_id << "\">\n";
        xml << content << "\n";
        xml << "  </BehaviorTree>\n";
        xml << "</root>\n";
        return xml.str();
    }

    // ----------------------------------------------------------------------------
    // Helper functions
    // ----------------------------------------------------------------------------

    geometry_msgs::msg::Pose createPose(double x, double y, double z,
                                        double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        p.orientation.x = qx;
        p.orientation.y = qy;
        p.orientation.z = qz;
        p.orientation.w = qw;
        return p;
    }

    geometry_msgs::msg::Pose createPoseRPY(const double &x,
                                           const double &y,
                                           const double &z,
                                           const double &roll,
                                           const double &pitch,
                                           const double &yaw)
    {
        auto pose = geometry_msgs::msg::Pose();

        // Set position
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        // Convert roll, pitch, yaw to a quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);

        // Set orientation
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();

        return pose;
    }

    std::string objectActionTypeToString(ObjectActionType type)
    {
        switch (type)
        {
        case ObjectActionType::ADD:
            return "AddCollisionObjectAction";
        case ObjectActionType::REMOVE:
            return "RemoveCollisionObjectAction";
        case ObjectActionType::ATTACH:
        case ObjectActionType::DETACH:
            return "AttachDetachObjectAction";
        case ObjectActionType::CHECK:
            return "CheckObjectExistsAction";
        case ObjectActionType::GET_POSE:
            return "GetObjectPoseAction";
        default:
            throw std::invalid_argument("Unsupported ObjectActionType");
        }
    }

    std::string serializePose(const geometry_msgs::msg::Pose &pose)
    {
        std::ostringstream oss;
        oss << "position: {x: " << pose.position.x
            << ", y: " << pose.position.y
            << ", z: " << pose.position.z
            << "}, orientation: {x: " << pose.orientation.x
            << ", y: " << pose.orientation.y
            << ", z: " << pose.orientation.z
            << ", w: " << pose.orientation.w
            << "}";
        return oss.str();
    }

    std::string serializeVector(const std::vector<double> &vec)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < vec.size(); ++i)
        {
            oss << vec[i];
            if (i != vec.size() - 1)
                oss << ",";
        }
        oss << "]";
        return oss.str();
    }

} // namespace manymove_cpp_trees
