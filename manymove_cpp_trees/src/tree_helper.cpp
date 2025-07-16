#include "manymove_cpp_trees/tree_helper.hpp"
#include <sstream>
#include <rclcpp/rclcpp.hpp>

// A static global counter to ensure unique move IDs across the entire tree
static int g_global_move_id = 0;

namespace manymove_cpp_trees
{
    // --------------------------------------------------------------------------
    // High level helper for objects
    // --------------------------------------------------------------------------

    ObjectSnippets createObjectSnippets(BT::Blackboard::Ptr blackboard,
                                        std::vector<manymove_cpp_trees::BlackboardEntry> &keys,
                                        const std::string &name,
                                        const std::string &shape,
                                        const geometry_msgs::msg::Pose &pose,
                                        const std::vector<double> &dimensions,
                                        const std::string &mesh_file,
                                        const std::vector<double> &scale,
                                        const std::string &link_name_key,
                                        const std::string &touch_links_key)
    {
        ObjectSnippets snip;

        std::string id_key = name + "_key";
        std::string shape_key = name + "_shape_key";
        std::string pose_key = name + "_pose_key";
        std::string dim_key = name + "_dimension_key";
        std::string scale_key = name + "_scale_key";
        std::string mesh_key = name + "_file_key";

        // Fail if the object was already defined
        std::string tmp;
        if (blackboard->get(id_key, tmp))
        {
            throw BT::RuntimeError("Object '" + name + "' already exists");
        }

        // Input validation
        if (shape == "box" || shape == "cylinder" || shape == "sphere")
        {
            if (dimensions.empty())
            {
                throw BT::RuntimeError("createObjectSnippets: missing dimensions for primitive '" + shape + "'");
            }

            bool size_ok = (shape == "box" && dimensions.size() == 3) ||
                           (shape == "cylinder" && dimensions.size() == 2) ||
                           (shape == "sphere" && dimensions.size() == 1);

            if (!size_ok)
            {
                throw BT::RuntimeError("createObjectSnippets: wrong number of dimensions for '" + shape + "'");
            }

            for (double d : dimensions)
            {
                if (d <= 0.0)
                {
                    throw BT::RuntimeError("createObjectSnippets: dimension values must be positive");
                }
            }
        }
        else if (shape == "mesh")
        {
            if (mesh_file.empty())
            {
                throw BT::RuntimeError("createObjectSnippets: mesh_file must be provided for mesh shape");
            }
        }
        else
        {
            throw BT::RuntimeError("createObjectSnippets: unsupported shape '" + shape + "'");
        }

        if (scale.size() != 3)
        {
            throw BT::RuntimeError("createObjectSnippets: scale vector must contain 3 elements");
        }
        for (double s : scale)
        {
            if (s < 0.0)
            {
                throw BT::RuntimeError("createObjectSnippets: scale values must be non-negative");
            }
        }

        blackboard->set(id_key, name);
        blackboard->set(shape_key, shape);

        blackboard->set(pose_key, pose);
        keys.push_back({pose_key, "pose"});

        if (!dimensions.empty())
        {
            blackboard->set(dim_key, dimensions);
        }

        if (!mesh_file.empty())
        {
            blackboard->set(mesh_key, mesh_file);
        }

        blackboard->set(scale_key, scale);
        keys.push_back({scale_key, "double_array"});

        snip.check_xml = buildObjectActionXML("check_" + name, createCheckObjectExists(id_key));
        snip.add_xml = buildObjectActionXML("add_" + name,
                                            createAddObject(id_key,
                                                            shape_key,
                                                            dimensions.empty() ? "" : dim_key,
                                                            pose_key,
                                                            scale_key,
                                                            mesh_file.empty() ? "" : mesh_key));
        snip.init_xml = fallbackWrapperXML("init_" + name + "_obj", {snip.check_xml, snip.add_xml});
        snip.remove_xml = fallbackWrapperXML("remove_" + name + "_obj_always_success",
                                             {buildObjectActionXML("remove_" + name,
                                                                   createRemoveObject(id_key)),
                                              "<AlwaysSuccess />"});

        snip.attach_xml = buildObjectActionXML("attach_" + name,
                                               createAttachObject(id_key, link_name_key, touch_links_key));
        snip.detach_xml = fallbackWrapperXML("detach_" + name + "_always_success",
                                             {buildObjectActionXML("detach_" + name,
                                                                   createDetachObject(id_key, link_name_key)),
                                              "<AlwaysSuccess />"});

        return snip;
    }

    // ----------------------------------------------------------------------------
    // Builder functions to build xml tree snippets programmatically
    // ----------------------------------------------------------------------------

    std::string buildMoveXML(const std::string &robot_prefix,
                             const std::string &node_prefix,
                             const std::vector<Move> &moves,
                             BT::Blackboard::Ptr blackboard,
                             bool reset_trajs)
    {
        std::ostringstream xml;

        // Use the current global move ID as the block start
        int blockStartID = g_global_move_id;

        // Collect the move IDs for later reset
        std::vector<int> move_ids;
        move_ids.reserve(moves.size());

        // Build a Sequence node that will run each move in order
        std::ostringstream execution_seq;
        execution_seq << "    <Sequence name=\"ExecutionSequence_" << node_prefix << "_" << blockStartID << "\">\n";

        for (const auto &move : moves)
        {
            int this_move_id = g_global_move_id; // unique ID for this move
            move_ids.push_back(this_move_id);

            // Check that the move's robot prefix is compatible
            if (!move.robot_prefix.empty() && (move.robot_prefix != robot_prefix))
            {
                RCLCPP_ERROR(rclcpp::get_logger("bt_client_node"),
                             "buildMoveXML: Move has prefix=%s, but user gave robot_prefix=%s: INVALID MOVE.",
                             move.robot_prefix.c_str(), robot_prefix.c_str());
                return "<INVALID TREE: MISMATCHING ROBOT PREFIX>";
            }

            // Populate the blackboard with the Move struct under key "move_<id>"
            std::string key = "move_" + std::to_string(this_move_id);
            blackboard->set(key, std::make_shared<Move>(move));
            blackboard->set("trajectory_" + std::to_string(this_move_id), moveit_msgs::msg::RobotTrajectory());

            RCLCPP_INFO(rclcpp::get_logger("bt_client_node"), "BB set: %s", key.c_str());

            // Build a RetryPauseAbort node that wraps a single MoveManipulatorAction.
            // This node is expected to either execute an existing trajectory or trigger a re–plan.
            execution_seq << "    <RetryPauseResetNode name=\"StopSafe_Retry_" << this_move_id << "\""
                          << " collision_detected=\"{" << robot_prefix << "collision_detected}\""
                          << " stop_execution=\"{" << robot_prefix << "stop_execution}\""
                          << " reset=\"{" << robot_prefix << "reset}\""
                          << " robot_prefix=\"" << robot_prefix << "\">\n"
                          << "    <Sequence>\n"
                          << "      <MoveManipulatorAction"
                          << " name=\"MoveManip_" << this_move_id << "\""
                          << " robot_prefix=\"" << robot_prefix << "\""
                          << " move_id=\"" << this_move_id << "\""
                          << " trajectory=\"{trajectory_" << this_move_id << "}\""
                          << " pose_key=\"" << move.pose_key << "\""
                          << " collision_detected=\"{" << robot_prefix << "collision_detected}\""
                          << " invalidate_traj_on_exec=\"" << (reset_trajs ? "true" : "false") << "\" "
                          << " stop_execution=\"{" << robot_prefix << "stop_execution}\""
                          << "/>\n"
                          << "    </Sequence>\n"
                          << "    </RetryPauseResetNode>\n";

            // Increment the global ID for the next move.
            g_global_move_id++;
        }

        execution_seq << "    </Sequence>\n";

        // Append a ResetTrajectories node if reset_trajs is true.
        if (reset_trajs)
        {
            execution_seq << "    <ResetTrajectories name=\"ResetTrajectories_" << node_prefix << "_" << blockStartID << "\" move_ids=\"";
            for (size_t i = 0; i < move_ids.size(); i++)
            {
                execution_seq << move_ids[i];
                if (i < move_ids.size() - 1)
                    execution_seq << ",";
            }
            execution_seq << "\"/>\n";
        }

        return execution_seq.str();
    }

    std::string buildObjectActionXML(const std::string &node_prefix, const ObjectAction &action)
    {
        // Generate a unique node name using the node_prefix and object_id
        std::string node_name = node_prefix + "_" + objectActionTypeToString(action.type);

        // Start constructing the XML node
        std::ostringstream xml;
        xml << "<" << objectActionTypeToString(action.type) << " ";
        xml << "name=\"" << node_name << "\" ";
        xml << "object_id=\"{" << action.object_id_key_st << "}\" ";

        // Handle different action types
        switch (action.type)
        {
        case ObjectActionType::ADD:
        {
            // Shape attribute
            xml << "shape=\"{" << action.shape_key_st << "}\" ";
            // Primitive-specific attributes
            xml << "dimensions=\"{" << action.dimensions_key_d_a << "}\" ";
            // Mesh-specific attributes
            xml << "mesh_file=\"{" << action.mesh_file_key_st << "}\" ";
            xml << "scale_mesh=\"{" << action.scale_key_d_a << "}\" ";
            // When a key is provided, use it as a reference.
            xml << "pose=\"{" << action.pose_key << "}\" ";

            break;
        }
        case ObjectActionType::REMOVE:
        {
            // No additional attributes needed
            break;
        }
        case ObjectActionType::ATTACH:
        {
            // Link name and attach flag
            xml << "link_name=\"{" << action.link_name_key_st << "}\" ";
            xml << "attach=\"" << "true" << "\" ";

            if (!action.touch_links_key_st_a.empty())
            {
                xml << "touch_links=\"{" << action.touch_links_key_st_a << "}\" ";
            }
            break;
        }
        case ObjectActionType::DETACH:
        {
            // Link name and attach flag
            xml << "link_name=\"{" << action.link_name_key_st << "}\" ";
            xml << "attach=\"" << "false" << "\" ";

            // If touch_links is not empty, serialize it as a comma-separated list.
            if (!action.touch_links_key_st_a.empty())
            {
                xml << "touch_links=\"{" << action.touch_links_key_st_a << "}\" ";
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
            xml << "pre_transform_xyz_rpy=\"{" << action.pre_transform_xyz_rpy_key_d_a << "}\" ";
            xml << "post_transform_xyz_rpy=\"{" << action.post_transform_xyz_rpy_key_d_a << "}\" ";
            xml << "link_name=\"{" << action.link_name_key_st << "}\" ";

            xml << "pose_key=\"" << action.pose_key << "\" ";
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
        xml << " success=\"{" << robot_prefix << io_type << "_" << ionum << "_success" << "}\"";

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
        xml << " value=\"{" << robot_prefix << io_type << "_" << ionum << "}\"";

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

        // Build the CheckKeyBoolValue node
        std::ostringstream inner_xml;
        inner_xml << "<CheckKeyBoolValue"
                  << " key=\"" << robot_prefix << io_type << "_" << ionum << "\""
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

    std::string buildWaitForObject(const std::string &robot_prefix,
                                   const std::string &node_prefix,
                                   const std::string &object_id_key,
                                   const bool exists,
                                   const int timeout_ms,
                                   const int poll_rate_ms)
    {
        // Construct a unique name for the node
        std::string node_name = robot_prefix + node_prefix + "_WaitForObject";

        std::ostringstream xml;
        xml << "<WaitForObjectAction"
            << " name=\"" << node_name << "\""
            << " object_id=\"{" << object_id_key << "}\""
            << " exists=\"" << (exists ? "true" : "false") << "\""
            << " timeout=\"" << ((timeout_ms > 0) ? (static_cast<double>(timeout_ms) / 1000.0) : 0.0) << "\""
            << " poll_rate=\"" << ((poll_rate_ms > 0) ? (static_cast<double>(poll_rate_ms) / 1000.0) : 0.0) << "\""
            << "/>";

        return xml.str();
    }

    std::string buildCheckKeyBool(const std::string &node_prefix,
                                  const std::string &key,
                                  const bool &value)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_CheckKey";

        // Build the CheckKeyBoolValue node
        std::ostringstream xml;

        xml << "<CheckKeyBoolValue"
            << " key=\"" << key << "\""
            << " value=\"" << (value ? "true" : "false") << "\" />";

        return xml.str();
    }

    std::string buildWaitForKeyBool(const std::string &robot_prefix,
                                    const std::string &node_prefix,
                                    const std::string &key_id,
                                    const bool &expected_value,
                                    const int timeout_ms,
                                    const int poll_rate_ms)
    {
        // Construct a unique name for the node
        std::string node_name = robot_prefix + node_prefix + "_WaitForKey";
        double timeout_sec = (timeout_ms <= 0) ? 0.0 : (double)timeout_ms / 1000.0;
        double poll_sec = (poll_rate_ms <= 0) ? 0.25 : (double)poll_rate_ms / 1000.0;

        std::ostringstream xml;
        xml << "<WaitForKeyBool"
            << " name=\"" << node_name << "\""
            << " key=\"" << key_id << "\""
            << " expected_value=\"" << (expected_value ? "true" : "false") << "\""
            << " timeout=\"" << timeout_sec << "\""
            << " poll_rate=\"" << poll_sec << "\""
            << "/>";
        return xml.str();
    }

    std::string buildSetKeyBool(const std::string &robot_prefix,
                                const std::string &node_prefix,
                                const std::string &key,
                                const bool &value)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_SetKey";

        // Build the XML snippet
        std::ostringstream xml;
        xml << "<SetKeyBoolValue "
            << "name=\"" << node_name << "\" "
            << "key=\"" << key << "\" "
            << "value=\"" << (value ? "true" : "false") << "\"/>";

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

            {xml.str(), buildSetKeyBool(robot_prefix, node_prefix, robot_prefix + "stop_execution", true)});

        return xml.str();
    }

    std::string buildGetLinkPoseXML(const std::string &node_prefix,
                                    const std::string &link_name_key,
                                    const std::string &pose_key,
                                    const std::string &ref_frame_key,
                                    const std::string &pre_key,
                                    const std::string &post_key)
    {
        std::ostringstream xml;
        xml << "<GetLinkPoseAction name=\"" << node_prefix << "_GetLinkPose\" "
            << "link_name=\"{" << link_name_key << "}\" "
            << "reference_frame=\"{" << ref_frame_key << "}\" "
            << "pre_transform_xyz_rpy=\"{" << pre_key << "}\" "
            << "post_transform_xyz_rpy=\"{" << post_key << "}\" "
            << "pose_key=\"" << pose_key << "\"/>";
        return xml.str();
    }

    std::string buildCheckPoseDistanceXML(const std::string &node_prefix,
                                          const std::string &reference_pose_key,
                                          const std::string &target_pose_key,
                                          double tolerance)
    {
        std::ostringstream xml;

        xml << "<CheckPoseDistance name=\"" << node_prefix << "_CheckPoseDistance\" "
            << "reference_pose_key=\"" << reference_pose_key << "\" "
            << "target_pose_key=\"" << target_pose_key << "\" "
            << "tolerance=\"" << tolerance << "\"/>";

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

    std::string repeatSequenceWrapperXML(const std::string &sequence_name,
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

    std::string repeatFallbackWrapperXML(const std::string &sequence_name,
                                         const std::vector<std::string> &branches,
                                         const int num_cycles)
    {
        std::ostringstream xml;
        xml << "  <Repeat name=\"" << sequence_name << "\" num_cycles=\"" << num_cycles << "\">\n";
        xml << "    <Fallback name=\"" << sequence_name << "_fallback\">\n";
        for (const auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "    </Fallback>\n";
        xml << "  </Repeat>\n";
        return xml.str();
    }

    std::string retrySequenceWrapperXML(const std::string &sequence_name,
                                        const std::vector<std::string> &branches,
                                        const int num_cycles)
    {
        std::ostringstream xml;
        xml << "  <RetryNode name=\"" << sequence_name << "\" num_attempts=\"" << num_cycles << "\">\n";
        xml << "    <Sequence name=\"" << sequence_name << "_sequence\">\n";
        for (const auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "    </Sequence>\n";
        xml << "  </RetryNode>\n";
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
