#ifndef MANYMOVE_CPP_TREES_ROBOT_HPP
#define MANYMOVE_CPP_TREES_ROBOT_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/blackboard.h>

namespace manymove_cpp_trees
{
    struct RobotParams
    {
        std::string model;
        std::string prefix;
        std::string tcp_frame;
        std::string gripper_action_server;
        std::vector<std::string> contact_links;
        bool is_real;
    };

    /**
     * @brief Declare and read robot-related parameters from a node, falling back
     *        on the provided values if parameters are not set.
     *
     * @param node_ptr The ROS2 node used to declare and read parameters.
     * @param blackboard The pointer to the blackboard to set system keys.
     * @param ID the postix to each parameter if we have more than one robot.
     * @param model Default robot model, e.g. "lite6".
     * @param prefix Default robot action server prefix, e.g. "R_".
     * @param tcp_frame Default robot TCP frame, e.g. "panda_link8".
     * @param gripper_action_server Default name for the gripper action server, if any.
     * @param contact_links Default contact links to be used for collision checking, etc.
     * @param is_real Whether we are connected to a real robot or simulation by default.
     *
     * @return A RobotParams struct populated from the parameter server or the defaults.
     */
    inline RobotParams defineRobotParams(
        const rclcpp::Node::SharedPtr& node_ptr,
        BT::Blackboard::Ptr blackboard,
        const std::string& ID = "",
        const std::string& model = "",
        const std::string& prefix = "",
        const std::string& tcp_frame = "",
        const std::string& gripper_action_server = "",
        const std::vector<std::string>& contact_links = {},
        bool is_real = false)
    {
        RobotParams rp;

        // Declare and read parameters: "robot_model", "robot_prefix", etc.
        // If the user has set these parameters externally (e.g., via .yaml or launch),
        // we get that value. Otherwise we get the * argument.
        node_ptr->declare_parameter<std::string>("robot_model" + ID, model);
        node_ptr->get_parameter_or<std::string>("robot_model" + ID, rp.model, model);

        node_ptr->declare_parameter<std::string>("robot_prefix" + ID, prefix);
        node_ptr->get_parameter_or<std::string>("robot_prefix" + ID, rp.prefix, prefix);

        node_ptr->declare_parameter<std::string>("tcp_frame" + ID, tcp_frame);
        node_ptr->get_parameter_or<std::string>("tcp_frame" + ID, rp.tcp_frame, tcp_frame);

        node_ptr->declare_parameter<std::string>("gripper_action_server" + ID, gripper_action_server);
        node_ptr->get_parameter_or<std::string>("gripper_action_server" + ID, rp.gripper_action_server, gripper_action_server);

        node_ptr->declare_parameter<std::vector<std::string>>("contact_links" + ID, contact_links);
        node_ptr->get_parameter_or<std::vector<std::string>>("contact_links" + ID, rp.contact_links, contact_links);

        node_ptr->declare_parameter<bool>("is_robot_real" + ID, is_real);
        node_ptr->get_parameter_or<bool>("is_robot_real" + ID, rp.is_real, is_real);

        /**
         * The following keys are important for the execution control logic: they are modified through
         * the HMI services and let you pause/stop, resume or abort/reset execution.
         */
        // Setting blackboard keys to control execution:
        blackboard->set(rp.prefix + "collision_detected", false);
        blackboard->set(rp.prefix + "stop_execution", true);
        blackboard->set(rp.prefix + "reset", false);

        return rp;
    }

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_ROBOT_HPP
