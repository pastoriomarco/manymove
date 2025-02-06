#ifndef MANYMOVE_CPP_TREES_MOVE_HPP
#define MANYMOVE_CPP_TREES_MOVE_HPP

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "manymove_planner/msg/movement_config.hpp"
#include "manymove_planner/msg/move_manipulator_goal.hpp"

namespace manymove_cpp_trees
{
    inline manymove_planner::msg::MovementConfig defaultMovementConfig()
    {
        manymove_planner::msg::MovementConfig config;
        config.velocity_scaling_factor = 1.0;
        config.acceleration_scaling_factor = 1.0;
        config.step_size = 0.01;
        config.jump_threshold = 0.0;
        config.max_cartesian_speed = 0.5;
        config.max_exec_tries = 5;
        config.plan_number_target = 8;
        config.plan_number_limit = 32;
        config.smoothing_type = "time_optimal";
        return config;
    }

    // ----------------------------------------------------------------------------
    // Setup functions
    // ----------------------------------------------------------------------------

    /**
     * @brief Return some standard MovementConfig presets (max_move, mid_move, slow_move).
     */
    inline std::unordered_map<std::string, manymove_planner::msg::MovementConfig>
    defineMovementConfigs()
    {
        using manymove_planner::msg::MovementConfig;

        MovementConfig max_move_config;
        max_move_config.velocity_scaling_factor = 1.0;
        max_move_config.acceleration_scaling_factor = 1.0;
        max_move_config.step_size = 0.01;
        max_move_config.jump_threshold = 0.0;
        max_move_config.max_cartesian_speed = 0.5;
        max_move_config.max_exec_tries = 5;
        max_move_config.plan_number_target = 8;
        max_move_config.plan_number_limit = 32;
        max_move_config.smoothing_type = "time_optimal";

        MovementConfig mid_move_config = max_move_config;
        mid_move_config.velocity_scaling_factor /= 2.0;
        mid_move_config.acceleration_scaling_factor /= 2.0;
        mid_move_config.max_cartesian_speed = 0.2;

        MovementConfig slow_move_config = max_move_config;
        slow_move_config.velocity_scaling_factor /= 4.0;
        slow_move_config.acceleration_scaling_factor /= 4.0;
        slow_move_config.max_cartesian_speed = 0.05;

        return {
            {"max_move", max_move_config},
            {"mid_move", mid_move_config},
            {"slow_move", slow_move_config}};
    }

    /**
     * @struct Move
     * @brief Represents a single move command with a specified type and configuration.
     *
     * This struct is used to store details of a single move, including its type
     * (e.g., "pose", "joint", "named", or "cartesian"), any relevant pose or joint data,
     * and the associated MovementConfig.
     */
    struct Move
    {
        std::string type;                             ///< The movement type
        std::string pose_key;                         ///< Blackboard key for dynamic pose
        std::vector<double> joint_values;             ///< Joint values for "joint" type.
        std::string named_target;                     ///< Named target for "named" type.
        manymove_planner::msg::MovementConfig config; ///< Movement configuration parameters.
        std::vector<double> start_joint_values;       ///< Starting joint values for planning.

        Move(const std::string &type,
             const std::string &pose_key = "",
             const std::vector<double> &joint_values = {},
             const std::string &named_target = "",
             const manymove_planner::msg::MovementConfig &config = defaultMovementConfig(),
             const std::vector<double> &start_joint_values = {})
            : type(type),
              pose_key(pose_key),
              joint_values(joint_values),
              named_target(named_target),
              config(config),
              start_joint_values(start_joint_values)
        {
        }

        manymove_planner::msg::MoveManipulatorGoal to_move_manipulator_goal() const
        {
            manymove_planner::msg::MoveManipulatorGoal goal;
            goal.movement_type = type;

            if (type == "pose" || type == "cartesian")
            {
                // Retrieve pose from blackboard using pose_key
                // This will be handled in the PlanningAction node
                // goal.pose_target = pose_target;
            }
            else if (type == "joint")
            {
                goal.joint_values = joint_values;
            }
            else if (type == "named")
            {
                goal.named_target = named_target;
            }
            goal.start_joint_values = start_joint_values;
            goal.config = config;
            return goal;
        }
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_MOVE_HPP
