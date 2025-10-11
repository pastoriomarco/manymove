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

// Compatibility include header to bridge MoveIt .h vs .hpp changes across ROS distros.
// Prefer .hpp if available (Jazzy), otherwise fall back to .h (Humble).

#pragma once

// MoveGroupInterface
#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
 # include <moveit/move_group_interface/move_group_interface.hpp>
#else
 # include <moveit/move_group_interface/move_group_interface.h>
#endif

// PlanningSceneInterface
#if __has_include(<moveit/planning_scene_interface/planning_scene_interface.hpp>)
 # include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#else
 # include <moveit/planning_scene_interface/planning_scene_interface.h>
#endif

// RobotTrajectory
#if __has_include(<moveit/robot_trajectory/robot_trajectory.hpp>)
 # include <moveit/robot_trajectory/robot_trajectory.hpp>
#else
 # include <moveit/robot_trajectory/robot_trajectory.h>
#endif

// Trajectory processing: TOTG and Ruckig
#if __has_include(<moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>)
 # include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#else
 # include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#endif

#if __has_include(<moveit/trajectory_processing/ruckig_traj_smoothing.hpp>)
 # include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
#else
 # include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#endif

// Planning Scene + Monitor + Collision common
#if __has_include(<moveit/planning_scene/planning_scene.hpp>)
 # include <moveit/planning_scene/planning_scene.hpp>
#else
 # include <moveit/planning_scene/planning_scene.h>
#endif

#if __has_include(<moveit/planning_scene_monitor/planning_scene_monitor.hpp>)
 # include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#else
 # include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#endif

#if __has_include(<moveit/collision_detection/collision_common.hpp>)
 # include <moveit/collision_detection/collision_common.hpp>
#else
 # include <moveit/collision_detection/collision_common.h>
#endif

// RobotModel
#if __has_include(<moveit/robot_model/robot_model.hpp>)
 # include <moveit/robot_model/robot_model.hpp>
#else
 # include <moveit/robot_model/robot_model.h>
#endif

// MoveItCpp + PlanningPipeline + PlanningComponent
#if __has_include(<moveit/moveit_cpp/moveit_cpp.hpp>)
 # include <moveit/moveit_cpp/moveit_cpp.hpp>
#else
 # include <moveit/moveit_cpp/moveit_cpp.h>
#endif

#if __has_include(<moveit/planning_pipeline/planning_pipeline.hpp>)
 # include <moveit/planning_pipeline/planning_pipeline.hpp>
#else
 # include <moveit/planning_pipeline/planning_pipeline.h>
#endif

#if __has_include(<moveit/moveit_cpp/planning_component.hpp>)
 # include <moveit/moveit_cpp/planning_component.hpp>
#else
 # include <moveit/moveit_cpp/planning_component.h>
#endif

// Cartesian Interpolator
#if __has_include(<moveit/robot_state/cartesian_interpolator.hpp>)
 # include <moveit/robot_state/cartesian_interpolator.hpp>
#else
 # include <moveit/robot_state/cartesian_interpolator.h>
#endif
