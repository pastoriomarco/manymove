# Copyright 2025 Flexin Group SRL
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Flexin Group SRL nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch description for the bt panda fake scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description entry point."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="panda_arm",
                description="Model of the robot (e.g., lite6, uf850, xarm)",
            ),
            DeclareLaunchArgument(
                "robot_prefix",
                default_value="",
                description="Prefix for the name of the robot arms",
            ),
            DeclareLaunchArgument(
                "tcp_frame",
                default_value="panda_link8",
                description="Name of the link to attach/detach objects to/from",
            ),
            DeclareLaunchArgument(
                "gripper_action_server",
                default_value="/panda_hand_controller/gripper_cmd",
                description="Name of the action server to control the gripper",
            ),
            DeclareLaunchArgument(
                "contact_links",
                default_value='["panda_leftfinger", "panda_rightfinger", "panda_hand"]',
                description="List of links to exclude from collision checking",
            ),
            DeclareLaunchArgument(
                "is_robot_real",
                default_value="false",
                description=(
                    "Set to true if connected with a real robot exposing the "
                    "necessary services needed by manymove_signals"
                ),
            ),
            Node(
                package="manymove_cpp_trees",
                executable="bt_client",
                name="manymove_cpp_trees_single_robot",
                output="screen",
                parameters=[
                    {
                        "robot_model": LaunchConfiguration("robot_model"),
                        "robot_prefix": LaunchConfiguration("robot_prefix"),
                        "tcp_frame": LaunchConfiguration("tcp_frame"),
                        "gripper_action_server": LaunchConfiguration("gripper_action_server"),
                        "contact_links": LaunchConfiguration("contact_links"),
                        "is_robot_real": LaunchConfiguration("is_robot_real"),
                    }
                ],
            ),
        ]
    )
