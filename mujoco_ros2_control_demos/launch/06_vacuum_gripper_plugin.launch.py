# Copyright 2026 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Tutorial 6: Vacuum Gripper Plugin

This tutorial demonstrates VacuumGripperPlugin: one plugin instance per suction
pickup. Each instance owns a pair of MJCF bodies (gripper side + part side) joined
by an inactive weld equality constraint. Calling ~/activate latches a vacuum (it is
rejected unless the bodies are in contact); the weld is active iff the vacuum is on
AND the bodies are in contact, re-evaluated every physics step. ~/release clears
the vacuum; losing contact drops the weld automatically while the vacuum stays
latched, so re-establishing contact re-engages the weld without a new call.

Key concepts:
- VacuumGripperPlugin: runtime activation of MJCF weld equality constraints,
  with a no-snap rewrite of eq_data on engage (the part is held where it is,
  not snapped into the MJCF-declared pose)
- Per-instance services: one activate/release/weld_state triplet per plugin
  instance, so two parts can be gripped independently
- Contact detection in pre_step from data->contact (physics-thread only)

Resources used:
- demo_resources/vacuum_gripper/vacuum_scene.xml (floor, two free-joint parts, two
  inactive weld equalities; requires <compiler fusestatic="false"/>)
- demo_resources/vacuum_gripper/vacuum_robot.xml (planar two-link arm + pad)
- demo_resources/vacuum_gripper/vacuum_robot.urdf (TF + ros2_control block)
- config/mujoco_ros2_control_plugins_vacuum_gripper.yaml (2 VacuumGripperPlugin
  instances + FreeJointStatePublisher for the parts)
- config/controllers_vacuum_gripper.yaml (joint_state_broadcaster +
  arm_position_controller)

Usage:
    ros2 launch mujoco_ros2_control_demos 06_vacuum_gripper_plugin.launch.py
    ros2 launch mujoco_ros2_control_demos 06_vacuum_gripper_plugin.launch.py headless:=true

Services (the hardware interface runs in the 'mujoco_ros2_control_node' node):
    ros2 service call /vacuum_part1/activate  std_srvs/srv/Trigger
    ros2 service call /vacuum_part2/release   std_srvs/srv/Trigger
    ros2 service call /vacuum_part1/weld_state mujoco_ros2_control_msgs/srv/WeldState

Drive the arm (2 joints: [joint1, joint2]):
    ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.83, -0.55]"

Watch the parts' states (FreeJointStatePublisher, 50 Hz):
    ros2 topic echo /part_state_publisher/free_joint_states

Pose table (verified against the compiled model; part top faces are at z = 0.04):
    neutral  [0.0, 0.0]     pad at (2.0, 1.05) -- hovering, no contact
    press1   [0.83, -0.55]  pad centered on part1 (x = 1.6), bottom inside part height
    press2   [1.26, -1.23]  pad centered on part2 (x = 1.3), nearly vertical (2 deg)
    drop     [-0.35, 0.5]   pad at (1.9, 1.25), clear of both parts
    NOTE: always lift to neutral between picks; the pad must not sweep across a part
    at floor level.

Demo sequence:
    1. Negative test: while at neutral (no contact),
       ros2 service call /vacuum_part1/activate std_srvs/srv/Trigger
       -> success: false, "activate ignored: bodies 'vacuum_pad' and 'part1' are not in contact"
    2. Command press1 [0.83, -0.55] and wait for the pad to settle on part1.
       ros2 service call /vacuum_part1/activate std_srvs/srv/Trigger
       -> success: true, "vacuum latched; weld engages on the next physics step"
       ros2 service call /vacuum_part1/weld_state mujoco_ros2_control_msgs/srv/WeldState
       -> weld_active: true
    3. Lift: command neutral [0.0, 0.0] -- part1 rides up rigidly with the pad and the
       weld stays active while riding (auto-release keys off genuine separation, not the
       contact list).
    4. Release: ros2 service call /vacuum_part1/release std_srvs/srv/Trigger
       -> success: true, "vacuum released" -- the weld deactivates immediately and part1
       falls. A welded part is pinned rigidly to the pad, so setting it down always
       requires an explicit release (or the part separating by more than the 0.05 m
       release margin, e.g. a failed hold).
    5. Park: command drop [-0.35, 0.5] to move the arm away clear of both parts.
    6. Pick part2: command press2 [1.26, -1.23], then
       ros2 service call /vacuum_part2/activate std_srvs/srv/Trigger
       -> success: true (independent per-instance state; part1's instance stays off).
    7. With part2's vacuum latched, call
       ros2 service call /mujoco_ros2_control_node/reset_world mujoco_ros2_control_msgs/srv/ResetWorld
       -> the world returns to its initial state and all plugin state clears
       (vacuum off, welds off).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("mujoco_ros2_control_demos")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "demo_resources", "vacuum_gripper", "vacuum_robot.urdf"]),
            " headless:=",
            LaunchConfiguration("headless"),
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    controllers_file = PathJoinSubstitution([pkg_share, "config", "controllers_vacuum_gripper.yaml"])
    mujoco_plugins_file = PathJoinSubstitution([pkg_share, "config", "mujoco_ros2_control_plugins_vacuum_gripper.yaml"])

    nodes = []

    # Robot state publisher
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # ros2_control node with MuJoCo. The arm is driven through the ros2_control
    # position interface (arm_position_controller); the parts and the vacuum
    # behaviour come from the plugins in mujoco_ros2_control_plugins_vacuum_gripper.yaml.
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers_file),
                ParameterFile(mujoco_plugins_file),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
        )
    )

    # Controller spawners
    controllers_to_spawn = ["joint_state_broadcaster", "arm_position_controller"]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", controllers_file],
                output="both",
            )
        )

    return nodes


def generate_launch_description():
    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run simulation without visualization window",
    )

    return LaunchDescription(
        [
            headless,
            OpaqueFunction(function=launch_setup),
        ]
    )
