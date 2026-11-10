# Copyright (c) 2024 FZI Forschungszentrum Informatik
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
#    * Neither the name of the {copyright_holder} nor the names of its
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

#
# Author: Felix Exner
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ros2_control_hardware_type = LaunchConfiguration(
        "ros2_control_hardware_type")

    moveit_config = (
        MoveItConfigsBuilder("proxima", package_name="proxima_moveit_config")
        .robot_description(
            file_path="config/proxima.urdf.xacro",
            mappings={"ros2_control_hardware_type": ros2_control_hardware_type},
        )
        .robot_description_semantic(file_path="config/proxima.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("proxima_telop", "config/proxima_config.yaml")
    servo_yaml["is_primary_planning_scene_monitor"] = False
    sim_time = {"use_sim_time": use_sim_time}
    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "arm"}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        condition=IfCondition(launch_servo),
        parameters=[
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"moveit_servo": servo_yaml},
            sim_time,
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "device_id": 0,
                "deadzone": 0.3,
                "autorepeat_rate": 20.0,
            },
            sim_time,
        ],
    )

    joystick_control = Node(
        package="proxima_telop",
        executable="joystick_control",
        name="joystick_control",
        output="screen",
        parameters=[servo_yaml, sim_time],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_servo",
                default_value="true",
                description="Start moveit_servo next to the joystick.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock from Isaac Sim.",
            ),
            DeclareLaunchArgument(
                "ros2_control_hardware_type",
                default_value="isaac",
                description="Must match proxima_demo.launch.py.",
            ),
            joy_node,
            joystick_control,
            servo_node,
        ]
    )
