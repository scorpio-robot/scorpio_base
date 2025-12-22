#!/usr/bin/env python3
# Copyright 2025 Lihan Chen
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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory("scorpio_base")

    # Create the launch configuration variables
    params_file = LaunchConfiguration("params_file")

    # Set environment variables for better logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Declare launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "base_param.yaml"),
        description="Path to parameter file",
    )

    # Main Scorpio base driver node
    start_scorpio_base_node = Node(
        package="scorpio_base",
        executable="scorpio_base_node",
        name="scorpio_base",
        output="screen",
        parameters=[params_file],
    )

    start_joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
        parameters=[params_file],
    )

    # Teleop twist joy node
    start_teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[params_file],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)

    # Add the nodes
    ld.add_action(start_scorpio_base_node)
    ld.add_action(start_joy_node)
    ld.add_action(start_teleop_twist_joy_node)

    return ld
