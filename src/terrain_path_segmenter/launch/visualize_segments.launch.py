"""Launch terrain path segmentation visualization without controlling SCAN."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("terrain_path_segmenter")
    return LaunchDescription([
        DeclareLaunchArgument("input_path", default_value="/initial_path"),
        DeclareLaunchArgument("body_pose", default_value="/quad_0/body_pose"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("max_linear_z_error", default_value="0.04"),
        DeclareLaunchArgument("slope_merge_threshold", default_value="0.04"),
        Node(
            package="terrain_path_segmenter",
            executable="terrain_path_visualizer",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "max_linear_z_error": LaunchConfiguration("max_linear_z_error"),
                "slope_merge_threshold": LaunchConfiguration(
                    "slope_merge_threshold"),
            }],
            remappings=[
                ("global_path", LaunchConfiguration("input_path")),
                ("body_pose", LaunchConfiguration("body_pose")),
                ("segments", "/terrain_path/segments"),
                ("processed_path", "/terrain_path/processed"),
                ("current_path", "/terrain_path/current"),
                ("current_goal", "/terrain_path/current_goal"),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "segments.rviz")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
