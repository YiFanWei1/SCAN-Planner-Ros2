"""Run SCAN-Planner against the real Go2 Livox/LIO interfaces."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("scan_planner")
    planner_config = os.path.join(share, "config", "planner.yaml")
    controller_config = os.path.join(share, "config", "controllers.yaml")
    real_config = os.path.join(share, "config", "real_go2_livox.yaml")
    rviz_config = os.path.join(share, "rviz", "default.rviz")

    adapter = Node(
        package="scan_planner",
        executable="real_go2_input_adapter",
        name="real_go2_input_adapter",
        output="screen",
        parameters=[real_config],
        remappings=[
            ("lidar_odom", "/lio_odom_hf"),
            ("cloud", "/livox/lidar"),
            ("global_path", "/plan"),
            ("body_pose", "/scan_planner/body_pose"),
            ("sensor_pose", "/scan_planner/sensor_pose"),
            ("cloud_out", "/scan_planner/cloud"),
            ("initial_path", "/scan_planner/initial_path"),
            ("status", "/scan_planner/input_status"),
        ],
    )

    planner = Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        output="screen",
        parameters=[planner_config, real_config],
        remappings=[
            ("body_pose", "/scan_planner/body_pose"),
            ("sensor_pose", "/scan_planner/sensor_pose"),
            ("cloud", "/scan_planner/cloud"),
            ("initial_path", "/scan_planner/initial_path"),
        ],
    )

    controller = Node(
        package="scan_planner",
        executable="closed_loop_controller",
        name="closed_loop_controller",
        output="screen",
        parameters=[controller_config, real_config],
        remappings=[
            ("body_pose", "/scan_planner/body_pose"),
            ("planning/bspline", "/planning/bspline"),
            ("cmd_vel", "/scan_planner/cmd_vel_raw"),
        ],
    )

    gate = Node(
        package="scan_planner",
        executable="cmd_vel_safety_gate",
        name="cmd_vel_safety_gate",
        output="screen",
        parameters=[real_config],
        remappings=[
            ("cmd_vel_raw", "/scan_planner/cmd_vel_raw"),
            ("body_pose", "/scan_planner/body_pose"),
            ("planning/bspline", "/planning/bspline"),
            ("planning/data_display", "/planning/data_display"),
            ("emergency_stop", "/scan_planner/emergency_stop"),
            ("enable_motion", "/scan_planner/enable_motion"),
            ("cmd_vel_safe", "/cmd_vel_smoothed"),
            ("status", "/scan_planner/control_status"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="scan_planner_real_rviz",
        output="screen",
        arguments=["-d", rviz_config, "-f", "camera_init"],
        parameters=[{"use_sim_time": False}],
        remappings=[
            ("/quad_0/cloud", "/scan_planner/cloud"),
            ("/quad_0/path", "/scan_planner/initial_path"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        adapter,
        planner,
        controller,
        gate,
        rviz,
    ])
