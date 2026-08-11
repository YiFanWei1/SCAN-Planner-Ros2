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
    terrain_share = get_package_share_directory("terrain_path_segmenter")
    planner_config = os.path.join(share, "config", "planner.yaml")
    controller_config = os.path.join(share, "config", "controllers.yaml")
    real_config = os.path.join(share, "config", "real_go2_livox.yaml")
    rviz_config = os.path.join(terrain_share, "rviz", "segments.rviz")

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
            # The adapter only validates, rate-limits and de-duplicates /plan.
            # Terrain segmentation owns the path sent to SCAN.
            ("initial_path", "/scan_planner/global_path_filtered"),
            ("status", "/scan_planner/input_status"),
        ],
    )

    segmenter = Node(
        package="terrain_path_segmenter",
        executable="terrain_path_visualizer",
        name="terrain_path_segmenter",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "max_linear_z_error": LaunchConfiguration("max_linear_z_error"),
            "slope_merge_threshold": LaunchConfiguration("slope_merge_threshold"),
            "minimum_segment_length": LaunchConfiguration("minimum_segment_length"),
            "accept_first_path_only": LaunchConfiguration(
                "accept_first_global_path_only"),
            "reverse_path_direction_cosine": LaunchConfiguration(
                "reverse_path_direction_cosine"),
            "segment_reached_tolerance": LaunchConfiguration(
                "segment_reached_tolerance"),
        }],
        remappings=[
            ("global_path", "/scan_planner/global_path_filtered"),
            ("body_pose", "/scan_planner/body_pose"),
            ("segments", "/terrain_path/segments"),
            ("processed_path", "/terrain_path/processed"),
            ("current_path", "/scan_planner/initial_path"),
            ("current_goal", "/terrain_path/current_goal"),
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
        parameters=[real_config, {
            "enable_motion_on_start": LaunchConfiguration("enable_motion"),
        }],
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

    analysis = Node(
        package="scan_planner_analysis",
        executable="telemetry_recorder",
        name="scan_planner_telemetry_recorder",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "output_file": LaunchConfiguration("analysis_output"),
            "output_directory": LaunchConfiguration("analysis_output_directory"),
        }],
        condition=IfCondition(LaunchConfiguration("analysis")),
    )

    map_to_camera_init_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_camera_init_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "camera_init",
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
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("max_linear_z_error", default_value="0.04"),
        DeclareLaunchArgument("slope_merge_threshold", default_value="0.04"),
        DeclareLaunchArgument("minimum_segment_length", default_value="0.5"),
        DeclareLaunchArgument("accept_first_global_path_only", default_value="false"),
        # In first-path-only mode, accept a clearly reversed route as a new
        # navigation task while continuing to ignore same-direction rolling
        # updates from the global planner.
        DeclareLaunchArgument("reverse_path_direction_cosine", default_value="-0.25"),
        # Switch slightly earlier than before while staying below the 0.50 m
        # minimum segment length, so a newly activated short segment is not
        # skipped immediately.
        DeclareLaunchArgument("segment_reached_tolerance", default_value="0.40"),
        DeclareLaunchArgument("analysis", default_value="true"),
        DeclareLaunchArgument("analysis_output", default_value=""),
        DeclareLaunchArgument(
            "analysis_output_directory",
            default_value=(
                "/home/wei/github_code/SCAN-Planner-Ros2/"
                "src/scan_planner_analysis/output")),
        adapter,
        segmenter,
        planner,
        controller,
        gate,
        analysis,
        map_to_camera_init_tf,
        rviz,
    ])
