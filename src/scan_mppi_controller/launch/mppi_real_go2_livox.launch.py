"""Run the real Go2/Livox SCAN pipeline with the standalone MPPI controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    terrain_share = get_package_share_directory("terrain_path_segmenter")
    mppi_share = get_package_share_directory("scan_mppi_controller")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    real_yaml = os.path.join(scan_share, "config", "real_go2_livox.yaml")
    mppi_yaml = os.path.join(mppi_share, "config", "mppi.yaml")
    mppi_real_yaml = os.path.join(mppi_share, "config", "mppi_real.yaml")

    planner_path_topic = IfElseSubstitution(
        condition=LaunchConfiguration("use_path_segmentation"),
        if_value="/scan_planner/initial_path",
        else_value="/scan_planner/global_path_filtered")
    raw_cloud_selected = EqualsSubstitution(
        LaunchConfiguration("point_cloud_type"), "1")
    input_cloud_topic = IfElseSubstitution(
        condition=raw_cloud_selected,
        if_value="/livox/lidar",
        else_value="/cloud_registered_body")
    lidar_to_base_x = IfElseSubstitution(
        condition=raw_cloud_selected, if_value="-0.15", else_value="0.0")
    lidar_to_base_z = IfElseSubstitution(
        condition=raw_cloud_selected, if_value="-0.21", else_value="0.0")

    adapter = Node(
        package="scan_planner",
        executable="real_go2_input_adapter",
        name="real_go2_input_adapter",
        output="screen",
        parameters=[real_yaml, {
            "lidar_to_base_x": ParameterValue(lidar_to_base_x, value_type=float),
            "lidar_to_base_y": 0.0,
            "lidar_to_base_z": ParameterValue(lidar_to_base_z, value_type=float),
        }],
        remappings=[
            ("lidar_odom", "/lio_odom_hf"),
            ("cloud", input_cloud_topic),
            ("global_path", "/plan"),
            ("body_pose", "/scan_planner/body_pose"),
            ("sensor_pose", "/scan_planner/sensor_pose"),
            ("cloud_out", "/scan_planner/cloud"),
            ("initial_path", "/scan_planner/global_path_filtered"),
            ("status", "/scan_planner/input_status"),
        ])

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
        condition=IfCondition(LaunchConfiguration("use_path_segmentation")))

    planner = Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        output="screen",
        parameters=[planner_yaml, real_yaml, {
            "grid_map.body_exclusion_enabled": True,
            "grid_map.body_exclusion_half_length": 0.40,
            "grid_map.body_exclusion_half_width": 0.22,
            "grid_map.body_exclusion_z_down": 0.18,
            "grid_map.body_exclusion_z_up": 0.18,
            "grid_map.occupancy_decay_front_only": ParameterValue(
                LaunchConfiguration("occupancy_decay_front_only"), value_type=bool),
            "fsm.project_reference_start_z": ParameterValue(
                LaunchConfiguration("project_reference_start_z"), value_type=bool),
            "fsm.reference_start_z_max_correction": ParameterValue(
                LaunchConfiguration("reference_start_z_max_correction"), value_type=float),
            "fsm.project_reference_start_velocity": ParameterValue(
                LaunchConfiguration("project_reference_start_velocity"), value_type=bool),
            "fsm.reference_velocity_tangent_half_window": ParameterValue(
                LaunchConfiguration("reference_velocity_tangent_half_window"), value_type=float),
            "fsm.reference_start_velocity_max": ParameterValue(
                LaunchConfiguration("reference_start_velocity_max"), value_type=float),
            "fsm.odom_twist_in_body_frame": ParameterValue(
                LaunchConfiguration("odom_twist_in_body_frame"), value_type=bool),
            "fsm.require_stop_before_emergency_replan": ParameterValue(
                LaunchConfiguration("require_stop_before_emergency_replan"), value_type=bool),
        }],
        remappings=[
            ("body_pose", "/scan_planner/body_pose"),
            ("sensor_pose", "/scan_planner/sensor_pose"),
            ("cloud", "/scan_planner/cloud"),
            ("initial_path", planner_path_topic),
        ])

    mppi = Node(
        package="scan_mppi_controller",
        executable="scan_mppi_controller_node",
        name="scan_mppi_controller",
        output="screen",
        parameters=[mppi_yaml, mppi_real_yaml],
        remappings=[
            ("body_pose", "/scan_planner/body_pose"),
            ("planning/bspline", "/planning/bspline"),
            ("grid_map/occupancy_inflate", "/grid_map/occupancy_inflate"),
            ("cmd_vel_raw", "/scan_planner/cmd_vel_raw"),
        ])

    safety_gate = Node(
        package="scan_planner",
        executable="cmd_vel_safety_gate",
        name="cmd_vel_safety_gate",
        output="screen",
        parameters=[mppi_yaml, real_yaml, {
            "enable_motion_on_start": ParameterValue(
                LaunchConfiguration("enable_motion"), value_type=bool),
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
        ])

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
        condition=IfCondition(LaunchConfiguration("analysis")))

    map_to_camera_init_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_camera_init_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "camera_init",
        ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="scan_mppi_real_rviz",
        output="screen",
        arguments=["-d", os.path.join(mppi_share, "rviz", "mppi_real.rviz"),
                   "-f", "camera_init"],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("rviz")))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("use_path_segmentation", default_value="true"),
        DeclareLaunchArgument("accept_first_global_path_only", default_value="false"),
        DeclareLaunchArgument("require_stop_before_emergency_replan", default_value="false"),
        DeclareLaunchArgument("occupancy_decay_front_only", default_value="true"),
        DeclareLaunchArgument("project_reference_start_z", default_value="true"),
        DeclareLaunchArgument("reference_start_z_max_correction", default_value="0.60"),
        DeclareLaunchArgument("project_reference_start_velocity", default_value="true"),
        DeclareLaunchArgument("reference_velocity_tangent_half_window", default_value="0.40"),
        DeclareLaunchArgument("reference_start_velocity_max", default_value="0.75"),
        DeclareLaunchArgument("odom_twist_in_body_frame", default_value="true"),
        DeclareLaunchArgument("point_cloud_type", default_value="1", choices=["1", "2"]),
        DeclareLaunchArgument("max_linear_z_error", default_value="0.06"),
        DeclareLaunchArgument("slope_merge_threshold", default_value="0.06"),
        DeclareLaunchArgument("minimum_segment_length", default_value="0.50"),
        DeclareLaunchArgument("reverse_path_direction_cosine", default_value="-0.25"),
        DeclareLaunchArgument("segment_reached_tolerance", default_value="0.35"),
        DeclareLaunchArgument("analysis", default_value="true"),
        DeclareLaunchArgument("analysis_output", default_value=""),
        DeclareLaunchArgument(
            "analysis_output_directory",
            default_value=os.path.join(mppi_share, "analysis_output")),
        LogInfo(msg=[
            "SCAN MPPI cloud input type=", LaunchConfiguration("point_cloud_type"),
            " topic=", input_cloud_topic,
            " body exclusion=[0.40, 0.22, -0.18..0.18]",
        ]),
        adapter,
        segmenter,
        planner,
        mppi,
        safety_gate,
        analysis,
        map_to_camera_init_tf,
        rviz,
    ])
