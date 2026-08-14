"""Run SCAN-Planner in simulation with the standalone MPPI controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    mppi_share = get_package_share_directory("scan_mppi_controller")
    go2_share = get_package_share_directory("go2_description")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    mppi_yaml = os.path.join(mppi_share, "config", "mppi.yaml")
    mppi_sim_yaml = os.path.join(mppi_share, "config", "mppi_sim.yaml")
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be lidar or depth")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")

    common = {"use_sim_time": use_sim_time}
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml, {
                **common,
                "fsm.navi_mode": navi_mode,
                "grid_map.sensor_type": sensor_type,
                "grid_map.cloud_is_world": True,
                "grid_map.need_extrinsic": False,
            }],
            remappings=[
                ("body_pose", "/quad_0/body_pose"),
                ("sensor_pose", "/quad_0/camera_pose" if sensor_type == "depth"
                 else "/quad_0/lidar_pose"),
                ("cloud", "/quad_0/cloud"),
                ("depth", "/quad_0/depth"),
                ("initial_path", "/initial_path"),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2_robot_state_publisher",
            output="screen",
            parameters=[common, {
                "robot_description": Command([
                    "xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                    " use_gazebo:=false",
                ])
            }],
        ),
        Node(
            package="scan_mppi_controller",
            executable="scan_mppi_controller_node",
            name="scan_mppi_controller",
            output="screen",
            parameters=[mppi_yaml, mppi_sim_yaml, common],
            remappings=[
                ("body_pose", "/quad_0/body_pose"),
                ("planning/bspline", "/planning/bspline"),
                ("grid_map/occupancy_inflate", "/grid_map/occupancy_inflate"),
                ("cmd_vel_raw", "/scan_planner/cmd_vel_raw"),
            ],
        ),
        Node(
            package="scan_planner",
            executable="cmd_vel_safety_gate",
            name="cmd_vel_safety_gate",
            output="screen",
            parameters=[mppi_yaml, common, {
                "enable_motion_on_start": ParameterValue(
                    LaunchConfiguration("enable_motion"), value_type=bool),
                "require_planner_heartbeat": False,
            }],
            remappings=[
                ("cmd_vel_raw", "/scan_planner/cmd_vel_raw"),
                ("body_pose", "/quad_0/body_pose"),
                ("planning/bspline", "/planning/bspline"),
                ("planning/data_display", "/planning/data_display"),
                ("emergency_stop", "/scan_planner/emergency_stop"),
                ("enable_motion", "/scan_planner/enable_motion"),
                ("cmd_vel_safe", "/quad_0/cmd_vel"),
                ("status", "/scan_planner/control_status"),
            ],
        ),
        Node(
            package="scan_planner",
            executable="go2_kinematic_sim",
            name="go2_kinematic_sim",
            output="screen",
            parameters=[controllers_yaml, common, {
                "init_x": ParameterValue(LaunchConfiguration("init_x"), value_type=float),
                "init_y": ParameterValue(LaunchConfiguration("init_y"), value_type=float),
                "init_z": ParameterValue(LaunchConfiguration("init_z"), value_type=float),
                "publish_tf": False,
            }],
            remappings=[
                ("body_pose", "/quad_0/body_pose"),
                ("cmd_vel", "/quad_0/cmd_vel"),
            ],
        ),
        Node(
            package="scan_planner",
            executable="go2_gait_publisher",
            name="go2_gait_publisher",
            output="screen",
            parameters=[controllers_yaml, common],
            remappings=[("body_pose", "/quad_0/body_pose")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scan_share, "launch", "simulator.launch.py")),
            launch_arguments={
                "is_real_world": "false",
                "sensor_type": sensor_type,
                "use_gpu": LaunchConfiguration("use_gpu"),
                "use_pcd_map": LaunchConfiguration("use_pcd_map"),
                "pcd_map_file": LaunchConfiguration("pcd_map_file"),
                "map_size_x": LaunchConfiguration("map_size_x"),
                "map_size_y": LaunchConfiguration("map_size_y"),
                "map_size_z": LaunchConfiguration("map_size_z"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="scan_mppi_sim_rviz",
            output="screen",
            arguments=["-d", os.path.join(mppi_share, "rviz", "mppi_sim.rviz")],
            parameters=[common],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("navi_mode", default_value="1"),
        DeclareLaunchArgument("sensor_type", default_value="lidar"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_gpu", default_value="false"),
        DeclareLaunchArgument("use_pcd_map", default_value="false"),
        DeclareLaunchArgument("pcd_map_file", default_value=""),
        DeclareLaunchArgument("map_size_x", default_value="40.0"),
        DeclareLaunchArgument("map_size_y", default_value="40.0"),
        DeclareLaunchArgument("map_size_z", default_value="5.0"),
        DeclareLaunchArgument("init_x", default_value="-19.0"),
        DeclareLaunchArgument("init_y", default_value="1.0"),
        DeclareLaunchArgument("init_z", default_value="0.25"),
        OpaqueFunction(function=_setup),
    ])
