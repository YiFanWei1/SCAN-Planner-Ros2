"""Gazebo 3D lidar closed-path demo for SCAN-Planner Mode 3."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    resource_root = os.path.dirname(go2_share)
    world = os.path.join(go2_share, "worlds", "scan_demo.sdf")
    # This demo owns world->base TF.  Do not bridge Gazebo model poses onto /tf,
    # otherwise two publishers describe the same moving robot.
    bridge_config = os.path.join(go2_share, "config", "bridge_kinematic.yaml")
    default_path_config = os.path.join(scan_share, "config", "gazebo_loop_path.yaml")
    path_config = LaunchConfiguration("path_config_file")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -v 3 ", world], "on_exit_shutdown": "true"}.items(),
    )
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(scan_share, "launch", "run.launch.py")),
        launch_arguments={
            "is_real_world": "false",
            "navi_mode": "3",
            "sensor_type": "lidar",
            "controller_mode": "closed_loop",
            "simulator_backend": LaunchConfiguration("simulator_backend"),
            "use_gpu": "false",
            "use_pcd_map": "false",
            "init_x": LaunchConfiguration("init_x"),
            "init_y": LaunchConfiguration("init_y"),
            "init_z": LaunchConfiguration("init_z"),
            "use_sim_time": "true",
            "initial_path_topic": LaunchConfiguration("initial_path_topic"),
            "clear_map_on_new_path": LaunchConfiguration("clear_map_on_new_path"),
            "fresh_observations_before_planning": LaunchConfiguration(
                "fresh_observations_before_planning"),
            "terrain_config_file": LaunchConfiguration("terrain_config_file"),
        }.items(),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description", "-name", "go2",
            "-allow_renaming", "false", "-x", LaunchConfiguration("init_x"),
            "-y", LaunchConfiguration("init_y"), "-z", LaunchConfiguration("init_z"),
        ],
    )
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="go2_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )
    pose_service_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_pose_service_bridge",
        output="screen",
        arguments=["/world/scan_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
    )
    loop_path = Node(
        package="scan_planner",
        executable="loop_path_publisher.py",
        name="loop_path_publisher",
        output="screen",
        parameters=[
            path_config,
            {
                "loop_enabled": LaunchConfiguration("loop_enabled"),
                "loop_position_tolerance": LaunchConfiguration("loop_position_tolerance"),
                "loop_cooldown": LaunchConfiguration("loop_cooldown"),
            },
        ],
        remappings=[("body_pose", "/quad_0/body_pose"), ("initial_path", "/initial_path")],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(scan_share, "launch", "default.rviz")],
        remappings=[("/quad_0/cloud", "/go2/lidar/points")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [resource_root, os.pathsep,
             EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
        ),
        DeclareLaunchArgument("simulator_backend", default_value="gazebo_lidar"),
        DeclareLaunchArgument("path_config_file", default_value=default_path_config),
        DeclareLaunchArgument("terrain_config_file", default_value=""),
        DeclareLaunchArgument("init_x", default_value="-6.0"),
        DeclareLaunchArgument("init_y", default_value="1.0"),
        DeclareLaunchArgument("init_z", default_value="0.4"),
        DeclareLaunchArgument("loop_enabled", default_value="false"),
        DeclareLaunchArgument("loop_position_tolerance", default_value="0.5"),
        DeclareLaunchArgument("loop_cooldown", default_value="3.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("initial_path_topic", default_value="/initial_path"),
        DeclareLaunchArgument("clear_map_on_new_path", default_value="false"),
        DeclareLaunchArgument("fresh_observations_before_planning", default_value="2"),
        gazebo,
        topic_bridge,
        pose_service_bridge,
        planner,
        spawn,
        loop_path,
        rviz,
    ])
