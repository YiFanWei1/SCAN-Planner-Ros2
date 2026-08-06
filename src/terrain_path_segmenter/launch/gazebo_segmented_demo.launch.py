"""Run the Gazebo demo with SCAN consuming one terrain-linear segment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    terrain_share = get_package_share_directory('terrain_path_segmenter')
    scan_share = get_package_share_directory('scan_planner')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, 'launch', 'gazebo_lidar_demo.launch.py')),
        launch_arguments={
            'rviz': 'false',
            'initial_path_topic': '/terrain_path/current',
        }.items(),
    )
    segmenter = Node(
        package='terrain_path_segmenter',
        executable='terrain_path_visualizer',
        name='terrain_path_segmenter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_linear_z_error': LaunchConfiguration('max_linear_z_error'),
            'slope_merge_threshold': LaunchConfiguration(
                'slope_merge_threshold'),
            'segment_reached_tolerance': LaunchConfiguration(
                'segment_reached_tolerance'),
        }],
        remappings=[
            ('global_path', '/initial_path'),
            ('body_pose', '/quad_0/body_pose'),
            ('segments', '/terrain_path/segments'),
            ('processed_path', '/terrain_path/processed'),
            ('current_path', '/terrain_path/current'),
            ('current_goal', '/terrain_path/current_goal'),
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(terrain_share, 'rviz', 'segments.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('max_linear_z_error', default_value='0.04'),
        DeclareLaunchArgument('slope_merge_threshold', default_value='0.04'),
        DeclareLaunchArgument('segment_reached_tolerance', default_value='0.25'),
        simulation,
        segmenter,
        rviz,
    ])
