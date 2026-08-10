"""SCAN closed-loop demo on a continuous piecewise-linear rolling terrain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    terrain_share = get_package_share_directory('terrain_path_segmenter')
    scan_share = get_package_share_directory('scan_planner')
    go2_share = get_package_share_directory('go2_description')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    resource_root = os.path.dirname(go2_share)
    world = os.path.join(go2_share, 'worlds', 'rough_terrain_demo.sdf')
    bridge_config = os.path.join(go2_share, 'config', 'bridge_kinematic.yaml')
    path_config = os.path.join(scan_share, 'config', 'rough_terrain_loop_path.yaml')
    terrain_config = os.path.join(scan_share, 'config', 'rough_terrain_sim.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, 'launch', 'run.launch.py')),
        launch_arguments={
            'is_real_world': 'false',
            'navi_mode': '3',
            'sensor_type': 'lidar',
            'controller_mode': 'closed_loop',
            'simulator_backend': 'gazebo_lidar',
            'use_gpu': 'false',
            'use_pcd_map': 'false',
            'init_x': '-1.5',
            'init_y': '-11.0',
            'init_z': '0.4',
            'use_sim_time': 'true',
            'initial_path_topic': '/terrain_path/current',
            'clear_map_on_new_path': 'true',
            'fresh_observations_before_planning': '2',
            'terrain_config_file': terrain_config,
            'gazebo_set_pose_service': '/world/rough_terrain_demo/set_pose',
        }.items(),
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-topic', 'robot_description', '-name', 'go2',
            '-allow_renaming', 'false', '-x', '-1.5', '-y', '-11.0', '-z', '0.4',
        ],
    )
    topic_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='go2_gz_bridge', output='screen',
        parameters=[{'config_file': bridge_config}],
    )
    pose_service_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gazebo_pose_service_bridge', output='screen',
        arguments=[
            '/world/rough_terrain_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
    )
    path_publisher = Node(
        package='scan_planner', executable='loop_path_publisher.py',
        name='loop_path_publisher', output='screen',
        parameters=[path_config],
        remappings=[
            ('body_pose', '/quad_0/body_pose'),
            ('initial_path', '/initial_path'),
        ],
    )
    segmenter = Node(
        package='terrain_path_segmenter', executable='terrain_path_visualizer',
        name='terrain_path_segmenter', output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_linear_z_error': LaunchConfiguration('max_linear_z_error'),
            'slope_merge_threshold': LaunchConfiguration('slope_merge_threshold'),
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
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(terrain_share, 'rviz', 'segments.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [resource_root, os.pathsep,
             EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]),
        DeclareLaunchArgument('max_linear_z_error', default_value='0.025'),
        DeclareLaunchArgument('slope_merge_threshold', default_value='0.025'),
        DeclareLaunchArgument('segment_reached_tolerance', default_value='0.25'),
        gazebo,
        topic_bridge,
        pose_service_bridge,
        planner,
        spawn,
        path_publisher,
        segmenter,
        rviz,
    ])
