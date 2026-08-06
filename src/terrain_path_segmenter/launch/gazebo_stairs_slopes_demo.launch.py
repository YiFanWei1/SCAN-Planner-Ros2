"""Original scan_demo extended with two staircases and the existing ramps."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    terrain_share = get_package_share_directory('terrain_path_segmenter')
    scan_share = get_package_share_directory('scan_planner')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(scan_share, 'launch', 'gazebo_lidar_demo.launch.py')),
        launch_arguments={
            'rviz': 'false',
            'loop_enabled': 'true',
            'initial_path_topic': '/terrain_path/current',
            'clear_map_on_new_path': 'true',
            'fresh_observations_before_planning': '2',
            'path_config_file': os.path.join(scan_share, 'config', 'gazebo_stairs_slopes_path.yaml'),
            'terrain_config_file': os.path.join(scan_share, 'config', 'stairs_slopes_sim.yaml'),
            'init_x': '-8.5', 'init_y': '-5.5', 'init_z': '0.4',
        }.items())
    segmenter = Node(
        package='terrain_path_segmenter', executable='terrain_path_visualizer',
        name='terrain_path_segmenter', output='screen',
        parameters=[{'use_sim_time': True, 'max_linear_z_error': 0.025,
                     'slope_merge_threshold': 0.025, 'segment_reached_tolerance': 0.25}],
        remappings=[('global_path','/initial_path'),('body_pose','/quad_0/body_pose'),
                    ('segments','/terrain_path/segments'),('processed_path','/terrain_path/processed'),
                    ('current_path','/terrain_path/current'),('current_goal','/terrain_path/current_goal')])
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', os.path.join(terrain_share,'rviz','segments.rviz')],
                parameters=[{'use_sim_time': True}])
    return LaunchDescription([simulation, segmenter, rviz])
