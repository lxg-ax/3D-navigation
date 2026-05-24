"""Mapping + navigation + vision (RealSense + DDRNet).

Camera path is unverified on real hardware. Tunables in
src/dddnav_bringup/config/runtime.yaml.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(
    get_package_share_directory('dddnav_bringup'), 'launch'))
import bringup_paths
from common_camera_nodes import vision_nodes


def _resolve_nav_config(context, *args, **kwargs):
    profile = LaunchConfiguration('nav_profile').perform(context)
    return [DeclareLaunchArgument(
        'nav_config',
        default_value=bringup_paths.nav_config_path(profile),
        description='Resolved absolute path to nav yaml (auto from nav_profile).',
    )]


def generate_launch_description():
    # Get the launch directories
    bringup_dir  = get_package_share_directory('dddnav_bringup')
    livox_dir    = get_package_share_directory('livox_ros_driver2')
    fast_lio_dir = get_package_share_directory('fast_lio')
    lio_sam_dir  = get_package_share_directory('lio_sam')

    # Load runtime knobs
    rt = bringup_paths.load_runtime()
    d  = rt['delays']
    m  = rt['lidar_mount']
    livox_user = os.path.join(livox_dir, 'config', 'MID360_config.json')

    # Create the launch configuration variables
    fastlio_config = LaunchConfiguration('fastlio_config')
    lio_sam_config = LaunchConfiguration('lio_sam_config')
    nav_config     = LaunchConfiguration('nav_config')
    rviz_config    = LaunchConfiguration('rviz_config')

    # Declare the launch arguments
    declare_fastlio_config_cmd = DeclareLaunchArgument(
        'fastlio_config',
        default_value=os.path.join(fast_lio_dir, 'config', 'mid360_pc2.yaml'),
        description='Full path to the FAST-LIO yaml',
    )
    declare_lio_sam_config_cmd = DeclareLaunchArgument(
        'lio_sam_config',
        default_value=os.path.join(lio_sam_dir, 'config', 'params_mid360.yaml'),
        description='Full path to the LIO-SAM yaml',
    )
    declare_nav_profile_cmd = DeclareLaunchArgument(
        'nav_profile',
        default_value='mid360_mapping_with_camera',
        description='Nav tuning profile under dddnav_bringup/config/nav/',
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'mapping_nav.rviz'),
        description='Full path to the RVIZ config file',
    )

    # Sensor + TF
    start_livox_driver_cmd = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen',
        parameters=[{
            'xfer_format': 0, 'multi_topic': 0, 'data_src': 0,
            'publish_freq': float(rt['livox_publish_freq']),
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': livox_user,
        }],
    )
    start_sensor_tf_cmd = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='sensor2baselink',
        arguments=[str(m['x']), str(m['y']), str(m['z']),
                   str(m['yaw']), str(m['pitch']), str(m['roll']),
                   'base_link', 'livox_frame'],
    )

    # Bridges
    start_livox_bridge_cmd = TimerAction(period=d['bridges'], actions=[
        Node(package='dddnav_utils', executable='livox_pc2_to_liosam',
             name='livox_pc2_to_liosam', output='screen',
             parameters=[{'input_topic': '/livox/lidar',
                          'liosam_output_topic': '/livox/lidar_liosam',
                          'xyzi_output_topic': '/livox/lidar_liosam_xyzi'}]),
    ])
    start_fast_lio_cmd = TimerAction(period=d['bridges'], actions=[
        Node(package='fast_lio', executable='fastlio_mapping',
             name='fast_lio', output='screen',
             parameters=[fastlio_config]),
    ])

    # LIO-SAM back-end
    lio_sam_params = [lio_sam_config, bringup_paths.lio_sam_save_pcd_overlay()]
    start_lio_sam_imu_cmd  = TimerAction(period=d['liosam_back'], actions=[
        Node(package='lio_sam', executable='lio_sam_imuPreintegration',
             output='screen', parameters=lio_sam_params)])
    start_lio_sam_proj_cmd = TimerAction(period=d['liosam_back'], actions=[
        Node(package='lio_sam', executable='lio_sam_imageProjection',
             output='screen', parameters=lio_sam_params)])
    start_lio_sam_feat_cmd = TimerAction(period=d['liosam_back'], actions=[
        Node(package='lio_sam', executable='lio_sam_featureExtraction',
             output='screen', parameters=lio_sam_params)])
    start_lio_sam_opt_cmd  = TimerAction(period=d['liosam_back'], actions=[
        Node(package='lio_sam', executable='lio_sam_mapOptimization',
             output='screen', parameters=lio_sam_params)])

    # Pose graph + health
    start_posegraph_cmd = TimerAction(period=d['liosam_to_pg'], actions=[
        Node(package='dddnav_utils', executable='liosam_to_posegraph.py',
             name='liosam_to_posegraph', output='screen',
             parameters=[bringup_paths.keyframes_yaml(),
                         bringup_paths.keyframes_save_dir_overlay()]),
    ])
    start_health_cmd = TimerAction(period=d['health'], actions=[
        Node(package='dddnav_utils', executable='slam_health_monitor.py',
             name='slam_health_monitor', output='screen'),
    ])

    # Navigation
    start_global_planner_cmd = TimerAction(period=d['global_planner'], actions=[
        Node(package='global_planner', executable='global_planner_node',
             output='screen', parameters=[nav_config])])
    start_move_base_cmd = TimerAction(period=d['move_base'], actions=[
        Node(package='p2p_move_base', executable='p2p_move_base_node',
             output='screen', parameters=[nav_config])])
    start_clicked_goal_cmd = TimerAction(period=d['clicked_goal'], actions=[
        Node(package='p2p_move_base', executable='clicked2goal.py',
             name='clicked2goal', output='screen')])

    start_rviz_cmd = TimerAction(period=d['rviz'], actions=[
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', rviz_config]),
    ])

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_fastlio_config_cmd)
    ld.add_action(declare_lio_sam_config_cmd)
    ld.add_action(declare_nav_profile_cmd)
    ld.add_action(OpaqueFunction(function=_resolve_nav_config))
    ld.add_action(declare_rviz_config_cmd)

    # Sensor + TF
    ld.add_action(start_livox_driver_cmd)
    ld.add_action(start_sensor_tf_cmd)

    # SLAM stack
    ld.add_action(start_livox_bridge_cmd)
    ld.add_action(start_fast_lio_cmd)
    ld.add_action(start_lio_sam_imu_cmd)
    ld.add_action(start_lio_sam_proj_cmd)
    ld.add_action(start_lio_sam_feat_cmd)
    ld.add_action(start_lio_sam_opt_cmd)
    ld.add_action(start_posegraph_cmd)
    ld.add_action(start_health_cmd)

    # Navigation
    ld.add_action(start_global_planner_cmd)
    ld.add_action(start_move_base_cmd)
    ld.add_action(start_clicked_goal_cmd)

    # Visualization
    ld.add_action(start_rviz_cmd)

    # Vision: colored mask on for nav debugging.
    for node in vision_nodes(publish_colored_mask=True):
        ld.add_action(node)
    return ld
