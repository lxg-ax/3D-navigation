"""Mapping + navigation + vision (RealSense + DDRNet).

Camera path is unverified on real hardware. Tunables in
src/dddnav_bringup/config/runtime.yaml.
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.join(get_package_share_directory('dddnav_bringup'), 'launch'))
import bringup_paths
from common_camera_nodes import vision_nodes


def generate_launch_description():
    rt = bringup_paths.load_runtime()
    d = rt['delays']
    m = rt['lidar_mount']

    livox_share    = get_package_share_directory('livox_ros_driver2')
    fast_lio_share = get_package_share_directory('fast_lio')
    lio_sam_share  = get_package_share_directory('lio_sam')
    p2p_share      = get_package_share_directory('p2p_move_base')
    bringup_share  = get_package_share_directory('dddnav_bringup')
    LIVOX_CONFIG   = os.path.join(livox_share, 'config', 'MID360_config.json')

    ld = LaunchDescription([
        DeclareLaunchArgument('fastlio_config',
            default_value=os.path.join(fast_lio_share, 'config', 'mid360_pc2.yaml')),
        DeclareLaunchArgument('lio_sam_config',
            default_value=os.path.join(lio_sam_share, 'config', 'params_mid360.yaml')),
        DeclareLaunchArgument('nav_config',
            default_value=os.path.join(p2p_share, 'config', 'mid360_mapping_with_camera.yaml')),
        DeclareLaunchArgument('rviz_config',
            default_value=os.path.join(bringup_share, 'rviz', 'mapping_nav.rviz')),

        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             parameters=[{
                 'xfer_format': 0, 'multi_topic': 0, 'data_src': 0,
                 'publish_freq': float(rt['livox_publish_freq']),
                 'output_data_type': 0,
                 'frame_id': 'livox_frame',
                 'user_config_path': LIVOX_CONFIG,
             }]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='sensor2baselink',
             arguments=[str(m['x']), str(m['y']), str(m['z']),
                        str(m['yaw']), str(m['pitch']), str(m['roll']),
                        'base_link', 'livox_frame']),

        TimerAction(period=d['bridges'], actions=[
            Node(package='dddnav_utils', executable='livox_pc2_to_liosam',
                 name='livox_pc2_to_liosam', output='screen',
                 parameters=[{'input_topic': '/livox/lidar',
                              'liosam_output_topic': '/livox/lidar_liosam',
                              'xyzi_output_topic': '/livox/lidar_liosam_xyzi'}]),
        ]),
        TimerAction(period=d['bridges'], actions=[
            Node(package='fast_lio', executable='fastlio_mapping',
                 name='fast_lio', output='screen',
                 parameters=[LaunchConfiguration('fastlio_config')]),
        ]),

        TimerAction(period=d['liosam_back'], actions=[
            Node(package='lio_sam', executable='lio_sam_imuPreintegration',
                 output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'),
                             bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=d['liosam_back'], actions=[
            Node(package='lio_sam', executable='lio_sam_imageProjection',
                 output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'),
                             bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=d['liosam_back'], actions=[
            Node(package='lio_sam', executable='lio_sam_featureExtraction',
                 output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'),
                             bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=d['liosam_back'], actions=[
            Node(package='lio_sam', executable='lio_sam_mapOptimization',
                 output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'),
                             bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),

        TimerAction(period=d['liosam_to_pg'], actions=[
            Node(package='dddnav_utils', executable='liosam_to_posegraph.py',
                 name='liosam_to_posegraph', output='screen',
                 parameters=[bringup_paths.keyframes_yaml(),
                             bringup_paths.keyframes_save_dir_overlay()]),
        ]),

        TimerAction(period=d['health'], actions=[
            Node(package='dddnav_utils', executable='slam_health_monitor.py',
                 name='slam_health_monitor', output='screen'),
        ]),

        TimerAction(period=d['global_planner'], actions=[
            Node(package='global_planner', executable='global_planner_node', output='screen',
                 parameters=[LaunchConfiguration('nav_config')]),
        ]),
        TimerAction(period=d['move_base'], actions=[
            Node(package='p2p_move_base', executable='p2p_move_base_node', output='screen',
                 parameters=[LaunchConfiguration('nav_config')]),
        ]),
        TimerAction(period=d['clicked_goal'], actions=[
            Node(package='p2p_move_base', executable='clicked2goal.py',
                 name='clicked2goal', output='screen'),
        ]),

        TimerAction(period=d['rviz'], actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', LaunchConfiguration('rviz_config')]),
        ]),
    ])

    # Vision: colored mask on for nav debugging.
    for node in vision_nodes(publish_colored_mask=True):
        ld.add_action(node)

    return ld
