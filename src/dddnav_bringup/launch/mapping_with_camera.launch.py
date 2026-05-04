"""
建图 + 视觉: FAST-LIO2 + LIO-SAM 回环 + RealSense + DDRNet 语义分割
保存地图: ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
保存位姿图: ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
"""

import os, sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

# 让同目录下的 common_camera_nodes 可被 import
sys.path.insert(0, os.path.join(get_package_share_directory('dddnav_bringup'), 'launch'))
import bringup_paths
from common_camera_nodes import vision_nodes


def generate_launch_description():

    livox_share = get_package_share_directory('livox_ros_driver2')
    fast_lio_share = get_package_share_directory('fast_lio')
    lio_sam_share = get_package_share_directory('lio_sam')
    bringup_share = get_package_share_directory('dddnav_bringup')
    map_dir = os.path.join(bringup_share, 'map')
    LIVOX_CONFIG = os.path.join(livox_share, 'config', 'MID360_config.json')

    ld = LaunchDescription([
        DeclareLaunchArgument('fastlio_config', default_value=os.path.join(fast_lio_share, 'config', 'mid360_pc2.yaml')),
        DeclareLaunchArgument('lio_sam_config', default_value=os.path.join(lio_sam_share, 'config', 'params_mid360.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'mapping.rviz')),

        # --- LiDAR ---
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             parameters=[{'xfer_format': 0, 'multi_topic': 0, 'data_src': 0,
                          'publish_freq': 10.0, 'output_data_type': 0,
                          'frame_id': 'livox_frame', 'user_config_path': LIVOX_CONFIG}]),

        # --- TF ---
        Node(package='tf2_ros', executable='static_transform_publisher', name='sensor2baselink',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'livox_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map2odom',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']),

        # --- 点云适配 + FAST-LIO2 ---
        TimerAction(period=1.0, actions=[
            Node(package='dddnav_utils', executable='livox_pc2_to_liosam.py',
                 name='livox_pc2_to_liosam', output='screen',
                 parameters=[{'input_topic': '/livox/lidar',
                              'liosam_output_topic': '/livox/lidar_liosam',
                              'xyzi_output_topic': '/livox/lidar_liosam_xyzi'}]),
        ]),
        TimerAction(period=1.0, actions=[
            Node(package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='screen',
                 parameters=[LaunchConfiguration('fastlio_config')]),
        ]),

        # --- LIO-SAM 后端 ---
        TimerAction(period=3.0, actions=[
            Node(package='lio_sam', executable='lio_sam_imuPreintegration', output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'), bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=3.0, actions=[
            Node(package='lio_sam', executable='lio_sam_imageProjection', output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'), bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=3.0, actions=[
            Node(package='lio_sam', executable='lio_sam_featureExtraction', output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'), bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=3.0, actions=[
            Node(package='lio_sam', executable='lio_sam_mapOptimization', output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'), bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),

        # --- 位姿图转换 ---
        TimerAction(period=5.0, actions=[
            Node(package='dddnav_utils', executable='liosam_to_posegraph.py',
                 name='liosam_to_posegraph', output='screen',
                 parameters=[{'save_dir': map_dir, 'keyframe_dist': 0.5,
                              'keyframe_angle': 0.15, 'ground_angle_thresh': 15.0}]),
        ]),

        # --- RViz ---
        TimerAction(period=2.0, actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', LaunchConfiguration('rviz_config')]),
        ]),
    ])

    # --- 视觉感知: 建图模式关闭彩色 mask, 省 ~20% CPU ---
    for node in vision_nodes(publish_colored_mask=False):
        ld.add_action(node)

    return ld
