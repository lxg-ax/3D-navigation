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
        # map→odom is published dynamically by LIO-SAM mapOptimization, no static publisher here.

        # --- 点云适配 + FAST-LIO2 ---
        TimerAction(period=1.0, actions=[
            Node(package='dddnav_utils', executable='livox_pc2_to_liosam',
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
        TimerAction(period=5.0, actions=[
            Node(package='lio_sam', executable='lio_sam_imuPreintegration', output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'), bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=5.0, actions=[
            Node(package='lio_sam', executable='lio_sam_imageProjection', output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'), bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=5.0, actions=[
            Node(package='lio_sam', executable='lio_sam_featureExtraction', output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'), bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),
        TimerAction(period=5.0, actions=[
            Node(package='lio_sam', executable='lio_sam_mapOptimization', output='screen',
                 parameters=[LaunchConfiguration('lio_sam_config'), bringup_paths.lio_sam_save_pcd_overlay()]),
        ]),

        # --- 位姿图转换，参数读自 keyframes_mid360.yaml ---
        TimerAction(period=8.0, actions=[
            Node(package='dddnav_utils', executable='liosam_to_posegraph.py',
                 name='liosam_to_posegraph', output='screen',
                 parameters=[bringup_paths.keyframes_yaml(),
                             bringup_paths.keyframes_save_dir_overlay()]),
        ]),

        # SLAM 健康监视器：FAST-LIO / LIO-SAM 任一停发 > 2s 自动 ERROR
        TimerAction(period=10.0, actions=[
            Node(package='dddnav_utils', executable='slam_health_monitor.py',
                 name='slam_health_monitor', output='screen'),
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
