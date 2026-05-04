"""
DDDNAV 定位导航模式: FAST-LIO2 前端 + MCL 3DL 定位 + 3D 导航栈
前置条件: 需要建图模式生成的位姿图
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.join(get_package_share_directory('dddnav_bringup'), 'launch'))
import bringup_paths


def generate_launch_description():

    livox_share = get_package_share_directory('livox_ros_driver2')
    fast_lio_share = get_package_share_directory('fast_lio')
    p2p_share = get_package_share_directory('p2p_move_base')
    bringup_share = get_package_share_directory('dddnav_bringup')
    LIVOX_CONFIG = os.path.join(livox_share, 'config', 'MID360_config.json')

    return LaunchDescription([
        DeclareLaunchArgument('fastlio_config', default_value=os.path.join(fast_lio_share, 'config', 'mid360_pc2.yaml')),
        DeclareLaunchArgument('nav_config', default_value=os.path.join(p2p_share, 'config', 'mid360_localization.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'localization.rviz')),

        # Livox Mid360 驱动
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             parameters=[{'xfer_format': 0, 'multi_topic': 0, 'data_src': 0,
                          'publish_freq': 10.0, 'output_data_type': 0,
                          'frame_id': 'livox_frame', 'user_config_path': LIVOX_CONFIG}]),

        # Static TF
        Node(package='tf2_ros', executable='static_transform_publisher', name='sensor2baselink',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'livox_frame']),

        # 点云适配: /livox/lidar → liosam/xyzi 格式
        TimerAction(period=1.0, actions=[
            Node(package='dddnav_utils', executable='livox_pc2_to_liosam.py',
                 name='livox_pc2_to_liosam', output='screen',
                 parameters=[{'input_topic': '/livox/lidar',
                              'liosam_output_topic': '/livox/lidar_liosam',
                              'xyzi_output_topic': '/livox/lidar_liosam_xyzi'}]),
        ]),

        # FAST-LIO2 前端: /Odometry, TF odom→base_link
        TimerAction(period=1.0, actions=[
            Node(package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='screen',
                 parameters=[LaunchConfiguration('fastlio_config')]),
        ]),

        # RViz
        TimerAction(period=2.0, actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', LaunchConfiguration('rviz_config')]),
        ]),

        # MCL 3DL 粒子滤波定位 (delay 3s, 加载地图), TF map→odom
        TimerAction(period=3.0, actions=[
            Node(package='mcl_3dl', executable='mcl_3dl', output='screen',
                 parameters=[LaunchConfiguration('nav_config'), bringup_paths.pose_graph_overlay()],
                 remappings=[('odom', '/Odometry'),
                             ('laser_cloud_sharp', '/laser_cloud_sharp'),
                             ('laser_cloud_less_sharp', '/laser_cloud_less_sharp'),
                             ('laser_cloud_flat', '/laser_cloud_flat'),
                             ('laser_cloud_less_flat', '/laser_cloud_less_flat')]),
        ]),

        # MCL 特征提取 (delay 5s, 等 FAST-LIO TF 稳定)
        TimerAction(period=5.0, actions=[
            Node(package='dddnav_mcl_feature', executable='mcl_feature', output='screen',
                 parameters=[LaunchConfiguration('nav_config')],
                 remappings=[('/lslidar_point_cloud', '/livox/lidar_liosam_xyzi'),
                             ('/odom', '/Odometry')]),
        ]),

        # 自动发布初始位姿 (delay 8s, 等 mcl_3dl 加载完地图)
        TimerAction(period=8.0, actions=[
            ExecuteProcess(cmd=[
                'ros2', 'topic', 'pub', '--once', '/initial_3d_pose',
                'geometry_msgs/msg/PoseWithCovarianceStamped',
                "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}",
            ], output='screen'),
        ]),

        # 全局规划 (delay 10s)
        TimerAction(period=10.0, actions=[
            Node(package='global_planner', executable='global_planner_node', output='screen',
                 parameters=[LaunchConfiguration('nav_config')]),
        ]),

        # Move Base + 局部规划 (delay 12s)
        TimerAction(period=12.0, actions=[
            Node(package='p2p_move_base', executable='p2p_move_base_node', output='screen',
                 parameters=[LaunchConfiguration('nav_config')]),
        ]),
        TimerAction(period=14.0, actions=[
            Node(package='p2p_move_base', executable='clicked2goal.py', name='clicked2goal', output='screen'),
        ]),
    ])
