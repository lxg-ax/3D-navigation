"""Mapping mode: FAST-LIO front + LIO-SAM back-end with loop closure.

Save map: ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
Save pose graph: ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
Tunables: src/dddnav_bringup/config/runtime.yaml
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(
    get_package_share_directory('dddnav_bringup'), 'launch'))
import bringup_paths


def generate_launch_description():
    # Get the launch directories
    bringup_dir  = get_package_share_directory('dddnav_bringup')
    livox_dir    = get_package_share_directory('livox_ros_driver2')
    fast_lio_dir = get_package_share_directory('fast_lio')
    lio_sam_dir  = get_package_share_directory('lio_sam')

    # Load runtime knobs (timing, sensor mount, ...)
    rt = bringup_paths.load_runtime()
    d  = rt['delays']
    m  = rt['lidar_mount']
    map_dir      = bringup_paths.bringup_map_dir()
    livox_user   = os.path.join(livox_dir, 'config', 'MID360_config.json')

    # Create the launch configuration variables
    fastlio_config    = LaunchConfiguration('fastlio_config')
    lio_sam_config    = LaunchConfiguration('lio_sam_config')
    rviz_config       = LaunchConfiguration('rviz_config')
    auto_save_on_exit = LaunchConfiguration('auto_save_on_exit')

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
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'mapping.rviz'),
        description='Full path to the RVIZ config file',
    )
    declare_auto_save_cmd = DeclareLaunchArgument(
        'auto_save_on_exit',
        default_value='false',
        description='True: clear old map at startup and auto-save on ctrl-C, '
                    'overwriting dddnav_bringup/map/.',
    )

    # Auto-save: clear stale pcds before launch, save fresh ones on shutdown.
    pre_clean_cmd = ExecuteProcess(
        cmd=['bash', '-lc',
             f'rm -rf "{map_dir}/pcd" '
             f'"{map_dir}/poses.pcd" "{map_dir}/edges.pcd" '
             f'"{map_dir}/map.pcd" "{map_dir}/ground.pcd" '
             f'"{map_dir}/GlobalMap.pcd" "{map_dir}/CornerMap.pcd" '
             f'"{map_dir}/SurfMap.pcd" "{map_dir}/trajectory.pcd" '
             f'"{map_dir}/transformations.pcd" && '
             f'mkdir -p "{map_dir}/pcd"'],
        output='screen',
        condition=IfCondition(auto_save_on_exit),
    )
    save_on_shutdown_cmd = RegisterEventHandler(OnShutdown(on_shutdown=[
        ExecuteProcess(
            cmd=['bash', '-lc',
                 'echo "[mapping.launch] auto_save_on_exit=true, saving map..." && '
                 'ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {} && '
                 'ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap '
                 f'"{{resolution: 0.2, destination: \\"{map_dir}/lio_sam\\"}}" && '
                 'echo "[mapping.launch] map saved to ' + map_dir + '"'],
            output='screen',
            condition=IfCondition(auto_save_on_exit),
        ),
    ]))

    # Sensor + static TF
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
    # base_link -> livox_frame static TF (edit lidar_mount in runtime.yaml).
    # map -> odom is published dynamically by LIO-SAM mapOptimization;
    # don't add a static publisher here — it would race with the dynamic one.
    start_sensor_tf_cmd = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='sensor2baselink',
        arguments=[str(m['x']), str(m['y']), str(m['z']),
                   str(m['yaw']), str(m['pitch']), str(m['roll']),
                   'base_link', 'livox_frame'],
    )

    # Bridges (after sensor)
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

    # LIO-SAM back-end (4 nodes share the same yaml + savePCD overlay)
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

    # Pose graph extractor (params from keyframes_mid360.yaml)
    start_posegraph_cmd = TimerAction(period=d['liosam_to_pg'], actions=[
        Node(package='dddnav_utils', executable='liosam_to_posegraph.py',
             name='liosam_to_posegraph', output='screen',
             parameters=[bringup_paths.keyframes_yaml(),
                         bringup_paths.keyframes_save_dir_overlay()]),
    ])

    # Watchdog: warns if odometry / key TF edges stop updating
    start_health_cmd = TimerAction(period=d['health'], actions=[
        Node(package='dddnav_utils', executable='slam_health_monitor.py',
             name='slam_health_monitor', output='screen'),
    ])

    start_rviz_cmd = TimerAction(period=d['rviz'], actions=[
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_config]),
    ])

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_fastlio_config_cmd)
    ld.add_action(declare_lio_sam_config_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_auto_save_cmd)

    # Auto-save
    ld.add_action(pre_clean_cmd)
    ld.add_action(save_on_shutdown_cmd)

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

    # Visualization
    ld.add_action(start_rviz_cmd)
    return ld
