"""Mapping + vision: same as mapping.launch.py plus RealSense + DDRNet.

Camera path is unverified on real hardware. Tunables in
``src/dddnav_bringup/config/runtime.yaml``.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.join(
    get_package_share_directory('dddnav_bringup'), 'launch'))
import bringup_paths
import common_nodes
from common_camera_nodes import vision_nodes


def generate_launch_description():
    bringup_dir  = get_package_share_directory('dddnav_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio')
    lio_sam_dir  = get_package_share_directory('lio_sam')

    rt = bringup_paths.load_runtime()
    cam_mount = bringup_paths.camera_mount(rt)

    fastlio_config = LaunchConfiguration('fastlio_config')
    lio_sam_config = LaunchConfiguration('lio_sam_config')
    rviz_config    = LaunchConfiguration('rviz_config')

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

    ld = LaunchDescription([
        declare_fastlio_config_cmd,
        declare_lio_sam_config_cmd,
        declare_rviz_config_cmd,
    ])

    for action in common_nodes.lidar_driver_and_tf(rt):
        ld.add_action(action)
    for action in common_nodes.lidar_front_end(rt, fastlio_config):
        ld.add_action(action)
    for action in common_nodes.liosam_back_end(
            rt, lio_sam_config,
            bringup_paths.lio_sam_save_pcd_overlay(),
            bringup_paths.keyframes_yaml(),
            bringup_paths.keyframes_save_dir_overlay()):
        ld.add_action(action)

    ld.add_action(common_nodes.rviz_action(rt, rviz_config))

    # Vision: keep colored mask off during mapping to save ~20% CPU.
    for node in vision_nodes(cam_mount, publish_colored_mask=False):
        ld.add_action(node)
    return ld
