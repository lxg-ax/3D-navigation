"""Localization + vision: same as localization.launch.py plus RealSense + DDRNet.

Camera path is unverified on real hardware — see README. Tunables in
``src/dddnav_bringup/config/reality/runtime.yaml``.
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
    bringup_dir       = get_package_share_directory('dddnav_bringup')
    fast_lio_dir      = get_package_share_directory('fast_lio')

    rt = bringup_paths.load_runtime()
    cam_mount = bringup_paths.camera_mount(rt)

    fastlio_config = LaunchConfiguration('fastlio_config')
    rviz_config    = LaunchConfiguration('rviz_config')
    pose_fusion_yaml = LaunchConfiguration('pose_fusion_yaml')

    declare_fastlio_config_cmd = DeclareLaunchArgument(
        'fastlio_config',
        default_value=os.path.join(fast_lio_dir, 'config', 'mid360_pc2.yaml'),
        description='Full path to the FAST-LIO yaml',
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'localization.rviz'),
        description='Full path to the RVIZ config file',
    )
    declare_pose_fusion_yaml_cmd = DeclareLaunchArgument(
        'pose_fusion_yaml',
        default_value=bringup_paths.pose_fusion_yaml(),
        description='pose_fusion yaml — use config/reality/tuning/*.yaml for overlays',
    )

    nav_profile_decl, nav_profile_resolve = bringup_paths.nav_profile_argument(
        default_profile='mid360_localization_with_camera')
    nav_config_param_files = [
        bringup_paths.nav_base_yaml(),
        LaunchConfiguration('nav_config'),
        bringup_paths.pose_graph_overlay(),
    ]

    ld = LaunchDescription([
        declare_fastlio_config_cmd,
        declare_rviz_config_cmd,
        declare_pose_fusion_yaml_cmd,
        nav_profile_decl,
        nav_profile_resolve,
    ])

    for action in common_nodes.lidar_driver_and_tf(rt):
        ld.add_action(action)
    for action in common_nodes.lidar_front_end(rt, fastlio_config):
        ld.add_action(action)
    for action in common_nodes.localization_stack(
            rt, nav_config_param_files, pose_fusion_yaml):
        ld.add_action(action)
    for action in common_nodes.nav_stack(rt, nav_config_param_files):
        ld.add_action(action)

    ld.add_action(common_nodes.rviz_action(rt, rviz_config))

    for node in vision_nodes(cam_mount, publish_colored_mask=True):
        ld.add_action(node)
    return ld
