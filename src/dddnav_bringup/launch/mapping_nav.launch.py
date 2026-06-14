"""Mapping + navigation in one shot.

Build a map and run the planner concurrently. Loop closures shift map→odom;
the global plan manager re-queries at ``global_plan_query_frequency`` (5 Hz
by default) so paths refresh automatically on the corrected map.

Save map: see mapping.launch.py.
Tunables: ``src/dddnav_bringup/config/reality/runtime.yaml``.
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


def generate_launch_description():
    bringup_dir  = get_package_share_directory('dddnav_bringup')

    rt = bringup_paths.load_runtime()
    map_dir = bringup_paths.bringup_map_dir()

    fastlio_config    = LaunchConfiguration('fastlio_config')
    lio_sam_config    = LaunchConfiguration('lio_sam_config')
    rviz_config       = LaunchConfiguration('rviz_config')
    auto_save_on_exit = LaunchConfiguration('auto_save_on_exit')

    declare_fastlio_config_cmd = DeclareLaunchArgument(
        'fastlio_config',
        default_value=bringup_paths.fastlio_yaml(),
        description='Full path to the FAST-LIO yaml '
                    '(default: dddnav_bringup/config/reality/slam/fastlio_mid360.yaml)',
    )
    declare_lio_sam_config_cmd = DeclareLaunchArgument(
        'lio_sam_config',
        default_value=bringup_paths.liosam_yaml(),
        description='Full path to the LIO-SAM yaml '
                    '(default: dddnav_bringup/config/reality/slam/liosam_mid360.yaml)',
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'mapping_nav.rviz'),
        description='Full path to the RVIZ config file',
    )
    declare_auto_save_cmd = DeclareLaunchArgument(
        'auto_save_on_exit',
        default_value='false',
        description='True: clear map/ at startup and run save_map_on_exit on '
                    'Ctrl-C.',
    )

    nav_profile_decl, nav_profile_resolve = bringup_paths.nav_profile_argument(
        default_profile='mid360_mapping')
    nav_config_param_files = [
        bringup_paths.nav_base_yaml(),
        LaunchConfiguration('nav_config'),
    ]

    ld = LaunchDescription([
        declare_fastlio_config_cmd,
        declare_lio_sam_config_cmd,
        declare_rviz_config_cmd,
        declare_auto_save_cmd,
        nav_profile_decl,
        nav_profile_resolve,
    ])

    for action in common_nodes.auto_save_actions(
            map_dir, auto_save_on_exit, label='mapping_nav'):
        ld.add_action(action)

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
    for action in common_nodes.nav_stack(rt, nav_config_param_files):
        ld.add_action(action)

    ld.add_action(common_nodes.rviz_action(rt, rviz_config))
    return ld
