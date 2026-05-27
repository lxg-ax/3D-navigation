"""Sim localization + nav: Gazebo Go2 + FAST-LIO + MCL + ESKF + planner.

Prereq: a pose graph under ``share/dddnav_bringup/map`` produced by either
``sim_mapping.launch.py`` or the real-robot mapping launches.

Drives the localization stack the same way the real-robot launch does, but
with use_sim_time everywhere and the perception layer remapped onto the
FAST-LIO body cloud (no Livox bridge in sim).
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.join(
    get_package_share_directory('dddnav_bringup'), 'launch'))
import bringup_paths
import common_nodes


def generate_launch_description():
    bringup_dir = get_package_share_directory('dddnav_bringup')
    go2_dir     = get_package_share_directory('go2_config')
    pose_fusion_yaml_default = os.path.join(
        get_package_share_directory('dddnav_pose_fusion'),
        'config', 'pose_fusion.yaml')

    rt = bringup_paths.load_runtime('simulation')

    fastlio_config   = LaunchConfiguration('fastlio_config')
    rviz_config      = LaunchConfiguration('rviz_config')
    pose_fusion_yaml = LaunchConfiguration('pose_fusion_yaml')
    world            = LaunchConfiguration('world')

    declare_fastlio_config_cmd = DeclareLaunchArgument(
        'fastlio_config',
        default_value=os.path.join(bringup_dir, 'config', 'simulation',
                                   'fastlio_velodyne_sim.yaml'),
        description='FAST-LIO yaml tuned for the sim Velodyne',
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'localization.rviz'),
        description='RViz config',
    )
    declare_pose_fusion_yaml_cmd = DeclareLaunchArgument(
        'pose_fusion_yaml',
        default_value=pose_fusion_yaml_default,
        description='pose_fusion yaml — use config/reality/tuning/*.yaml for overlays',
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(go2_dir, 'worlds',
                                   'slope_with_pillar_2.world'),
        description='Gazebo world file path',
    )

    nav_profile_decl, nav_profile_resolve = bringup_paths.nav_profile_argument(
        default_profile='sim_velodyne_localization', variant='simulation')
    nav_config_param_files = [
        bringup_paths.nav_base_yaml(),
        LaunchConfiguration('nav_config'),
        bringup_paths.pose_graph_overlay(),
    ]

    sim_kwargs = {'use_sim_time': True}

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_dir, 'launch', 'gazebo_velodyne.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    ld = LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        declare_fastlio_config_cmd,
        declare_rviz_config_cmd,
        declare_pose_fusion_yaml_cmd,
        declare_world_cmd,
        nav_profile_decl,
        nav_profile_resolve,
        gazebo_ld,
    ])

    for action in common_nodes.lidar_front_end_sim(rt, fastlio_config,
                                                   **sim_kwargs):
        ld.add_action(action)
    for action in common_nodes.localization_stack(
            rt, nav_config_param_files, pose_fusion_yaml,
            mcl_feature_cloud_topic='/cloud_registered_body',
            sc_cloud_topic='/cloud_registered_body',
            **sim_kwargs):
        ld.add_action(action)
    for action in common_nodes.nav_stack(rt, nav_config_param_files,
                                         **sim_kwargs):
        ld.add_action(action)

    ld.add_action(common_nodes.rviz_action(rt, rviz_config, **sim_kwargs))
    return ld
