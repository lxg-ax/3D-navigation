"""Sim build-while-navigating: Gazebo Go2 + FAST-LIO + LIO-SAM + planner.

Same as sim_mapping but adds the global / local planner so you can drop
goals in RViz while the back-end is still building the pose graph. Loop
closures shift map→odom and the global plan manager re-queries every
0.2 s, so paths refresh against the corrected map automatically.
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

    rt = bringup_paths.load_runtime('simulation')
    map_dir = bringup_paths.bringup_map_dir()

    fastlio_config    = LaunchConfiguration('fastlio_config')
    lio_sam_config    = LaunchConfiguration('lio_sam_config')
    rviz_config       = LaunchConfiguration('rviz_config')
    auto_save_on_exit = LaunchConfiguration('auto_save_on_exit')
    world             = LaunchConfiguration('world')

    declare_fastlio_config_cmd = DeclareLaunchArgument(
        'fastlio_config',
        default_value=os.path.join(bringup_dir, 'config', 'simulation',
                                   'fastlio_velodyne_sim.yaml'),
        description='FAST-LIO yaml tuned for the sim Velodyne',
    )
    declare_lio_sam_config_cmd = DeclareLaunchArgument(
        'lio_sam_config',
        default_value=os.path.join(bringup_dir, 'config', 'simulation',
                                   'lio_sam_velodyne_sim.yaml'),
        description='LIO-SAM yaml tuned for the sim Velodyne',
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'mapping_nav.rviz'),
        description='RViz config',
    )
    declare_auto_save_cmd = DeclareLaunchArgument(
        'auto_save_on_exit',
        default_value='false',
        description='Clear map/ at startup and run save_map_on_exit on Ctrl-C',
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(go2_dir, 'worlds',
                                   'slope_with_pillar_2.world'),
        description='Gazebo world file path',
    )

    nav_profile_decl, nav_profile_resolve = bringup_paths.nav_profile_argument(
        default_profile='sim_velodyne_mapping', variant='simulation')
    nav_config_param_files = [
        bringup_paths.nav_base_yaml(),
        LaunchConfiguration('nav_config'),
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
        declare_lio_sam_config_cmd,
        declare_rviz_config_cmd,
        declare_auto_save_cmd,
        declare_world_cmd,
        nav_profile_decl,
        nav_profile_resolve,
        gazebo_ld,
    ])

    for action in common_nodes.auto_save_actions(
            map_dir, auto_save_on_exit, label='sim_mapping_nav'):
        ld.add_action(action)

    for action in common_nodes.lidar_front_end_sim(rt, fastlio_config,
                                                   **sim_kwargs):
        ld.add_action(action)
    for action in common_nodes.liosam_back_end(
            rt, lio_sam_config,
            bringup_paths.lio_sam_save_pcd_overlay(),
            bringup_paths.keyframes_yaml('simulation'),
            bringup_paths.keyframes_save_dir_overlay(),
            **sim_kwargs):
        ld.add_action(action)
    for action in common_nodes.nav_stack(rt, nav_config_param_files,
                                         **sim_kwargs):
        ld.add_action(action)

    ld.add_action(common_nodes.rviz_action(rt, rviz_config, **sim_kwargs))
    return ld
