"""Sim mapping: Gazebo Go2 (VLP-16) + FAST-LIO + LIO-SAM back-end.

Self-contained: brings up the Gazebo world, robot description, gait
controller, FAST-LIO front-end, LIO-SAM back-end, pose-graph extractor,
slam_health_monitor and (optionally) auto-save on Ctrl-C.

Drive the robot with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

Save manually:
  ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
  ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"

Or pass ``auto_save_on_exit:=true`` to land the map under
``share/dddnav_bringup/map`` and reuse it from sim_localization.launch.py.
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
        default_value=os.path.join(bringup_dir, 'rviz', 'mapping.rviz'),
        description='RViz config (reuses the real-mapping panel)',
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
        gazebo_ld,
    ])

    for action in common_nodes.auto_save_actions(
            map_dir, auto_save_on_exit, label='sim_mapping'):
        ld.add_action(action)

    # No Livox driver / Livox bridge in sim; URDF already publishes the
    # base_link → velodyne / imu_link static TFs.
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

    ld.add_action(common_nodes.rviz_action(rt, rviz_config, **sim_kwargs))
    return ld
