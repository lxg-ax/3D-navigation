"""Go2 / Velodyne localization: injects sub_maps.pose_graph_dir to dddnav_bringup/map (YAML no longer sets it)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    p2p_share = get_package_share_directory('p2p_move_base')
    cfg = os.path.join(p2p_share, 'config', 'go2_localization.yaml')
    rviz_cfg = os.path.join(p2p_share, 'rviz', 'p2p_move_base_localization.rviz')
    use_sim = LaunchConfiguration('use_sim_time')
    map_dir = os.path.join(get_package_share_directory('dddnav_bringup'), 'map')
    pose_graph_overlay = {'sub_maps': {'ros__parameters': {'pose_graph_dir': map_dir}}}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='lego_loam_bor',
            executable='mcl_feature',
            output='screen',
            respawn=False,
            parameters=[cfg, {'use_sim_time': use_sim}],
            remappings=[
                ('/lslidar_point_cloud', '/velodyne_points'),
                ('/odom', '/odom'),
            ],
        ),
        Node(
            package='mcl_3dl',
            executable='mcl_3dl',
            output='screen',
            respawn=False,
            parameters=[cfg, pose_graph_overlay, {'use_sim_time': use_sim}],
        ),
        Node(
            package='global_planner',
            executable='global_planner_node',
            output='screen',
            respawn=False,
            parameters=[cfg, {'use_sim_time': use_sim}],
        ),
        Node(
            package='p2p_move_base',
            executable='p2p_move_base_node',
            output='screen',
            respawn=False,
            parameters=[cfg, {'use_sim_time': use_sim}],
        ),
        Node(
            package='p2p_move_base',
            executable='clicked2goal.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': use_sim}],
        ),
    ])
