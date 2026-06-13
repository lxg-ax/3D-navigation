"""Standalone pose_fusion launch.

Drop-in for `localization*.launch.py`: spawn this *after* MCL 3DL is up, set
mcl_3dl publish_tf=false in your mcl yaml so that pose_fusion owns map→odom.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('dddnav_bringup'),
        'config', 'reality', 'pose_fusion.yaml')

    return LaunchDescription([
        Node(
            package='dddnav_pose_fusion',
            executable='pose_fusion_node',
            name='pose_fusion',
            output='screen',
            parameters=[cfg],
        ),
    ])
