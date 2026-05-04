# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_color': True,
                'enable_depth': True,
                'depth_module.emitter_enabled': 1,
                'depth_module.profile': '848x480x30',
                'rgb_camera.color_profile': '848x480x30',
                'pointcloud.enable': True,
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'unite_imu_method': 2
        }]
    )

    trt_node = Node(
        package='dddnav_semantic_segmentation',
        executable='ddrnet_ros_img_sub.py',
        remappings=[
            ('/camera/camera/color/image_raw', '/camera/camera/color/image_raw')
        ]
    )


    mask2pointcloud_node = Node(
        package='dddnav_semantic_segmentation',
        executable='semantic_segmentation2point_cloud',
        parameters=[{
            'max_distance': 6.0,
            'sample_step': 2,
            'voxel_size': 0.05
        }],
        remappings=[
            ('/camera_info', '/camera/camera/depth/camera_info'),
            ('/ddrnet_inferenced_mask', '/ddrnet_inferenced_mask'),
            ('/image_rect_raw', '/camera/camera/depth/image_rect_raw')
        ]
    )
    
    pkg_path = get_package_share_directory('dddnav_semantic_segmentation')
    rviz2_node = Node(
            package="rviz2",
            namespace=namespace,
            executable="rviz2",
            output="screen",
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'rs_semantic_segmentation_launch.rviz')]
    ) 


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    

    ld.add_action(realsense_camera_node)
    ld.add_action(trt_node)
    ld.add_action(mask2pointcloud_node)
    ld.add_action(rviz2_node)

    return ld