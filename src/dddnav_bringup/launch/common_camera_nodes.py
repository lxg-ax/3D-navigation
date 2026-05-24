"""公共视觉感知节点: RealSense + DDRNet 语义分割 + 语义点云.

用法: 在同目录 launch 文件中 ``from common_camera_nodes import vision_nodes``
ament_cmake install(DIRECTORY launch ...) 会把本文件一起装到 share 目录,
launch 运行时 sys.path 包含 ``share/dddnav_bringup/launch``.

camera_mount(xyz/rpy) 现在从 ``dddnav_bringup/config/runtime.yaml`` 的
``camera_mount`` 块读取, 与 ``lidar_mount`` 同级, 保证 "不改 launch 只改 yaml".
"""

from launch_ros.actions import Node
from launch.actions import TimerAction


def realsense_node():
    """RealSense D435/D455 RGBD 相机."""
    return Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        name='camera', namespace='camera', output='screen',
        parameters=[{
            'enable_infra1': False, 'enable_infra2': False,
            'enable_color': True, 'enable_depth': True,
            'depth_module.emitter_enabled': 1,
            'depth_module.profile': '848x480x30',
            'rgb_camera.color_profile': '848x480x30',
            'enable_gyro': False, 'enable_accel': False,
        }])


def camera_tf_nodes(mount):
    """base_link → camera_link → optical_frame TF.

    ``mount`` is the dict returned by ``bringup_paths.camera_mount(rt)``.
    """
    return [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base2camera',
             arguments=[str(mount['x']), str(mount['y']), str(mount['z']),
                        str(mount['yaw']), str(mount['pitch']), str(mount['roll']),
                        'base_link', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera2optical',
             arguments=['0.0', '0.0', '0.0', '-1.571', '0.0', '-1.571',
                        'camera_link', 'camera_depth_optical_frame']),
    ]


def ddrnet_node(publish_colored_mask=False):
    """DDRNet 语义分割 TensorRT 推理, delay 2s 等相机启动."""
    return TimerAction(period=2.0, actions=[
        Node(package='dddnav_semantic_segmentation', executable='ddrnet_ros_img_sub.py',
             output='screen',
             parameters=[{
                 'publish_colored_mask_result': publish_colored_mask,
                 'use_class_based_mask_result': True,
             }]),
    ])


def semantic_pointcloud_node(max_distance=5.0, sample_step=2,
                             voxel_size=0.05, exclude_class=None):
    """语义 mask + 深度图 → 语义点云, delay 4s 等 TRT 引擎加载."""
    if exclude_class is None:
        exclude_class = [0]
    return TimerAction(period=4.0, actions=[
        Node(package='dddnav_semantic_segmentation', executable='semantic_segmentation2point_cloud',
             output='screen',
             parameters=[{
                 'max_distance': max_distance,
                 'sample_step': sample_step,
                 'voxel_size': voxel_size,
                 'exclude_class': exclude_class,
             }],
             remappings=[
                 ('/camera_info', '/camera/camera/depth/camera_info'),
                 ('/image_rect_raw', '/camera/camera/depth/image_rect_raw'),
             ]),
    ])


def vision_nodes(camera_mount, publish_colored_mask=False,
                 max_distance=5.0, exclude_class=None):
    """所有视觉感知节点的组合.

    ``camera_mount`` 直接传 ``bringup_paths.camera_mount(rt)`` 的结果,
    避免在 launch 文件里再次拼装位姿字典.
    """
    nodes = [realsense_node()]
    nodes.extend(camera_tf_nodes(camera_mount))
    nodes.append(ddrnet_node(publish_colored_mask=publish_colored_mask))
    nodes.append(semantic_pointcloud_node(max_distance=max_distance,
                                          exclude_class=exclude_class))
    return nodes
