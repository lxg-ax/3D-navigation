"""Shared node-building blocks used by every dddnav_bringup launch file.

Every entry launch (mapping / mapping_nav / localization, with/without camera)
used to inline the same Livox driver + static TF + LIO-SAM back-end + nav
stack. All of that is now consolidated here so each launch file is just a
composition of a handful of action lists.

Conventions
-----------
* All builders take the parsed ``runtime.yaml`` dict as their first arg, so
  delays / sensor mount / driver freq stay editable from one file.
* Builders return a *list* of launch actions so callers can ``extend()``.
* No builder declares LaunchArguments; the calling launch file is the only
  place arguments are declared (keeps argparse-able help readable).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (ExecuteProcess, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

from bringup_paths import bringup_map_dir


# ---------------------------------------------------------------------------
# Sensor + TF + LiDAR front-end (Livox + livox_pc2_to_liosam + FAST-LIO).
# ---------------------------------------------------------------------------

def lidar_driver_and_tf(rt):
    """Livox driver + base_link→livox_frame static TF.

    Both run with no delay; they are the prerequisites for everything else.
    """
    livox_user = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config', 'MID360_config.json')
    m = rt['lidar_mount']
    return [
        Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen',
            parameters=[{
                'xfer_format': 0, 'multi_topic': 0, 'data_src': 0,
                'publish_freq': float(rt['livox_publish_freq']),
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': livox_user,
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='sensor2baselink',
            arguments=[str(m['x']), str(m['y']), str(m['z']),
                       str(m['yaw']), str(m['pitch']), str(m['roll']),
                       'base_link', 'livox_frame'],
        ),
    ]


def lidar_front_end(rt, fastlio_config):
    """Livox→LIO-SAM bridge + FAST-LIO mapping node."""
    d = rt['delays']
    return [
        TimerAction(period=d['bridges'], actions=[
            Node(package='dddnav_utils', executable='livox_pc2_to_liosam',
                 name='livox_pc2_to_liosam', output='screen',
                 parameters=[{'input_topic': '/livox/lidar',
                              'liosam_output_topic': '/livox/lidar_liosam',
                              'xyzi_output_topic': '/livox/lidar_liosam_xyzi'}]),
        ]),
        TimerAction(period=d['bridges'], actions=[
            Node(package='fast_lio', executable='fastlio_mapping',
                 name='fast_lio', output='screen',
                 parameters=[fastlio_config]),
        ]),
    ]


# ---------------------------------------------------------------------------
# LIO-SAM back-end (4 nodes share one yaml + savePCD overlay) + pose-graph
# extractor + slam health monitor. Used by mapping and mapping_nav modes.
# ---------------------------------------------------------------------------

def liosam_back_end(rt, lio_sam_config, save_pcd_overlay,
                    keyframes_yaml, keyframes_save_dir_overlay):
    """LIO-SAM (imu/proj/feat/opt) + liosam_to_posegraph + health monitor."""
    d = rt['delays']
    lio_sam_params = [lio_sam_config, save_pcd_overlay]

    actions = []
    for execu in ('lio_sam_imuPreintegration', 'lio_sam_imageProjection',
                  'lio_sam_featureExtraction', 'lio_sam_mapOptimization'):
        actions.append(TimerAction(period=d['liosam_back'], actions=[
            Node(package='lio_sam', executable=execu,
                 output='screen', parameters=lio_sam_params),
        ]))

    actions.append(TimerAction(period=d['liosam_to_pg'], actions=[
        Node(package='dddnav_utils', executable='liosam_to_posegraph.py',
             name='liosam_to_posegraph', output='screen',
             parameters=[keyframes_yaml, keyframes_save_dir_overlay]),
    ]))
    actions.append(TimerAction(period=d['health'], actions=[
        Node(package='dddnav_utils', executable='slam_health_monitor.py',
             name='slam_health_monitor', output='screen',
             # In mapping mode pose_fusion is not running, so /odom_filtered
             # has no publisher. Disable that watch path here; topic/TF
             # liveness is still meaningful — those WARNs flag a stalled
             # mapOptimization (which is the real problem, not noise).
             # ok_timeout/fail_timeout are also relaxed: LIO-SAM
             # mapOptimization throttles by mappingProcessInterval (0.1 s)
             # and a single optimisation can spike to ~150 ms, so 0.5 s
             # is too tight on stamp_age.
             parameters=[{
                 'filtered_odom_topic': '',
                 'ok_timeout':   1.0,
                 'fail_timeout': 3.0,
             }]),
    ]))
    return actions


# ---------------------------------------------------------------------------
# Localization stack: MCL 3DL + ESKF pose fusion + MCL feature + bootstrap
# initial pose.
# ---------------------------------------------------------------------------

def localization_stack(rt, nav_config_params, pose_fusion_yaml,
                       sc_init_enabled=True, sc_init_yaml=None):
    """MCL 3DL + pose_fusion + mcl_feature + (optional) SC global init +
    operator-supplied initial-pose bootstrap.

    ``nav_config_params`` is the full parameters list (common yaml + profile
    yaml + dynamic overlays) so each node sees the same merged config.

    ``sc_init_enabled`` toggles the Scan Context global localiser. It runs
    before the operator-supplied bootstrap so the first /initial_3d_pose
    publication wins; if SC fails to find a confident match the bootstrap
    keeps the legacy behaviour (use runtime.yaml.initial_pose).
    ``sc_init_yaml`` is an optional dict / yaml file with overrides. The
    helper auto-injects the canonical sc_db / poses paths.
    """
    d = rt['delays']
    ip = rt['initial_pose']

    initial_pose_msg = (
        f"{{header: {{frame_id: 'map'}}, "
        f"pose: {{pose: {{position: {{x: {ip['x']}, y: {ip['y']}, z: {ip['z']}}}, "
        f"orientation: {{w: 1.0}}}}}}}}"
    )

    actions = [
        TimerAction(period=d['mcl_3dl'], actions=[
            Node(package='mcl_3dl', executable='mcl_3dl', output='screen',
                 parameters=[*nav_config_params,
                             {'publish_tf': False, 'publish_odom_tf': False}],
                 remappings=[('odom', '/Odometry'),
                             ('laser_cloud_sharp', '/laser_cloud_sharp'),
                             ('laser_cloud_less_sharp', '/laser_cloud_less_sharp'),
                             ('laser_cloud_flat', '/laser_cloud_flat'),
                             ('laser_cloud_less_flat', '/laser_cloud_less_flat')]),
        ]),
        TimerAction(period=d['pose_fusion'], actions=[
            Node(package='dddnav_pose_fusion', executable='pose_fusion_node',
                 name='pose_fusion', output='screen',
                 parameters=[pose_fusion_yaml]),
        ]),
        TimerAction(period=d['mcl_feature'], actions=[
            Node(package='dddnav_mcl_feature', executable='mcl_feature',
                 output='screen', parameters=nav_config_params,
                 remappings=[('/lslidar_point_cloud', '/livox/lidar_liosam_xyzi'),
                             ('/odom', '/Odometry')]),
        ]),
    ]

    if sc_init_enabled:
        # Auto-derive sc_db / poses paths from the same map dir the rest of
        # the stack uses. Caller can fully override via sc_init_yaml.
        map_dir = bringup_map_dir()
        sc_params = [{
            'sc_db_path':     os.path.join(map_dir, 'lio_sam', 'sc_db.bin'),
            'poses_pcd_path': os.path.join(map_dir, 'poses.pcd'),
            'cloud_topic':    '/livox/lidar_liosam_xyzi',
            'odom_topic':     '/odom_filtered',
            'init_pose_topic': '/initial_3d_pose',
            # Watchdog defaults: 2 Hz check, 8 s warm-up, ±6 m / ±60deg gate
            # against the fused pose. Override via sc_init_yaml when needed.
            'enable_watchdog':           True,
            'watchdog_check_hz':         2.0,
            'watchdog_warmup_sec':       8.0,
            'relocate_max_jump_m':       6.0,
            'relocate_max_jump_yaw':     1.05,
            'relocate_min_delta':        0.05,
            'relocate_max_candidate_dist': 0.25,
            'relocate_consensus':        4,
            'relocate_holdoff_sec':      10.0,
        }]
        if sc_init_yaml:
            sc_params.append(sc_init_yaml)
        # Run a hair before mcl_3dl finishes spinning up so its first
        # particle distribution can already be re-centred.
        actions.append(TimerAction(period=max(d['mcl_3dl'] - 0.5, 1.0), actions=[
            Node(package='dddnav_utils', executable='sc_global_init',
                 name='sc_global_init', output='screen',
                 parameters=sc_params),
        ]))

    # Operator-supplied seed (legacy) — still useful when SC misses or when
    # there is no map yet (no sc_db.bin → SC node returns early).
    actions.append(TimerAction(period=d['initial_pose'], actions=[
        ExecuteProcess(cmd=[
            'ros2', 'topic', 'pub', '--once', '/initial_3d_pose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            initial_pose_msg,
        ], output='screen'),
    ]))
    return actions


# ---------------------------------------------------------------------------
# Navigation stack (global planner + p2p_move_base + clicked2goal).
# ---------------------------------------------------------------------------

def nav_stack(rt, nav_config_params):
    d = rt['delays']
    return [
        TimerAction(period=d['global_planner'], actions=[
            Node(package='global_planner', executable='global_planner_node',
                 output='screen', parameters=nav_config_params)]),
        TimerAction(period=d['move_base'], actions=[
            Node(package='p2p_move_base', executable='p2p_move_base_node',
                 output='screen', parameters=nav_config_params)]),
        TimerAction(period=d['clicked_goal'], actions=[
            Node(package='p2p_move_base', executable='clicked2goal.py',
                 name='clicked2goal', output='screen')]),
        # Telemetry: planner / control / front-end rate + payload health
        # to /diagnostics. Cheap (~1Hz timer, small subscriptions) and
        # invaluable when tuning. Pair with slam_health_monitor for the
        # complete picture.
        TimerAction(period=d['move_base'], actions=[
            Node(package='dddnav_utils', executable='nav_perf_monitor.py',
                 name='nav_perf_monitor', output='screen')]),
    ]


# ---------------------------------------------------------------------------
# RViz (delayed so there is something to render when it pops up).
# ---------------------------------------------------------------------------

def rviz_action(rt, rviz_config):
    return TimerAction(period=rt['delays']['rviz'], actions=[
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', rviz_config]),
    ])


# ---------------------------------------------------------------------------
# Auto-save: clear stale pcds at startup, fire /save_full_map on shutdown.
# Replaces the old ``ros2 service call`` shell pipeline that was sensitive to
# launch tear-down ordering.
# ---------------------------------------------------------------------------

_AUTO_SAVE_PRECLEAN_FILES = (
    'poses.pcd', 'edges.pcd', 'map.pcd', 'ground.pcd',
    'GlobalMap.pcd', 'CornerMap.pcd', 'SurfMap.pcd',
    'trajectory.pcd', 'transformations.pcd',
)


def auto_save_actions(map_dir, auto_save_flag, label):
    """Pre-clean + save-on-shutdown actions, gated by ``auto_save_flag``.

    ``label`` is a free-form string included in log lines so users can tell
    which launch file fired the save (mapping vs mapping_nav).
    """
    rm_files = ' '.join(f'"{map_dir}/{f}"' for f in _AUTO_SAVE_PRECLEAN_FILES)
    pre_clean = ExecuteProcess(
        cmd=['bash', '-lc',
             f'rm -rf "{map_dir}/pcd" {rm_files} && mkdir -p "{map_dir}/pcd"'],
        output='screen',
        condition=IfCondition(auto_save_flag),
    )
    save_on_shutdown = RegisterEventHandler(OnShutdown(on_shutdown=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'dddnav_utils', 'save_map_on_exit.py',
                 '--map-dir', map_dir, '--label', label],
            output='screen',
            condition=IfCondition(auto_save_flag),
        ),
    ]))
    return [pre_clean, save_on_shutdown]
