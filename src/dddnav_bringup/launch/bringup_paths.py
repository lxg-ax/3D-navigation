"""Resolve install-space paths for dddnav_bringup.

Config layout::

    config/
      nav_base.yaml                # shared planner / controller defaults
      reality/
        runtime.yaml               # delays / mounts / driver freq
        keyframes_mid360.yaml      # liosam_to_posegraph thresholds
        pose_fusion.yaml           # ESKF Q/R, ZUPT, adaptive Q
        slam/
          fastlio_mid360.yaml      # FAST-LIO front-end (mirrors upstream config)
          liosam_mid360.yaml       # LIO-SAM back-end (mirrors upstream config)
          IMU_NOTES.md             # cross-algo IMU param mapping
        nav/<profile>.yaml         # mid360_*.yaml overlays
        tuning/                    # in-field overlays (pose_fusion / mcl / mppi)

Helpers resolve against the *installed* share path so they keep working
when the workspace is sourced from ``install/``.
"""
import os

from ament_index_python.packages import get_package_share_directory


def _share():
    return get_package_share_directory('dddnav_bringup')


def _config():
    return os.path.join(_share(), 'config')


def _reality():
    return os.path.join(_config(), 'reality')


def bringup_map_dir():
    return os.path.join(_share(), 'map')


def pose_graph_overlay():
    return {'sub_maps': {'ros__parameters': {'pose_graph_dir': bringup_map_dir()}}}


def lio_sam_save_pcd_overlay():
    # LIO-SAM savePCDDirectory must end with '/'.
    d = bringup_map_dir()
    return {'savePCDDirectory': d if d.endswith(os.sep) else d + os.sep}


def keyframes_save_dir_overlay():
    return {'liosam_to_posegraph': {'ros__parameters': {'save_dir': bringup_map_dir()}}}


def runtime_yaml_path():
    return os.path.join(_reality(), 'runtime.yaml')


def load_runtime():
    import yaml
    with open(runtime_yaml_path(), 'r') as f:
        return yaml.safe_load(f) or {}


def keyframes_yaml():
    return os.path.join(_reality(), 'keyframes_mid360.yaml')


def pose_fusion_yaml():
    return os.path.join(_reality(), 'pose_fusion.yaml')


def _slam():
    return os.path.join(_reality(), 'slam')


def fastlio_yaml(name='fastlio_mid360.yaml'):
    """FAST-LIO front-end yaml under reality/slam/."""
    return os.path.join(_slam(), name)


def liosam_yaml(name='liosam_mid360.yaml'):
    """LIO-SAM back-end yaml under reality/slam/."""
    return os.path.join(_slam(), name)


def nav_base_yaml():
    return os.path.join(_config(), 'nav_base.yaml')


def nav_config_path(profile):
    """Resolve a nav profile name (or absolute path) to a yaml file."""
    if not profile:
        raise ValueError('nav profile is empty')
    if os.path.isabs(profile) and os.path.isfile(profile):
        return profile
    name = profile if profile.endswith('.yaml') else f'{profile}.yaml'
    return os.path.join(_reality(), 'nav', name)


def nav_param_chain(profile):
    chain = [nav_base_yaml(), nav_config_path(profile)]
    return [p for p in chain if os.path.isfile(p)]


def nav_profile_argument(default_profile):
    """Bundle nav_profile launch arg + nav_config OpaqueFunction."""
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.substitutions import LaunchConfiguration

    declare = DeclareLaunchArgument(
        'nav_profile',
        default_value=default_profile,
        description='Nav tuning profile under config/reality/nav/, '
                    'or an absolute path to a custom yaml',
    )

    def _resolve(context, *args, **kwargs):
        profile = LaunchConfiguration('nav_profile').perform(context)
        return [DeclareLaunchArgument(
            'nav_config',
            default_value=nav_config_path(profile),
            description='Resolved absolute path to the nav yaml.',
        )]

    return declare, OpaqueFunction(function=_resolve)


def camera_mount(rt):
    cm = (rt or {}).get('camera_mount') or {}
    return {
        'x':     float(cm.get('x',     0.2)),
        'y':     float(cm.get('y',     0.0)),
        'z':     float(cm.get('z',     0.3)),
        'roll':  float(cm.get('roll',  0.0)),
        'pitch': float(cm.get('pitch', 0.0)),
        'yaw':   float(cm.get('yaw',   0.0)),
    }
