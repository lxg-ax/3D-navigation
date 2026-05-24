"""Resolve install-space paths for dddnav_bringup (no hardcoded workspace roots)."""
import os

from ament_index_python.packages import get_package_share_directory


def bringup_map_dir():
    return os.path.join(get_package_share_directory('dddnav_bringup'), 'map')


def pose_graph_overlay():
    """Overlay for mcl_3dl process (sub_maps node reads pose_graph_dir)."""
    return {'sub_maps': {'ros__parameters': {'pose_graph_dir': bringup_map_dir()}}}


def lio_sam_save_pcd_overlay():
    """LIO-SAM savePCDDirectory must end with '/' (see utility.hpp / mapOptimization)."""
    d = bringup_map_dir()
    return {'savePCDDirectory': d if d.endswith(os.sep) else d + os.sep}


def keyframes_yaml():
    """Path to the shared keyframe-extraction yaml (used by liosam_to_posegraph)."""
    return os.path.join(get_package_share_directory('dddnav_bringup'),
                        'config', 'keyframes_mid360.yaml')


def keyframes_save_dir_overlay():
    """Override save_dir in the keyframes yaml so output lands in
    dddnav_bringup/map/ regardless of how the yaml is shipped."""
    return {'liosam_to_posegraph': {'ros__parameters': {'save_dir': bringup_map_dir()}}}


def runtime_yaml_path():
    """Path to runtime.yaml (timing / lidar mount / initial pose knobs)."""
    return os.path.join(get_package_share_directory('dddnav_bringup'),
                        'config', 'runtime.yaml')


def load_runtime():
    """Parse runtime.yaml. Cached so launch files can call freely."""
    import yaml
    with open(runtime_yaml_path(), 'r') as f:
        return yaml.safe_load(f) or {}


def nav_config_path(profile):
    """Resolve a nav-tuning yaml under dddnav_bringup/config/nav/.

    `profile` may be a bare name (e.g. ``mid360_mapping``), a name with
    suffix (``mid360_mapping.yaml``) or an absolute path. Absolute paths and
    paths that already exist on disk are passed through untouched, which lets
    callers point ``nav_config:=/abs/path/custom.yaml`` for one-off tuning.
    """
    if not profile:
        raise ValueError('nav profile is empty')
    # Absolute path or already-resolved file: trust it.
    if os.path.isabs(profile) and os.path.isfile(profile):
        return profile
    name = profile if profile.endswith('.yaml') else f'{profile}.yaml'
    nav_dir = os.path.join(get_package_share_directory('dddnav_bringup'),
                           'config', 'nav')
    return os.path.join(nav_dir, name)
