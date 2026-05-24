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

    ``profile`` may be a bare name (e.g. ``mid360_mapping``), a name with
    suffix (``mid360_mapping.yaml``) or an absolute path. Absolute paths and
    paths that already exist on disk are passed through untouched, which lets
    callers point ``nav_profile:=/abs/path/custom.yaml`` for one-off tuning.
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


def nav_base_yaml():
    """Common nav yaml: every profile is layered on top of this.

    Anything robot-shape, controller-frequency, or planner-graph related lives
    here. Keep mode-specific knobs (mapping vs localization, depth-camera
    plugin, etc.) in the per-profile overlay.
    """
    return os.path.join(get_package_share_directory('dddnav_bringup'),
                        'config', 'nav', 'base.yaml')


def nav_param_chain(profile):
    """Build the parameter file list for a nav profile.

    Returns ``[base.yaml, profile.yaml]``. ROS 2 lets later parameter files
    override earlier ones, so the profile yaml only needs to repeat keys it
    actually changes.
    """
    chain = [nav_base_yaml(), nav_config_path(profile)]
    return [p for p in chain if os.path.isfile(p)]


def nav_profile_argument(default_profile):
    """Return ``(DeclareLaunchArgument, OpaqueFunction)`` so launch files do
    not have to re-implement the nav_profile→nav_config indirection.

    Usage in a launch file::

        from launch.actions import DeclareLaunchArgument, OpaqueFunction
        ...
        ld.add_action(declare_nav_profile_cmd)
        ld.add_action(OpaqueFunction(function=resolve_nav_config))

    This helper just bundles the boilerplate; see ``nav_param_chain`` if you
    only need the resolved file list synchronously.
    """
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.substitutions import LaunchConfiguration

    declare = DeclareLaunchArgument(
        'nav_profile',
        default_value=default_profile,
        description='Nav tuning profile under dddnav_bringup/config/nav/, '
                    'or an absolute path to a custom yaml',
    )

    def _resolve(context, *args, **kwargs):
        profile = LaunchConfiguration('nav_profile').perform(context)
        return [DeclareLaunchArgument(
            'nav_config',
            default_value=nav_config_path(profile),
            description='Resolved absolute path to nav yaml '
                        '(auto from nav_profile).',
        )]

    return declare, OpaqueFunction(function=_resolve)


def camera_mount(rt):
    """Return camera mount xyz/rpy from runtime.yaml.

    Falls back to a sensible default so legacy ``runtime.yaml`` files without
    a ``camera_mount`` key still work.
    """
    cm = (rt or {}).get('camera_mount') or {}
    return {
        'x':     float(cm.get('x',     0.2)),
        'y':     float(cm.get('y',     0.0)),
        'z':     float(cm.get('z',     0.3)),
        'roll':  float(cm.get('roll',  0.0)),
        'pitch': float(cm.get('pitch', 0.0)),
        'yaw':   float(cm.get('yaw',   0.0)),
    }
