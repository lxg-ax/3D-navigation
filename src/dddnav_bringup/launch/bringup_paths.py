"""Resolve install-space paths for dddnav_bringup.

Config layout (post-restructure)::

    config/
      nav_base.yaml                # shared planner/controller defaults
      reality/
        runtime.yaml               # real-robot delays / mounts / driver freq
        keyframes_mid360.yaml      # liosam_to_posegraph thresholds (real)
        nav/<profile>.yaml         # mid360_*.yaml overlays
        tuning/                    # in-field overlays (pose_fusion / mcl)
      simulation/
        runtime.yaml               # sim delays / spawn pose
        fastlio_velodyne_sim.yaml
        lio_sam_velodyne_sim.yaml
        nav/<profile>.yaml         # sim_velodyne_*.yaml overlays

All helpers below resolve against the *installed* share path so they keep
working when the workspace is sourced from ``install/``.
"""
import os

from ament_index_python.packages import get_package_share_directory


# ---------------------------------------------------------------------------
# Map / pose graph (shared between reality and sim — sim writes here, then
# the localization launch reads it back).
# ---------------------------------------------------------------------------

def bringup_map_dir():
    return os.path.join(get_package_share_directory('dddnav_bringup'), 'map')


def pose_graph_overlay():
    """Overlay for mcl_3dl process (sub_maps node reads pose_graph_dir)."""
    return {'sub_maps': {'ros__parameters': {'pose_graph_dir': bringup_map_dir()}}}


def lio_sam_save_pcd_overlay():
    """LIO-SAM savePCDDirectory must end with '/' (utility.hpp / mapOptimization)."""
    d = bringup_map_dir()
    return {'savePCDDirectory': d if d.endswith(os.sep) else d + os.sep}


def keyframes_save_dir_overlay():
    """Override save_dir in keyframes yaml → outputs land in
    dddnav_bringup/map/ regardless of how the yaml is shipped."""
    return {'liosam_to_posegraph': {'ros__parameters': {'save_dir': bringup_map_dir()}}}


# ---------------------------------------------------------------------------
# Variant-aware base directory + runtime.yaml loader.
# ---------------------------------------------------------------------------

_VALID_VARIANTS = ('reality', 'simulation')


def _variant_dir(variant):
    if variant not in _VALID_VARIANTS:
        raise ValueError(
            f"variant must be one of {_VALID_VARIANTS}, got {variant!r}")
    return os.path.join(get_package_share_directory('dddnav_bringup'),
                        'config', variant)


def runtime_yaml_path(variant='reality'):
    return os.path.join(_variant_dir(variant), 'runtime.yaml')


def load_runtime(variant='reality'):
    """Parse the variant's runtime.yaml. Launch files call this once."""
    import yaml
    with open(runtime_yaml_path(variant), 'r') as f:
        return yaml.safe_load(f) or {}


def keyframes_yaml(variant='reality'):
    """Path to the keyframe-extraction yaml used by liosam_to_posegraph.

    Real robot ships the Mid360-tuned values; sim reuses the same defaults
    today, so simulation/keyframes_mid360.yaml is optional and falls back to
    reality/ when absent.
    """
    candidate = os.path.join(_variant_dir(variant), 'keyframes_mid360.yaml')
    if os.path.isfile(candidate):
        return candidate
    # Fallback to reality copy (sim doesn't customise keyframe thresholds yet).
    return os.path.join(_variant_dir('reality'), 'keyframes_mid360.yaml')


# ---------------------------------------------------------------------------
# Nav profile resolution (variant-aware).
# ---------------------------------------------------------------------------

def nav_base_yaml():
    """Shared planner / controller / critic / MCL defaults.

    Lives at ``config/nav_base.yaml`` (one file for both variants). Each
    profile yaml only writes the keys it changes.
    """
    return os.path.join(get_package_share_directory('dddnav_bringup'),
                        'config', 'nav_base.yaml')


def nav_config_path(profile, variant='reality'):
    """Resolve a nav profile to an absolute yaml path.

    ``profile`` accepts:
      * bare name (``mid360_mapping``) — looked up under the variant nav dir
      * with suffix (``mid360_mapping.yaml``)
      * absolute path — passed through untouched

    Falls through to the other variant directory if the file isn't in the
    expected one. Avoids confusing failures when callers mix variant and
    profile name.
    """
    if not profile:
        raise ValueError('nav profile is empty')
    if os.path.isabs(profile) and os.path.isfile(profile):
        return profile
    name = profile if profile.endswith('.yaml') else f'{profile}.yaml'
    primary = os.path.join(_variant_dir(variant), 'nav', name)
    if os.path.isfile(primary):
        return primary
    # Cross-variant fallback (e.g. user passed a sim profile to a reality launch).
    other = 'simulation' if variant == 'reality' else 'reality'
    fallback = os.path.join(_variant_dir(other), 'nav', name)
    if os.path.isfile(fallback):
        return fallback
    return primary  # let the launcher fail loudly with the expected path


def nav_param_chain(profile, variant='reality'):
    """Return ``[nav_base.yaml, profile.yaml]`` filtering out missing files."""
    chain = [nav_base_yaml(), nav_config_path(profile, variant)]
    return [p for p in chain if os.path.isfile(p)]


def nav_profile_argument(default_profile, variant='reality'):
    """Bundle nav_profile launch arg + nav_config OpaqueFunction.

    Usage in a launch file::

        decl, resolve = bringup_paths.nav_profile_argument(
            'mid360_localization', variant='reality')
        ld.add_action(decl)
        ld.add_action(resolve)
        params = [bringup_paths.nav_base_yaml(),
                  LaunchConfiguration('nav_config'),
                  bringup_paths.pose_graph_overlay()]
    """
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.substitutions import LaunchConfiguration

    declare = DeclareLaunchArgument(
        'nav_profile',
        default_value=default_profile,
        description=f'Nav tuning profile under config/{variant}/nav/, or '
                    'an absolute path to a custom yaml',
    )

    def _resolve(context, *args, **kwargs):
        profile = LaunchConfiguration('nav_profile').perform(context)
        return [DeclareLaunchArgument(
            'nav_config',
            default_value=nav_config_path(profile, variant),
            description='Resolved absolute path to nav yaml '
                        '(auto from nav_profile).',
        )]

    return declare, OpaqueFunction(function=_resolve)


# ---------------------------------------------------------------------------
# Camera mount accessor (kept for the *_with_camera launches).
# ---------------------------------------------------------------------------

def camera_mount(rt):
    """Return camera mount xyz/rpy from runtime.yaml with safe defaults."""
    cm = (rt or {}).get('camera_mount') or {}
    return {
        'x':     float(cm.get('x',     0.2)),
        'y':     float(cm.get('y',     0.0)),
        'z':     float(cm.get('z',     0.3)),
        'roll':  float(cm.get('roll',  0.0)),
        'pitch': float(cm.get('pitch', 0.0)),
        'yaw':   float(cm.get('yaw',   0.0)),
    }
