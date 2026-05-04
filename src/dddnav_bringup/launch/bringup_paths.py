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
