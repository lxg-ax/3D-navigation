#!/usr/bin/env python3
"""Atomic map saver triggered by ``mapping*.launch.py`` on shutdown.

The previous implementation chained two ``ros2 service call`` invocations
inside a bash one-liner registered with ``OnShutdown``. Two pain points:

* The launch tear-down can clobber the environment (PYTHONPATH /
  AMENT_PREFIX_PATH) before the bash call runs, so the ``ros2`` CLI
  occasionally fails to find the right RMW or service typesupport.
* Users had to memorise that ``/save_liosam_posegraph`` *must* run first or
  LIO-SAM will ``rm -r`` the keyframe pcds on top of itself.

This node wraps the canonical save sequence (pose graph → LIO-SAM dump) in
plain rclpy, with retries + per-step timeouts, and writes everything under
``--map-dir``. ``--label`` only colours log lines so users can tell which
launch fired the save.
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


def _wait_service(node, client, name, timeout):
    """Block up to ``timeout`` seconds until ``client`` is available."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.5):
            return True
        rclpy.spin_once(node, timeout_sec=0.0)
    node.get_logger().warn(f'service {name} not advertised after {timeout:.1f}s')
    return False


def _call_empty(node, client, name, timeout):
    """Call an Empty service synchronously, returning True on success."""
    fut = client.call_async(Empty.Request())
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.exception() is None
    node.get_logger().error(f'service {name} timed out after {timeout:.1f}s')
    return False


def _call_save_lio_sam(node, map_dir, timeout):
    """Call /lio_sam/save_map. The service type lives in lio_sam package, so
    we import it lazily — that lets the rest of this script stay useful even
    when only the pose graph save is wanted (during debugging)."""
    try:
        from lio_sam.srv import SaveMap                      # noqa: WPS433
    except ImportError as e:
        node.get_logger().error(f'lio_sam.srv.SaveMap unavailable: {e}')
        return False

    client = node.create_client(SaveMap, '/lio_sam/save_map')
    if not _wait_service(node, client, '/lio_sam/save_map', timeout):
        return False

    req = SaveMap.Request()
    req.resolution = 0.2
    req.destination = os.path.join(map_dir, 'lio_sam')
    fut = client.call_async(req)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.exception() is None
    node.get_logger().error('/lio_sam/save_map timed out')
    return False


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-dir', required=True,
                        help='Pose graph + LIO-SAM dump directory')
    parser.add_argument('--label', default='mapping',
                        help='Free-form label used in log lines')
    parser.add_argument('--service-timeout', type=float, default=15.0,
                        help='Per-service wait + call timeout (seconds)')
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node('save_map_on_exit')
    log = node.get_logger()
    log.info(f'[{args.label}] saving map to {args.map_dir}')
    os.makedirs(args.map_dir, exist_ok=True)

    # Step 1: pose graph (must run first; LIO-SAM save will rm -r its dir).
    pg_client = node.create_client(Empty, '/save_liosam_posegraph')
    rc = 0
    if _wait_service(node, pg_client, '/save_liosam_posegraph',
                     args.service_timeout):
        if _call_empty(node, pg_client, '/save_liosam_posegraph',
                       args.service_timeout):
            log.info(f'[{args.label}] pose graph saved')
        else:
            log.error(f'[{args.label}] pose graph save failed')
            rc = 2
    else:
        rc = 2

    # Step 2: LIO-SAM dump under <map_dir>/lio_sam/.
    if _call_save_lio_sam(node, args.map_dir, args.service_timeout):
        log.info(f'[{args.label}] LIO-SAM dumps written')
    else:
        log.error(f'[{args.label}] LIO-SAM save failed')
        rc = max(rc, 2)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
