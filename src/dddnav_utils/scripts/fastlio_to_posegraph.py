#!/usr/bin/env python3
"""
FAST-LIO2 to dddnav pose graph converter.

Subscribes to FAST-LIO2 output and saves a pose graph compatible with
dddnav mcl_3dl localization.

Output structure:
  <save_dir>/
    poses.pcd          - PointXYZIRPYT keyframe poses (map -> base_link)
    pcd/
      0_feature.pcd    - corner/feature cloud in base_link frame
      0_ground.pcd     - ground cloud in base_link frame
      ...
"""

import os
import sys
import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Empty
import sensor_msgs_py.point_cloud2 as pc2


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to roll/pitch/yaw (XYZ extrinsic) using pure numpy."""
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _rpy_to_matrix(roll, pitch, yaw):
    """Build rotation matrix from roll/pitch/yaw (XYZ extrinsic)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ]
    ])


def _matrix_to_rpy(R):
    """Extract roll/pitch/yaw from rotation matrix (XYZ extrinsic)."""
    pitch = math.asin(-R[2, 0])
    roll  = math.atan2(R[2, 1], R[2, 2])
    yaw   = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def transform_inverse(tx, ty, tz, roll, pitch, yaw):
    """Return inverse transform (map->base to base->map)."""
    R = _rpy_to_matrix(roll, pitch, yaw)
    t = np.array([tx, ty, tz])
    R_inv = R.T
    t_inv = -R_inv @ t
    rpy_inv = _matrix_to_rpy(R_inv)
    return t_inv[0], t_inv[1], t_inv[2], rpy_inv[0], rpy_inv[1], rpy_inv[2]


def transform_cloud_inverse(points_xyz, tx, ty, tz, roll, pitch, yaw):
    """Transform points from map frame to base_link frame."""
    R = _rpy_to_matrix(roll, pitch, yaw)
    t = np.array([tx, ty, tz])
    # map -> base_link: p_base = R^T * (p_map - t)
    pts = points_xyz - t
    pts_base = (R.T @ pts.T).T
    return pts_base


def save_pcd_xyzi(filename, points_xyz, intensity=0.0):
    """Save PointXYZI pcd file in ASCII format."""
    n = len(points_xyz)
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for pt in points_xyz:
            f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f} {intensity:.6f}\n")


def save_poses_pcd(filename, poses):
    """
    Save poses as PointXYZIRPYT pcd (ASCII).
    poses: list of (x, y, z, roll, pitch, yaw, intensity/index)
    """
    n = len(poses)
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity roll pitch yaw time\n")
        f.write("SIZE 4 4 4 4 4 4 4 8\n")
        f.write("TYPE F F F F F F F F\n")
        f.write("COUNT 1 1 1 1 1 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for p in poses:
            x, y, z, roll, pitch, yaw, idx = p
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {float(idx):.1f} "
                    f"{roll:.6f} {pitch:.6f} {yaw:.6f} 0.0\n")


class FastLioToPoseGraph(Node):

    def __init__(self):
        super().__init__('fastlio_to_posegraph')

        self.declare_parameter('save_dir', '/tmp/fastlio_map')
        self.declare_parameter('keyframe_dist', 0.5)      # meters
        self.declare_parameter('keyframe_angle', 0.3)     # radians
        self.declare_parameter('ground_angle_thresh', 10.0)  # degrees, points below this elevation are ground

        self.save_dir = self.get_parameter('save_dir').value
        self.kf_dist = self.get_parameter('keyframe_dist').value
        self.kf_angle = self.get_parameter('keyframe_angle').value
        self.ground_angle_thresh = math.radians(
            self.get_parameter('ground_angle_thresh').value)

        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(os.path.join(self.save_dir, 'pcd'), exist_ok=True)

        self.keyframes = []   # list of (x,y,z,roll,pitch,yaw)
        self.last_kf_pose = None
        self.current_odom = None
        self.current_cloud = None
        self.cloud_stamp = None
        self.odom_stamp = None

        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry', self.odom_cb, 10)
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cloud_cb, 10)

        self.srv_save = self.create_service(
            Empty, 'save_fastlio_posegraph', self.save_cb)

        # Publishers for perception_3d in mapping mode
        self.pub_map = self.create_publisher(PointCloud2, 'lego_loam_map', 10)
        self.pub_ground = self.create_publisher(PointCloud2, 'lego_loam_ground', 10)
        # Publish accumulated map every 2 seconds
        self.create_timer(2.0, self.publish_map_cb)

        self.get_logger().info(
            f'FastLio->PoseGraph ready. save_dir={self.save_dir}, '
            f'kf_dist={self.kf_dist}m, kf_angle={math.degrees(self.kf_angle):.1f}deg')
        self.get_logger().info('Call service /save_fastlio_posegraph to save.')

        # accumulated map clouds for publishing to perception_3d
        self.acc_feature_pts = []   # list of np arrays (map frame)
        self.acc_ground_pts = []

    def odom_cb(self, msg):
        self.current_odom = msg
        self.odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._try_add_keyframe()

    def cloud_cb(self, msg):
        self.current_cloud = msg
        self.cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _try_add_keyframe(self):
        if self.current_odom is None or self.current_cloud is None:
            return

        # Check time sync (within 0.2s)
        if abs(self.odom_stamp - self.cloud_stamp) > 0.2:
            return

        p = self.current_odom.pose.pose.position
        q = self.current_odom.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        pose = (p.x, p.y, p.z, roll, pitch, yaw)

        if self.last_kf_pose is None:
            self._add_keyframe(pose)
            return

        lx, ly, lz, lr, lp, lyaw = self.last_kf_pose
        dist = math.sqrt((p.x-lx)**2 + (p.y-ly)**2 + (p.z-lz)**2)
        d_yaw = abs(yaw - lyaw)
        if d_yaw > math.pi:
            d_yaw = 2*math.pi - d_yaw

        if dist >= self.kf_dist or d_yaw >= self.kf_angle:
            self._add_keyframe(pose)

    def _add_keyframe(self, pose):
        idx = len(self.keyframes)
        self.keyframes.append(pose)
        self.last_kf_pose = pose

        # Extract points from current cloud (map frame)
        try:
            pts_raw = list(pc2.read_points(
                self.current_cloud, field_names=('x', 'y', 'z'), skip_nans=True))
            pts = np.array([[p[0], p[1], p[2]] for p in pts_raw], dtype=np.float64)
        except Exception as e:
            self.get_logger().warn(f'Cloud read error: {e}')
            return

        if len(pts) == 0:
            return

        # Transform to base_link frame
        tx, ty, tz, roll, pitch, yaw = pose
        pts_base = transform_cloud_inverse(pts, tx, ty, tz, roll, pitch, yaw)

        # Split into ground and feature by elevation angle from sensor (base_link frame)
        ranges = np.sqrt(pts_base[:, 0]**2 + pts_base[:, 1]**2)
        elev = np.arctan2(-pts_base[:, 2], ranges + 1e-6)

        # ground: elevation angle is negative (pointing down), below threshold
        ground_mask = elev < -self.ground_angle_thresh
        feature_mask = ~ground_mask

        ground_pts = pts_base[ground_mask]
        feature_pts = pts_base[feature_mask]

        # Ensure at least some points in each
        if len(ground_pts) == 0:
            ground_pts = pts_base[:min(10, len(pts_base))]
        if len(feature_pts) == 0:
            feature_pts = pts_base

        pcd_dir = os.path.join(self.save_dir, 'pcd')
        save_pcd_xyzi(os.path.join(pcd_dir, f'{idx}_feature.pcd'), feature_pts)
        save_pcd_xyzi(os.path.join(pcd_dir, f'{idx}_ground.pcd'), ground_pts)

        # accumulate map-frame clouds for publishing to perception_3d
        # feature pts are already in map frame (from /cloud_registered)
        self.acc_feature_pts.append(pts[feature_mask])
        self.acc_ground_pts.append(pts[ground_mask] if ground_mask.any()
                                   else pts[:min(10, len(pts))])

        self.get_logger().info(
            f'Keyframe {idx}: pos=({tx:.2f},{ty:.2f},{tz:.2f}) '
            f'feat={len(feature_pts)} ground={len(ground_pts)}')

    def publish_map_cb(self):
        """Publish accumulated map to lego_loam_map and lego_loam_ground for perception_3d."""
        if not self.acc_feature_pts:
            return

        stamp = self.get_clock().now().to_msg()

        def make_pc2(pts_list, frame='map'):
            all_pts = np.vstack(pts_list)
            msg = PointCloud2()
            msg.header.stamp = stamp
            msg.header.frame_id = frame
            msg.height = 1
            msg.width = len(all_pts)
            msg.is_dense = True
            msg.is_bigendian = False
            msg.fields = [
                PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step = msg.point_step * msg.width
            xyzi = np.hstack([all_pts.astype(np.float32),
                              np.zeros((len(all_pts), 1), dtype=np.float32)])
            msg.data = xyzi.tobytes()
            return msg

        self.pub_map.publish(make_pc2(self.acc_feature_pts))
        self.pub_ground.publish(make_pc2(self.acc_ground_pts))

    def save_cb(self, request, response):
        if len(self.keyframes) == 0:
            self.get_logger().warn('No keyframes to save!')
            return response

        poses_data = []
        for i, (x, y, z, roll, pitch, yaw) in enumerate(self.keyframes):
            poses_data.append((x, y, z, roll, pitch, yaw, i))

        save_poses_pcd(os.path.join(self.save_dir, 'poses.pcd'), poses_data)

        # Also save a dummy edges.pcd (sequential edges)
        edges_path = os.path.join(self.save_dir, 'edges.pcd')
        n = len(self.keyframes)
        with open(edges_path, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {max(1, n-1)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {max(1, n-1)}\n")
            f.write("DATA ascii\n")
            for i in range(n - 1):
                f.write(f"{float(i):.1f} {float(i):.1f} {float(i+1):.1f}\n")
            if n <= 1:
                f.write("0.0 0.0 0.0\n")

        self.get_logger().info(
            f'Saved {n} keyframes to {self.save_dir}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FastLioToPoseGraph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
