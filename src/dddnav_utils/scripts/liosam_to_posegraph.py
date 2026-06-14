#!/usr/bin/env python3
# LIO-SAM 建图结果转换为 dddnav 定位格式
#
# 订阅 LIO-SAM 输出，实时收集关键帧，调用 save service 后生成：
#   <save_dir>/
#     poses.pcd            - PointXYZIRPYT 关键帧位姿（map frame）
#     edges.pcd            - 顺序边（占位）
#     pcd/N_feature.pcd    - 特征点云（base_link frame）
#     pcd/N_ground.pcd     - 地面点云（base_link frame）
#
# 保存: ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}

import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Empty


# 几何工具

def quat_to_rpy(x, y, z, w):
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp  = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(sinp)
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr],
    ])


def transform_pts(pts, tx, ty, tz, roll, pitch, yaw):
    """map frame → base_link frame: p_base = R^T * (p_map - t)"""
    R = rpy_to_matrix(roll, pitch, yaw)
    return (R.T @ (pts - np.array([tx, ty, tz])).T).T


# PCD 写入 (binary little-endian)
#
# 关键帧建图一次写几百个文件，每帧 2 个 pcd（feature + ground）。原本 ASCII 实现
# 在 N=2000+ 关键帧时落盘会变成 IO 瓶颈：fprintf 拼字符串 + 文件大约比二进制大 3~4
# 倍，10k+ 点的关键帧每个 30~80ms。改为 binary 后 < 5ms，且文件小、加载快。

def write_pcd_xyzi(path, pts):
    """Binary PCD with FIELDS x y z intensity (intensity 写 0)."""
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    n = len(pts)
    # struct: x(4) y(4) z(4) intensity(4) = 16 bytes/point, contiguous, intensity=0
    payload = np.zeros((n, 4), dtype=np.float32)
    payload[:, :3] = pts
    header = (
        "# .PCD v0.7\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")
    with open(path, 'wb') as f:
        f.write(header)
        f.write(payload.tobytes(order='C'))


def write_poses_pcd(path, poses):
    """poses: list of (x,y,z,roll,pitch,yaw,idx)"""
    n = len(poses)
    with open(path, 'w') as f:
        f.write("# .PCD v0.7\nVERSION 0.7\n")
        f.write("FIELDS x y z intensity roll pitch yaw time\n")
        f.write("SIZE 4 4 4 4 4 4 4 8\nTYPE F F F F F F F F\nCOUNT 1 1 1 1 1 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
        for x, y, z, roll, pitch, yaw, idx in poses:
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {float(idx):.1f} "
                    f"{roll:.6f} {pitch:.6f} {yaw:.6f} 0.0\n")


def write_edges_pcd(path, n):
    edges = max(1, n - 1)
    with open(path, 'w') as f:
        f.write("# .PCD v0.7\nVERSION 0.7\n")
        f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {edges}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {edges}\nDATA ascii\n")
        for i in range(n - 1):
            f.write(f"{float(i):.1f} {float(i):.1f} {float(i+1):.1f}\n")
        if n <= 1:
            f.write("0.0 0.0 0.0\n")


# 主节点

class LioSamToPoseGraph(Node):

    def __init__(self):
        super().__init__('liosam_to_posegraph')

        self.declare_parameter('save_dir',           '/tmp/fastlio_map')
        self.declare_parameter('keyframe_dist',      0.5)   # m
        self.declare_parameter('keyframe_angle',     0.3)   # rad
        self.declare_parameter('ground_angle_thresh', 15.0) # deg

        self.save_dir    = self.get_parameter('save_dir').value
        self.kf_dist     = self.get_parameter('keyframe_dist').value
        self.kf_angle    = self.get_parameter('keyframe_angle').value
        self.gnd_thresh  = math.radians(self.get_parameter('ground_angle_thresh').value)

        os.makedirs(os.path.join(self.save_dir, 'pcd'), exist_ok=True)

        # 关键帧数据
        self.keyframes   = []          # (x,y,z,roll,pitch,yaw) in map frame
        self.kf_clouds   = []          # np array (N,3) in map frame
        self.last_pose   = None

        # 当前帧缓存（等待时间同步）
        self._odom  = None
        self._cloud = None
        self._odom_t  = 0.0
        self._cloud_t = 0.0

        # 实时发布给 perception_3d（建图模式用），使用 transient_local 确保后启动的订阅者也能收到
        self.acc_feat = []
        self.acc_gnd  = []
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map    = self.create_publisher(PointCloud2, 'lego_loam_map',    latched_qos)
        self.pub_ground = self.create_publisher(PointCloud2, 'lego_loam_ground', latched_qos)
        self.create_timer(2.0, self._publish_map)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # LIO-SAM 输出：全局优化里程计 + 每帧原始点云（odom frame）
        self.create_subscription(Odometry,     'lio_sam/mapping/odometry',          self._odom_cb,  qos)
        self.create_subscription(PointCloud2,  'lio_sam/mapping/cloud_registered_raw', self._cloud_cb, qos)

        self.create_service(Empty, 'save_liosam_posegraph', self._save_cb)

        self.get_logger().info(
            f'liosam_to_posegraph ready  save_dir={self.save_dir}  '
            f'kf_dist={self.kf_dist}m  kf_angle={math.degrees(self.kf_angle):.1f}°')
        self.get_logger().info('Save: ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}')

    # 回调

    def _odom_cb(self, msg: Odometry):
        self._odom   = msg
        self._odom_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._try_keyframe()

    def _cloud_cb(self, msg: PointCloud2):
        self._cloud   = msg
        self._cloud_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _try_keyframe(self):
        if self._odom is None or self._cloud is None:
            return
        if abs(self._odom_t - self._cloud_t) > 0.3:
            return

        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        pose = (p.x, p.y, p.z, roll, pitch, yaw)

        # 关键帧筛选
        if self.last_pose is not None:
            lx, ly, lz = self.last_pose[:3]
            lyaw = self.last_pose[5]
            dist = math.sqrt((p.x-lx)**2 + (p.y-ly)**2 + (p.z-lz)**2)
            da   = abs(yaw - lyaw)
            if da > math.pi:
                da = 2*math.pi - da
            if dist < self.kf_dist and da < self.kf_angle:
                return

        self._add_keyframe(pose, self._cloud)

    def _add_keyframe(self, pose, cloud_msg: PointCloud2):
        idx = len(self.keyframes)
        self.keyframes.append(pose)
        self.last_pose = pose

        # 读取点云（map frame），直接用 numpy 解析避免类型转换问题
        try:
            raw = np.frombuffer(bytes(cloud_msg.data), dtype=np.float32)
            raw = raw.reshape(-1, cloud_msg.point_step // 4)  # point_step/4 个 float32 per point
            pts_odom = raw[:, :3].astype(np.float64)  # 取 x y z
        except Exception as e:
            self.get_logger().warn(f'cloud read error: {e}')
            pts_odom = np.zeros((0, 3))

        if len(pts_odom) == 0:
            write_pcd_xyzi(os.path.join(self.save_dir, 'pcd', f'{idx}_feature.pcd'), np.zeros((1,3)))
            write_pcd_xyzi(os.path.join(self.save_dir, 'pcd', f'{idx}_ground.pcd'),  np.zeros((1,3)))
            self.get_logger().warn(f'Keyframe {idx}: empty cloud')
            return

        # cloud_registered_raw 发布时用的是 transformTobeMapped（map frame 位姿）做变换，
        # header.frame_id 虽然写的是 odometryFrame("odom")，但实际坐标已经在 map frame。
        # 与 lio_sam/mapping/odometry 的位姿一致，可以直接用。
        tx, ty, tz, roll, pitch, yaw = pose
        pts_map = pts_odom

        # map frame → base_link frame
        pts_base = transform_pts(pts_map, tx, ty, tz, roll, pitch, yaw)

        # 地面/特征分割（仰角阈值）
        r = np.sqrt(pts_base[:,0]**2 + pts_base[:,1]**2)
        elev = np.arctan2(-pts_base[:,2], r + 1e-6)
        gnd_mask  = elev < -self.gnd_thresh
        feat_mask = ~gnd_mask

        gnd_pts  = pts_base[gnd_mask]  if gnd_mask.any()  else pts_base[:min(10, len(pts_base))]
        feat_pts = pts_base[feat_mask] if feat_mask.any() else pts_base

        write_pcd_xyzi(os.path.join(self.save_dir, 'pcd', f'{idx}_feature.pcd'), feat_pts)
        write_pcd_xyzi(os.path.join(self.save_dir, 'pcd', f'{idx}_ground.pcd'),  gnd_pts)

        # 累积 map frame 点云供实时发布
        self.acc_feat.append(pts_map[feat_mask] if feat_mask.any() else pts_map)
        self.acc_gnd.append( pts_map[gnd_mask]  if gnd_mask.any()  else pts_map[:min(10,len(pts_map))])

        self.get_logger().info(
            f'KF {idx}: ({tx:.2f},{ty:.2f},{tz:.2f})  feat={len(feat_pts)}  gnd={len(gnd_pts)}')

    # 实时地图发布（供 perception_3d mapping_mode 使用）

    def _publish_map(self):
        if not self.acc_feat:
            return
        stamp = self.get_clock().now().to_msg()

        def _make(pts_list):
            all_pts = np.vstack(pts_list).astype(np.float32)
            n = len(all_pts)
            msg = PointCloud2()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'map'
            msg.height = 1
            msg.width  = n
            msg.is_dense = True
            msg.is_bigendian = False
            msg.fields = [
                PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step   = 16 * n
            xyzi = np.hstack([all_pts, np.zeros((n,1), dtype=np.float32)])
            msg.data = xyzi.tobytes()
            return msg

        self.pub_map.publish(_make(self.acc_feat))
        self.pub_ground.publish(_make(self.acc_gnd))

    # 保存

    def _save_cb(self, req, res):
        n = len(self.keyframes)
        if n == 0:
            self.get_logger().warn('No keyframes yet!')
            return res

        poses_data = [(x,y,z,r,p,yaw,i)
                      for i,(x,y,z,r,p,yaw) in enumerate(self.keyframes)]
        write_poses_pcd(os.path.join(self.save_dir, 'poses.pcd'), poses_data)
        write_edges_pcd(os.path.join(self.save_dir, 'edges.pcd'), n)

        self.get_logger().info(f'Saved {n} keyframes → {self.save_dir}')
        return res


def main():
    rclpy.init()
    rclpy.spin(LioSamToPoseGraph())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
