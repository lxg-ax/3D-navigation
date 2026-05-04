#!/usr/bin/env python3
"""
在当前位置用实时点云生成测试地图，供定位模式测试。
用法: ros2 run dddnav_bringup generate_test_map.py --ros-args -p save_dir:=/path/to/map
需要先启动 livox 驱动 + livox_pc2_to_liosam 适配器。
"""
import os, sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

XYZI_DTYPE = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])

def write_pcd_xyzi(path, pts):
    n = len(pts)
    with open(path, 'w') as f:
        f.write("# .PCD v0.7\nVERSION 0.7\n")
        f.write("FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f}\n")

class MapGenerator(Node):
    def __init__(self):
        super().__init__('generate_test_map')
        self.declare_parameter('save_dir', '')
        self.declare_parameter('num_frames', 10)
        self.declare_parameter('num_keyframes', 5)
        self.save_dir = self.get_parameter('save_dir').value
        self.num_frames = self.get_parameter('num_frames').value
        self.num_kf = self.get_parameter('num_keyframes').value

        if not self.save_dir:
            self.get_logger().error('save_dir not set!')
            sys.exit(1)

        os.makedirs(os.path.join(self.save_dir, 'pcd'), exist_ok=True)
        self.frames = []
        self.sub = self.create_subscription(PointCloud2, '/livox/lidar_liosam_xyzi', self.cb, 10)
        self.get_logger().info(f'Collecting {self.num_frames} frames for {self.num_kf} keyframes...')

    def cb(self, msg):
        n = msg.width * msg.height
        if n == 0:
            return
        raw = np.frombuffer(bytes(msg.data), dtype=XYZI_DTYPE, count=n)
        pts = np.column_stack([raw['x'], raw['y'], raw['z'], raw['intensity']])
        # 过滤 NaN 和太近的点
        valid = np.isfinite(pts[:, 0]) & (np.linalg.norm(pts[:, :3], axis=1) > 0.3)
        self.frames.append(pts[valid])
        self.get_logger().info(f'Frame {len(self.frames)}/{self.num_frames}: {valid.sum()} pts')

        if len(self.frames) >= self.num_frames:
            self.generate()
            rclpy.shutdown()

    def generate(self):
        all_pts = np.vstack(self.frames)
        # 按高度分割: z < -0.3 为地面, 其余为特征
        gnd_thresh = np.percentile(all_pts[:, 2], 20)  # 最低20%当地面
        gnd_mask = all_pts[:, 2] < gnd_thresh
        feat_pts = all_pts[~gnd_mask]
        gnd_pts = all_pts[gnd_mask]

        # 分成 num_kf 个关键帧 (都在原点, 模拟静止)
        feat_per_kf = len(feat_pts) // self.num_kf
        gnd_per_kf = len(gnd_pts) // self.num_kf

        poses = []
        for i in range(self.num_kf):
            f_start = i * feat_per_kf
            f_end = f_start + feat_per_kf if i < self.num_kf - 1 else len(feat_pts)
            g_start = i * gnd_per_kf
            g_end = g_start + gnd_per_kf if i < self.num_kf - 1 else len(gnd_pts)

            write_pcd_xyzi(os.path.join(self.save_dir, 'pcd', f'{i}_feature.pcd'), feat_pts[f_start:f_end])
            write_pcd_xyzi(os.path.join(self.save_dir, 'pcd', f'{i}_ground.pcd'), gnd_pts[g_start:g_end])
            poses.append((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i))

        # poses.pcd
        n = len(poses)
        with open(os.path.join(self.save_dir, 'poses.pcd'), 'w') as f:
            f.write("# .PCD v0.7\nVERSION 0.7\n")
            f.write("FIELDS x y z intensity roll pitch yaw time\n")
            f.write("SIZE 4 4 4 4 4 4 4 8\nTYPE F F F F F F F F\nCOUNT 1 1 1 1 1 1 1 1\n")
            f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
            for x, y, z, roll, pitch, yaw, idx in poses:
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {float(idx):.1f} {roll:.6f} {pitch:.6f} {yaw:.6f} 0.0\n")

        # edges.pcd
        edges = max(1, n - 1)
        with open(os.path.join(self.save_dir, 'edges.pcd'), 'w') as f:
            f.write("# .PCD v0.7\nVERSION 0.7\n")
            f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {edges}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {edges}\nDATA ascii\n")
            for i in range(n - 1):
                f.write(f"{float(i):.1f} {float(i):.1f} {float(i+1):.1f}\n")

        self.get_logger().info(f'Map saved to {self.save_dir}')
        self.get_logger().info(f'  {self.num_kf} keyframes, {len(feat_pts)} feature pts, {len(gnd_pts)} ground pts')
        self.get_logger().info(f'  Now restart localization.launch.py to test')

def main():
    rclpy.init()
    rclpy.spin(MapGenerator())

if __name__ == '__main__':
    main()
