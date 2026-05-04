#!/usr/bin/env python3
# livox PointCloud2 (xfer_format=0) 字段适配
# 输出:
#   /livox/lidar_liosam       - LIO-SAM VelodynePointXYZIRT 格式 (32字节, PCL EIGEN_ALIGN16对齐)，供 LIO-SAM 建图用
#   /livox/lidar_liosam_xyzi  - PointXYZI 兼容格式 (16字节)，供 dddnav mcl_ip 定位用

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

# livox 原始格式 (xfer_format=0)
LIVOX_DTYPE = np.dtype([
    ('x', np.float32), ('y', np.float32), ('z', np.float32),
    ('intensity', np.float32), ('tag', np.uint8), ('line', np.uint8),
    ('timestamp', np.float64),  # 纳秒（double 类型存储），设备时间
])

# LIO-SAM VelodynePointXYZIRT 格式 (32字节，与 PCL EIGEN_ALIGN16 对齐一致)
# PCL_ADD_POINT4D = x(4)+y(4)+z(4)+pad(4)=16, PCL_ADD_INTENSITY=4, ring(2), time(4), pad(6) = 32
LIOSAM_DTYPE = np.dtype({
    'names':   ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': [np.float32, np.float32, np.float32, np.float32, np.uint16, np.float32],
    'offsets': [0, 4, 8, 16, 20, 22],
    'itemsize': 32,
})

# PointXYZI 兼容格式 (16字节): 供 dddnav mcl_ip 使用
XYZI_DTYPE = np.dtype([
    ('x', np.float32), ('y', np.float32), ('z', np.float32),
    ('intensity', np.float32),
])


class LivoxAdapter(Node):
    def __init__(self):
        super().__init__('livox_pc2_to_liosam')
        self.declare_parameter('input_topic',          '/livox/lidar')
        self.declare_parameter('liosam_output_topic',  '/livox/lidar_liosam')
        self.declare_parameter('xyzi_output_topic',    '/livox/lidar_liosam_xyzi')

        in_topic      = self.get_parameter('input_topic').value
        liosam_topic  = self.get_parameter('liosam_output_topic').value
        xyzi_topic    = self.get_parameter('xyzi_output_topic').value

        self.sub         = self.create_subscription(PointCloud2, in_topic, self.callback, 10)
        self.pub_liosam  = self.create_publisher(PointCloud2, liosam_topic,  10)
        self.pub_xyzi    = self.create_publisher(PointCloud2, xyzi_topic,    10)
        self.get_logger().info(f'{in_topic} → {liosam_topic} + {xyzi_topic}')

    def callback(self, msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            return

        raw = np.frombuffer(bytes(msg.data), dtype=LIVOX_DTYPE, count=n)
        t0  = float(raw['timestamp'][0])

        # LIO-SAM 输出 (VelodynePointXYZIRT 格式, 32字节, 含 ring/time，供 LIO-SAM 建图用)
        ls = np.zeros(n, dtype=LIOSAM_DTYPE)  # zeros 确保 padding 区域为0
        ls['x']         = raw['x']
        ls['y']         = raw['y']
        ls['z']         = raw['z']
        ls['intensity'] = raw['intensity']
        ls['ring']      = raw['line'].astype(np.uint16)
        ls['time']      = ((raw['timestamp'] - t0) * 1e-9).astype(np.float32)  # ns → s

        out_ls = PointCloud2()
        out_ls.header    = msg.header
        out_ls.height    = 1
        out_ls.width     = n
        out_ls.is_bigendian = False
        out_ls.is_dense  = True
        out_ls.point_step = LIOSAM_DTYPE.itemsize  # 32 bytes，与 PCL VelodynePointXYZIRT 一致
        out_ls.row_step   = n * LIOSAM_DTYPE.itemsize
        out_ls.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
            PointField(name='time',      offset=22, datatype=PointField.FLOAT32, count=1),
        ]
        out_ls.data = ls.tobytes()
        self.pub_liosam.publish(out_ls)

        # PointXYZI 输出 (16字节，供 dddnav mcl_ip 定位用)
        xi = np.empty(n, dtype=XYZI_DTYPE)
        xi['x']         = raw['x']
        xi['y']         = raw['y']
        xi['z']         = raw['z']
        xi['intensity'] = raw['intensity']

        out_xi = PointCloud2()
        out_xi.header    = msg.header
        out_xi.height    = 1
        out_xi.width     = n
        out_xi.is_bigendian = False
        out_xi.is_dense  = True
        out_xi.point_step = XYZI_DTYPE.itemsize  # 16 bytes
        out_xi.row_step   = n * XYZI_DTYPE.itemsize
        out_xi.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out_xi.data = xi.tobytes()
        self.pub_xyzi.publish(out_xi)


def main():
    rclpy.init()
    rclpy.spin(LivoxAdapter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
