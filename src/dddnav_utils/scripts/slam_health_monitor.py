#!/usr/bin/env python3
"""SLAM health watchdog — odom topic liveness + TF edge liveness.

监督两类信号:
  1. odometry topic liveness: FAST-LIO / LIO-SAM 是否还在出 /Odometry
  2. TF edge liveness: map->odom, odom->base_link 是否最近还在更新

OK / WARN / FAIL 三档, 只发日志不重启上游.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import tf2_ros


class SlamHealth(Node):
    def __init__(self):
        super().__init__('slam_health_monitor')

        self.declare_parameter('ok_timeout',   0.5)
        self.declare_parameter('fail_timeout', 2.0)
        self.declare_parameter('check_period', 1.0)
        self.declare_parameter('fastlio_topic', '/Odometry')
        self.declare_parameter('liosam_topic',  'lio_sam/mapping/odometry')
        self.declare_parameter('tf_edges',
                               ['map->odom', 'odom->base_link'])

        self.ok_to   = self.get_parameter('ok_timeout').value
        self.fail_to = self.get_parameter('fail_timeout').value
        period       = self.get_parameter('check_period').value
        edges_str    = self.get_parameter('tf_edges').value

        # parse "parent->child"
        self.tf_edges = []
        for s in edges_str:
            if '->' in s:
                p, c = s.split('->', 1)
                self.tf_edges.append((p.strip(), c.strip()))

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self._last_topic = {}
        for topic in (self.get_parameter('fastlio_topic').value,
                      self.get_parameter('liosam_topic').value):
            self._last_topic[topic] = None
            self.create_subscription(
                Odometry, topic,
                lambda msg, t=topic: self._on_msg(t, msg), qos)

        self.create_timer(period, self._check)
        self.get_logger().info(
            f'health: ok<={self.ok_to}s fail>{self.fail_to}s  '
            f'tf_edges={self.tf_edges}')

    def _on_msg(self, topic, _msg):
        self._last_topic[topic] = self.get_clock().now().nanoseconds * 1e-9

    def _check(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        for topic, last in self._last_topic.items():
            if last is None:
                self.get_logger().warn(f'[topic {topic}] no message yet')
                continue
            dt = now - last
            if dt > self.fail_to:
                self.get_logger().error(f'[topic {topic}] FAIL last={dt:.1f}s')
            elif dt > self.ok_to:
                self.get_logger().warn(f'[topic {topic}] WARN last={dt:.1f}s')

        # TF age check: ask the buffer for the most recent stamp on each edge.
        for parent, child in self.tf_edges:
            try:
                tf = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
                t_msg = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                dt = now - t_msg
                if dt > self.fail_to:
                    self.get_logger().error(
                        f'[tf {parent}->{child}] FAIL stamp_age={dt:.1f}s')
                elif dt > self.ok_to:
                    self.get_logger().warn(
                        f'[tf {parent}->{child}] WARN stamp_age={dt:.1f}s')
            except Exception as e:
                self.get_logger().warn(
                    f'[tf {parent}->{child}] not available: {e}')


def main():
    rclpy.init()
    rclpy.spin(SlamHealth())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
