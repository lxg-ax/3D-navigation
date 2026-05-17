#!/usr/bin/env python3
"""SLAM health monitor — passive watchdog.

订阅 FAST-LIO `/Odometry` 与 LIO-SAM `lio_sam/mapping/odometry`, 记录每个话题
最近一次到达时间, 周期性检查超时. 三种状态:

  OK       last_msg_dt ≤ ok_timeout
  WARN     ok_timeout < last_msg_dt ≤ fail_timeout
  FAIL     last_msg_dt > fail_timeout 或从未收到过

只发警告日志, 不试图重启上游节点. 想把状态接出来做联动 (RViz / 运动控制
fail-safe) 可以扩展成 publish std_msgs/String, 这里保持最小依赖.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class SlamHealth(Node):
    def __init__(self):
        super().__init__('slam_health_monitor')

        self.declare_parameter('ok_timeout',   0.5)   # s, FAST-LIO 100Hz, LIO-SAM ~5Hz
        self.declare_parameter('fail_timeout', 2.0)
        self.declare_parameter('check_period', 1.0)
        self.declare_parameter('fastlio_topic', '/Odometry')
        self.declare_parameter('liosam_topic',  'lio_sam/mapping/odometry')

        self.ok_to   = self.get_parameter('ok_timeout').value
        self.fail_to = self.get_parameter('fail_timeout').value
        period       = self.get_parameter('check_period').value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self._last = {}  # topic -> ros time (sec)

        for topic in (self.get_parameter('fastlio_topic').value,
                      self.get_parameter('liosam_topic').value):
            self._last[topic] = None
            self.create_subscription(
                Odometry, topic,
                lambda msg, t=topic: self._on_msg(t, msg), qos)

        self.create_timer(period, self._check)
        self.get_logger().info(
            f'SLAM health monitor: ok≤{self.ok_to:.1f}s fail>{self.fail_to:.1f}s')

    def _on_msg(self, topic, msg):
        # use wall clock instead of msg stamp; we are watching publish liveness,
        # not timestamp drift.
        self._last[topic] = self.get_clock().now().nanoseconds * 1e-9

    def _check(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for topic, last in self._last.items():
            if last is None:
                self.get_logger().warn(f'[{topic}] no message received yet')
                continue
            dt = now - last
            if dt > self.fail_to:
                self.get_logger().error(
                    f'[{topic}] FAIL  last msg {dt:.1f}s ago')
            elif dt > self.ok_to:
                self.get_logger().warn(
                    f'[{topic}] WARN  last msg {dt:.1f}s ago')


def main():
    rclpy.init()
    rclpy.spin(SlamHealth())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
