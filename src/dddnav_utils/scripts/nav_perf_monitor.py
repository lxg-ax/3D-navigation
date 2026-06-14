#!/usr/bin/env python3
"""Runtime performance monitor for the dddnav navigation stack.

Tracks message rate + payload age on a handful of high-impact topics and
publishes everything as ``diagnostic_msgs/DiagnosticArray`` on
``/diagnostics``. Foxglove / Plotjuggler / RViz Diagnostic panels can chart
the stream directly, so we get visibility into:

* SLAM front-end:  ``/Odometry`` (FAST-LIO 100Hz), ``/odom_filtered``
  (ESKF 100Hz). Falling rate or rising latency = front-end struggling.
* Planning:  ``/global_planner_plan`` (or whichever topic the global planner
  publishes the latest plan on), tracked for *gaps* — long gaps between
  plan refreshes mean the planner is failing repeatedly.
* Control:  ``cmd_vel`` from p2p_move_base. Staleness here = robot will
  coast on whatever the last command was.
* Cost map:  ``/perception_3d_local/markings`` (if published) — drop-outs
  imply the costmap pipeline stalled.
* SLAM front-end health:  ``/fast_lio/health`` residual / feature count, so
  we can log when the LiDAR front-end is under stress without subscribing
  to the high-volume point cloud.

The node is read-only; it never restarts upstream. Pair it with
``slam_health_monitor.py`` (TF / odom liveness) for a complete picture.
"""

import math  # noqa: F401
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray


_OK = DiagnosticStatus.OK
_WARN = DiagnosticStatus.WARN
_ERROR = DiagnosticStatus.ERROR


class TopicProbe:
    """Lightweight rolling-window rate + age tracker.

    Stores receive timestamps in a deque so rate is computed over the last N
    samples instead of since startup. The age is reported as the gap between
    *now* and the most-recent stamp — both message header stamp (when the
    message carries one) and rclpy receive time, since the two answer
    different questions.
    """

    def __init__(self, ok_rate, warn_rate, err_rate, window=30):
        self.ok_rate = ok_rate
        self.warn_rate = warn_rate
        self.err_rate = err_rate
        self.times = deque(maxlen=window)
        self.last_header_age = None
        self.last_recv = None

    def stamp(self, recv_t, header_t=None):
        self.times.append(recv_t)
        self.last_recv = recv_t
        if header_t is not None:
            self.last_header_age = max(0.0, recv_t - header_t)

    def rate(self):
        if len(self.times) < 2:
            return 0.0
        return (len(self.times) - 1) / max(1e-6, self.times[-1] - self.times[0])

    def age(self, now):
        if self.last_recv is None:
            return None
        return now - self.last_recv

    def level(self, now):
        if self.last_recv is None:
            return _ERROR, 'never received'
        rate = self.rate()
        age = now - self.last_recv
        if rate < self.err_rate or age > 5.0:
            return _ERROR, f'rate={rate:.2f}Hz age={age:.1f}s'
        if rate < self.warn_rate or age > 2.0:
            return _WARN, f'rate={rate:.2f}Hz age={age:.1f}s'
        return _OK, f'rate={rate:.2f}Hz age={age:.1f}s'


class NavPerfMonitor(Node):
    def __init__(self):
        super().__init__('nav_perf_monitor')

        self.declare_parameter('check_period', 1.0)
        self.declare_parameter('lio_residual_warn', 0.10)   # m
        self.declare_parameter('lio_residual_error', 0.30)
        self.declare_parameter('lio_min_feats_warn', 80)

        period = self.get_parameter('check_period').value
        self.lio_resid_warn = float(self.get_parameter('lio_residual_warn').value)
        self.lio_resid_err  = float(self.get_parameter('lio_residual_error').value)
        self.lio_feat_warn  = int(self.get_parameter('lio_min_feats_warn').value)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=20)
        latched_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST, depth=2)

        # Probes — thresholds chosen for typical Mid360 / FAST-LIO 100Hz +
        # planner 5Hz + control 10Hz operation.
        self.probes = {
            '/Odometry':                TopicProbe(50.0, 30.0, 10.0),
            '/odom_filtered':           TopicProbe(50.0, 30.0, 10.0),
            '/cmd_vel':                 TopicProbe(5.0, 2.0, 0.5),
            '/cmd_vel_stamped':         TopicProbe(5.0, 2.0, 0.5),
            '/global_planner/path':     TopicProbe(0.5, 0.1, 0.0),
            '/perception_3d/markings':  TopicProbe(2.0, 0.5, 0.0),
            '/fast_lio/health':         TopicProbe(50.0, 20.0, 5.0),
        }

        # Subscriptions. We don't actually inspect message content for most
        # topics — only header stamps and arrival rate. The exception is
        # /fast_lio/health where the payload itself drives a status.
        self.create_subscription(
            Odometry, '/Odometry',
            lambda m: self._on_header('/Odometry', m), sensor_qos)
        self.create_subscription(
            Odometry, '/odom_filtered',
            lambda m: self._on_header('/odom_filtered', m), sensor_qos)
        self.create_subscription(
            Twist, '/cmd_vel',
            lambda m: self._on_simple('/cmd_vel'), sensor_qos)
        self.create_subscription(
            TwistStamped, '/cmd_vel_stamped',
            lambda m: self._on_header('/cmd_vel_stamped', m), sensor_qos)
        self.create_subscription(
            Path, '/global_planner/path',
            lambda m: self._on_header('/global_planner/path', m), latched_qos)

        self._lio_residual = None
        self._lio_feats = None
        self.create_subscription(
            Float64MultiArray, '/fast_lio/health',
            self._on_lio_health, sensor_qos)

        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'nav_perf_monitor up; probing {len(self.probes)} topics')

    # ------------------------------------------------------------------- cb
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_header(self, key, msg):
        recv = self._now()
        h = msg.header.stamp
        header_t = h.sec + h.nanosec * 1e-9 if h.sec or h.nanosec else None
        self.probes[key].stamp(recv, header_t)

    def _on_simple(self, key):
        self.probes[key].stamp(self._now())

    def _on_lio_health(self, msg: Float64MultiArray):
        self.probes['/fast_lio/health'].stamp(self._now())
        if msg.data:
            self._lio_residual = msg.data[0]
            if len(msg.data) >= 2:
                self._lio_feats = int(msg.data[1])

    # ----------------------------------------------------------------- tick
    def _tick(self):
        now = self._now()
        statuses = []

        for name, probe in self.probes.items():
            level, summary = probe.level(now)
            extras = {'rate_hz': f'{probe.rate():.2f}'}
            age = probe.age(now)
            if age is not None:
                extras['age_s'] = f'{age:.3f}'
            if probe.last_header_age is not None:
                extras['header_lag_s'] = f'{probe.last_header_age:.3f}'
            statuses.append(self._mk(level, f'topic {name}', summary, extras))

        # Front-end residual / feature count — independent of arrival rate
        # because we want to flag "data is coming but it is bad".
        if self._lio_residual is not None:
            r = self._lio_residual
            f = self._lio_feats if self._lio_feats is not None else -1
            level = _OK
            text = f'res={r:.3f}m feats={f}'
            if r >= self.lio_resid_err or (0 <= f < self.lio_feat_warn // 2):
                level = _ERROR
            elif r >= self.lio_resid_warn or (0 <= f < self.lio_feat_warn):
                level = _WARN
            statuses.append(self._mk(
                level, 'fast_lio residual', text,
                {'residual_m': f'{r:.4f}', 'feats': str(f)}))

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = statuses
        self._diag_pub.publish(arr)

    @staticmethod
    def _mk(level, name, message, extras=None):
        s = DiagnosticStatus()
        s.level = level
        s.name = f'nav_perf: {name}'
        s.hardware_id = 'dddnav'
        s.message = message
        if extras:
            s.values = [KeyValue(key=str(k), value=str(v))
                        for k, v in extras.items()]
        return s


def main():
    rclpy.init()
    rclpy.spin(NavPerfMonitor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
