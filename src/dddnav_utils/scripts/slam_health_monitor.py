#!/usr/bin/env python3
"""SLAM health watchdog.

Tracks four signal classes:

* **Topic liveness** — `/Odometry`, `lio_sam/mapping/odometry` (configurable).
* **TF edge liveness** — `map→odom`, `odom→base_link` etc., compared against
  the *stamp* on the latest transform (not just the lookup time, so a stale
  republisher gets caught too).
* **TF jump detection** — flags large discrete jumps on `map→odom`. Helpful
  when a loop closure or a relocalisation event slams the chain, which can
  surprise downstream consumers.
* **Filter covariance** — optional, monitors the trace of `pose.covariance`
  on `/odom_filtered` so we notice when ESKF starts diverging.

Outputs:
* logs at OK (≤ ok_timeout) → WARN (≤ fail_timeout) → ERROR (> fail_timeout);
* a `diagnostic_msgs/DiagnosticArray` on `/diagnostics` so RViz Diagnostic
  panels and external monitors (foxglove etc.) can show colour-coded status.

This node is read-only — it never restarts upstream nodes.
"""

import math
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


_OK = DiagnosticStatus.OK
_WARN = DiagnosticStatus.WARN
_ERROR = DiagnosticStatus.ERROR


class SlamHealth(Node):
    def __init__(self):
        super().__init__('slam_health_monitor')

        # ---------- params ------------------------------------------------
        self.declare_parameter('ok_timeout',   0.5)
        self.declare_parameter('fail_timeout', 2.0)
        self.declare_parameter('check_period', 1.0)
        self.declare_parameter('fastlio_topic', '/Odometry')
        self.declare_parameter('liosam_topic',  'lio_sam/mapping/odometry')
        self.declare_parameter('filtered_odom_topic', '/odom_filtered')
        self.declare_parameter('tf_edges',
                               ['map->odom', 'odom->base_link'])
        # map->odom jump alarm — anything above is reported.
        self.declare_parameter('tf_jump_dist', 1.0)    # metres
        self.declare_parameter('tf_jump_angle', 0.5)   # radians
        # Pose covariance trace warn / error thresholds (m²+rad²).
        self.declare_parameter('cov_trace_warn',  0.5)
        self.declare_parameter('cov_trace_error', 5.0)
        # FAST-LIO degeneracy thresholds (Hessian min eigval / cond number).
        # Min eigval is in (correspondences²), so it scales with feature count;
        # leave at 0 to disable. Tuned to flag only severe degeneracy
        # (the SC watchdog handles recovery — this is for visibility).
        self.declare_parameter('lio_health_topic',         '/fast_lio/health')
        self.declare_parameter('lio_min_eigval_warn',      50.0)
        self.declare_parameter('lio_min_eigval_error',     5.0)
        self.declare_parameter('lio_cond_number_warn',     1.0e4)
        self.declare_parameter('lio_cond_number_error',    1.0e6)
        self.declare_parameter('lio_feats_warn',           80)
        self.declare_parameter('lio_feats_error',          30)
        self.declare_parameter('publish_diagnostics', True)

        self.ok_to        = self.get_parameter('ok_timeout').value
        self.fail_to      = self.get_parameter('fail_timeout').value
        period            = self.get_parameter('check_period').value
        self.jump_dist    = float(self.get_parameter('tf_jump_dist').value)
        self.jump_angle   = float(self.get_parameter('tf_jump_angle').value)
        self.cov_warn     = float(self.get_parameter('cov_trace_warn').value)
        self.cov_err      = float(self.get_parameter('cov_trace_error').value)
        self.lio_health_topic = self.get_parameter('lio_health_topic').value
        self.lio_min_eig_warn = float(self.get_parameter('lio_min_eigval_warn').value)
        self.lio_min_eig_err  = float(self.get_parameter('lio_min_eigval_error').value)
        self.lio_cond_warn    = float(self.get_parameter('lio_cond_number_warn').value)
        self.lio_cond_err     = float(self.get_parameter('lio_cond_number_error').value)
        self.lio_feats_warn   = int(self.get_parameter('lio_feats_warn').value)
        self.lio_feats_err    = int(self.get_parameter('lio_feats_error').value)
        self.pub_diag     = bool(self.get_parameter('publish_diagnostics').value)

        edges_str = self.get_parameter('tf_edges').value
        self.tf_edges = []
        for s in edges_str:
            if '->' in s:
                p, c = s.split('->', 1)
                self.tf_edges.append((p.strip(), c.strip()))

        # ---------- TF listener ------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- topic liveness ---------------------------------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self._last_topic = OrderedDict()
        for topic in (self.get_parameter('fastlio_topic').value,
                      self.get_parameter('liosam_topic').value):
            self._last_topic[topic] = None
            self.create_subscription(
                Odometry, topic,
                lambda msg, t=topic: self._on_topic_msg(t), qos)

        # ---------- filtered odom covariance -----------------------------
        self._cov_trace = None
        filtered_topic = self.get_parameter('filtered_odom_topic').value
        if filtered_topic:
            self.create_subscription(
                Odometry, filtered_topic, self._on_filtered, qos)
            self._filtered_topic = filtered_topic
        else:
            self._filtered_topic = None

        # ---------- FAST-LIO degeneracy health ---------------------------
        # Latest [residual, feats, min_eigval, cond_number]; publisher uses
        # default reliable QoS so we match it.
        self._lio_health = None
        self._lio_health_t = None
        if self.lio_health_topic:
            self.create_subscription(
                Float64MultiArray, self.lio_health_topic,
                self._on_lio_health, 10)

        # ---------- map->odom jump tracking -------------------------------
        self._last_map_odom = None  # (t_sec, x, y, z, qw, qx, qy, qz)

        # ---------- diagnostics publisher --------------------------------
        if self.pub_diag:
            self._diag_pub = self.create_publisher(
                DiagnosticArray, '/diagnostics', 10)
        else:
            self._diag_pub = None

        self.create_timer(period, self._check)
        self.get_logger().info(
            f'health: ok<={self.ok_to}s fail>{self.fail_to}s  '
            f'tf_edges={self.tf_edges}  jump=({self.jump_dist}m,{self.jump_angle}rad)')

    # ---------------- subscriber callbacks ---------------------------------

    def _on_topic_msg(self, topic):
        self._last_topic[topic] = self._now()

    def _on_filtered(self, msg: Odometry):
        # Sum of the diagonal — a single scalar is a coarse but useful "is the
        # filter spreading out?" indicator.
        cov = msg.pose.covariance
        if len(cov) == 36:
            tr = sum(cov[i * 6 + i] for i in range(6))
            if math.isfinite(tr):
                self._cov_trace = tr

    def _on_lio_health(self, msg: Float64MultiArray):
        # data layout (FAST-LIO laserMapping.cpp publish_odometry):
        #   [residual_m, effct_feats, hessian_min_eigval, cond_number]
        # Pre-Hessian builds only ship the first two — we tolerate that and
        # leave the eigval/cond fields as None so checks skip silently.
        if len(msg.data) < 2:
            return
        d = list(msg.data)
        self._lio_health = (
            float(d[0]),
            float(d[1]),
            float(d[2]) if len(d) > 2 else None,
            float(d[3]) if len(d) > 3 else None,
        )
        self._lio_health_t = self._now()

    # ---------------- helpers ----------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _level(dt, ok, fail):
        if dt is None:
            return _WARN
        if dt > fail:
            return _ERROR
        if dt > ok:
            return _WARN
        return _OK

    @staticmethod
    def _quat_angle(q1, q2):
        # Angle between two quaternions: 2 * acos(|<q1, q2>|).
        dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
        dot = max(-1.0, min(1.0, dot))
        return 2.0 * math.acos(dot)

    def _check(self):
        now = self._now()
        statuses = []

        # --- topic liveness ---
        for topic, last in self._last_topic.items():
            dt = None if last is None else now - last
            level = self._level(dt, self.ok_to, self.fail_to)
            msg = ('no message yet' if dt is None
                   else f'last={dt:.1f}s')
            self._log_at(level, f'[topic {topic}] {msg}')
            statuses.append(self._make_status(
                level, f'topic {topic}', msg, {'age_s': f'{dt or -1:.3f}'}))

        # --- tf edge liveness + map->odom jump ---
        for parent, child in self.tf_edges:
            try:
                tf = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
            except Exception as e:                         # noqa: BLE001
                self.get_logger().warn(
                    f'[tf {parent}->{child}] not available: {e}')
                statuses.append(self._make_status(
                    _WARN, f'tf {parent}->{child}', 'not available'))
                continue

            t_msg = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            dt = now - t_msg
            level = self._level(dt, self.ok_to, self.fail_to)
            self._log_at(level, f'[tf {parent}->{child}] stamp_age={dt:.1f}s')

            extras = {'stamp_age_s': f'{dt:.3f}'}

            # Jump detection only on map->odom (the high-impact edge).
            if parent == 'map' and child == 'odom':
                tr = tf.transform.translation
                rot = tf.transform.rotation
                cur = (t_msg, tr.x, tr.y, tr.z, rot.w, rot.x, rot.y, rot.z)
                if self._last_map_odom is not None:
                    prev = self._last_map_odom
                    dx = cur[1] - prev[1]
                    dy = cur[2] - prev[2]
                    dz = cur[3] - prev[3]
                    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                    ang = self._quat_angle(prev[4:], cur[4:])
                    extras['jump_dist_m'] = f'{dist:.3f}'
                    extras['jump_angle_rad'] = f'{ang:.3f}'
                    if dist > self.jump_dist or ang > self.jump_angle:
                        level = max(level, _WARN)
                        self.get_logger().warn(
                            f'[tf map->odom] JUMP dist={dist:.2f}m '
                            f'angle={ang:.2f}rad — likely loop closure')
                self._last_map_odom = cur

            statuses.append(self._make_status(
                level, f'tf {parent}->{child}',
                f'stamp_age={dt:.2f}s', extras))

        # --- filter covariance ---
        if self._filtered_topic is not None:
            tr = self._cov_trace
            if tr is None:
                level, summary = _WARN, 'no covariance yet'
            elif tr > self.cov_err:
                level, summary = _ERROR, f'cov_trace={tr:.2f}'
            elif tr > self.cov_warn:
                level, summary = _WARN, f'cov_trace={tr:.2f}'
            else:
                level, summary = _OK, f'cov_trace={tr:.2f}'
            self._log_at(level, f'[{self._filtered_topic}] {summary}')
            statuses.append(self._make_status(
                level, f'cov {self._filtered_topic}', summary,
                {'cov_trace': '' if tr is None else f'{tr:.4f}'}))

        # --- FAST-LIO degeneracy ---
        # Drops below lio_min_eigval_error → ERROR (long corridor / planar wall
        # eating the geometry). Cond number is informative — high values warn
        # but don't error on their own (handled jointly with min_eig).
        if self.lio_health_topic and self._lio_health is not None:
            residual, feats, min_eig, cond = self._lio_health
            stale = (now - self._lio_health_t) if self._lio_health_t else None

            if stale is not None and stale > self.fail_to:
                level = _ERROR
                summary = f'health stale={stale:.1f}s'
            elif feats < self.lio_feats_err:
                level = _ERROR
                summary = f'feats={int(feats)} (<{self.lio_feats_err})'
            elif min_eig is not None and min_eig < self.lio_min_eig_err:
                level = _ERROR
                summary = f'min_eig={min_eig:.2f} (<{self.lio_min_eig_err})'
            elif feats < self.lio_feats_warn:
                level = _WARN
                summary = f'feats={int(feats)} (<{self.lio_feats_warn})'
            elif min_eig is not None and min_eig < self.lio_min_eig_warn:
                level = _WARN
                summary = f'min_eig={min_eig:.2f} (<{self.lio_min_eig_warn})'
            elif cond is not None and cond > self.lio_cond_warn:
                level = _WARN
                summary = f'cond={cond:.0f}'
            else:
                level = _OK
                summary = (f'res={residual:.3f} feats={int(feats)}'
                           + (f' min_eig={min_eig:.2f}' if min_eig is not None else '')
                           + (f' cond={cond:.0f}' if cond is not None else ''))
            self._log_at(level, f'[fast_lio degeneracy] {summary}')
            statuses.append(self._make_status(
                level, 'fast_lio degeneracy', summary,
                {'residual_m':   f'{residual:.4f}',
                 'effct_feats':  f'{int(feats)}',
                 'min_eigval':   '' if min_eig is None else f'{min_eig:.4f}',
                 'cond_number':  '' if cond is None else f'{cond:.2f}'}))

        # --- publish diagnostics ---
        if self._diag_pub is not None:
            arr = DiagnosticArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.status = statuses
            self._diag_pub.publish(arr)

    def _log_at(self, level, text):
        if level == _ERROR:
            self.get_logger().error(text)
        elif level == _WARN:
            self.get_logger().warn(text)
        # OK: stay quiet, the diagnostics topic is enough.

    @staticmethod
    def _make_status(level, name, message, extras=None):
        s = DiagnosticStatus()
        s.level = level
        s.name = f'slam_health: {name}'
        s.hardware_id = 'dddnav'
        s.message = message
        if extras:
            s.values = [KeyValue(key=str(k), value=str(v))
                        for k, v in extras.items()]
        return s


def main():
    rclpy.init()
    rclpy.spin(SlamHealth())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
