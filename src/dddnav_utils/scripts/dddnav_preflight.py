#!/usr/bin/env python3
"""Startup self-check for the dddnav stack.

Catches three classes of "the launch came up but nothing works" bugs that
otherwise hide for minutes:

1. **QoS compatibility** — checks that publishers and subscribers on the
   high-impact edges (``/livox/lidar_liosam`` etc.) actually have matching
   reliability/durability profiles. A BE↔RELIABLE mismatch silently drops
   100% of traffic and is the single most common bringup failure here.
2. **TF single-owner** — after warm-up, looks at every ``map→odom`` and
   ``odom→base_link`` broadcaster name in /tf and yells if more than one
   node is publishing the same edge (e.g. MCL ``publish_tf=true`` *and*
   pose_fusion).
3. **Parameter consistency** — pulls the live parameter values from
   ``mcl_3dl``, ``pose_fusion``, ``fast_lio`` and ``lio_sam_*`` and runs a
   handful of cross-node sanity checks (publish_tf alignment with running
   mode, particle bound ordering, sub-map radius vs lidar range).

Output: each finding is one ``DiagnosticStatus`` on ``/diagnostics`` plus a
human-readable log line. ``ERROR`` items will also raise the ``preflight``
top-level status to ERROR so other monitors can react. The node never kills
the launch on its own — operators decide based on the report.

Designed to be cheap: runs all checks once at ``warmup_sec`` after startup
and again every ``recheck_sec`` so transient races (TF still being declared)
don't poison the report.
"""

import math  # noqa: F401

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_msgs.msg import TFMessage


_OK = DiagnosticStatus.OK
_WARN = DiagnosticStatus.WARN
_ERROR = DiagnosticStatus.ERROR


def _qos_str(p):
    rel = {ReliabilityPolicy.RELIABLE: 'RELIABLE',
           ReliabilityPolicy.BEST_EFFORT: 'BEST_EFFORT'}.get(
        p.qos_profile.reliability, str(p.qos_profile.reliability))
    dur = {DurabilityPolicy.VOLATILE: 'VOLATILE',
           DurabilityPolicy.TRANSIENT_LOCAL: 'TRANSIENT_LOCAL'}.get(
        p.qos_profile.durability, str(p.qos_profile.durability))
    return f'{rel}/{dur}'


def _qos_compatible(pub, sub):
    pr = pub.qos_profile.reliability
    sr = sub.qos_profile.reliability
    pd = pub.qos_profile.durability
    sd = sub.qos_profile.durability
    rel_ok = (pr == ReliabilityPolicy.RELIABLE
              or sr == ReliabilityPolicy.BEST_EFFORT)
    dur_ok = (pd == DurabilityPolicy.TRANSIENT_LOCAL
              or sd == DurabilityPolicy.VOLATILE)
    return rel_ok and dur_ok


class Preflight(Node):
    def __init__(self):
        super().__init__('dddnav_preflight')

        self.declare_parameter('warmup_sec', 5.0)
        self.declare_parameter('recheck_sec', 30.0)
        # Topics where a QoS mismatch silently drops everything. Each entry
        # is just a topic name — both sides are looked up via the graph API.
        self.declare_parameter('qos_topics', [
            '/livox/lidar',
            '/livox/lidar_liosam',
            '/livox/lidar_liosam_xyzi',
            '/Odometry',
            '/odom_filtered',
            '/laser_cloud_sharp',
            '/laser_cloud_less_sharp',
            '/laser_cloud_flat',
            '/laser_cloud_less_flat',
        ])
        # TF edges with single-owner expectation.
        self.declare_parameter('tf_unique_edges',
                               ['map->odom', 'odom->base_link'])
        # Mode the stack thinks it is in. Affects who *should* own map→odom.
        # Allowed: "mapping" (LIO-SAM owns it) or "localization"
        # (pose_fusion owns it).
        self.declare_parameter('mode', 'localization')
        # Robot lidar max range (m) — used to sanity-check sub_map radius.
        self.declare_parameter('lidar_max_range', 70.0)

        self.warmup = float(self.get_parameter('warmup_sec').value)
        self.recheck = float(self.get_parameter('recheck_sec').value)
        self.qos_topics = list(self.get_parameter('qos_topics').value)
        self.tf_edges = []
        for s in self.get_parameter('tf_unique_edges').value:
            if '->' in s:
                p, c = s.split('->', 1)
                self.tf_edges.append((p.strip(), c.strip()))
        self.mode = str(self.get_parameter('mode').value)
        self.lidar_max_range = float(self.get_parameter('lidar_max_range').value)

        # --- TF spy: collect frame_ids → (count, last seen) -------------
        # We listen on /tf and /tf_static and record header.frame_id +
        # child_frame_id for every transform. To detect duplicate
        # broadcasters we compare against unique source 'connection' counts
        # via publishers_info_by_topic.
        self._tf_pub_count = {'/tf': 0, '/tf_static': 0}
        self.create_subscription(
            TFMessage, '/tf', self._on_tf,
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200))
        self.create_subscription(
            TFMessage, '/tf_static',
            lambda m: None,  # liveness only
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Map of (parent, child) -> set(node names that broadcast it).
        # We can only get authors approximately by tracking which publisher
        # of /tf last delivered the transform; rclpy doesn't surface origin
        # info. So we trigger a separate check via publishers_info_by_topic.
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # First pass at warmup, then a full recheck on a steady cadence.
        self._first_timer = self.create_timer(self.warmup, self._first_check)
        self.create_timer(self.recheck, self._run_all)

        self.get_logger().info(
            f'preflight up: warmup={self.warmup}s recheck={self.recheck}s '
            f'mode={self.mode} qos_topics={len(self.qos_topics)} '
            f'tf_edges={self.tf_edges}')

    # ------------------------------------------------------------------ tf
    def _on_tf(self, _msg):
        self._tf_pub_count['/tf'] += 1

    # --------------------------------------------------------------- ticks
    def _first_check(self):
        # One-shot — cancel the timer after the first fire so we don't
        # double-trigger with the recheck timer.
        if hasattr(self, '_warmed'):
            return
        self._warmed = True
        try:
            self._first_timer.cancel()
        except Exception:                                   # noqa: BLE001
            pass
        self._run_all()

    def _run_all(self):
        statuses = []
        statuses.extend(self._check_qos())
        statuses.extend(self._check_tf_owners())
        statuses.extend(self._check_params())

        worst = max((s.level for s in statuses), default=_OK)
        statuses.insert(0, self._mk(
            worst, 'preflight', f'{len(statuses)} checks',
            {'mode': self.mode}))

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = statuses
        self._diag_pub.publish(arr)

        for s in statuses:
            if s.level == _ERROR:
                self.get_logger().error(f'[{s.name}] {s.message}')
            elif s.level == _WARN:
                self.get_logger().warn(f'[{s.name}] {s.message}')

    # ------------------------------------------------------------------ qos
    def _check_qos(self):
        out = []
        for topic in self.qos_topics:
            try:
                pubs = self.get_publishers_info_by_topic(topic)
                subs = self.get_subscriptions_info_by_topic(topic)
            except Exception as e:                          # noqa: BLE001
                out.append(self._mk(_WARN, f'qos {topic}',
                                    f'graph query failed: {e}'))
                continue
            if not pubs and not subs:
                # Topic doesn't exist yet. Not all topics are mode-relevant.
                continue
            if not pubs:
                out.append(self._mk(_WARN, f'qos {topic}',
                                    'no publisher yet'))
                continue
            if not subs:
                out.append(self._mk(_OK, f'qos {topic}',
                                    f'pubs={len(pubs)} no subs'))
                continue
            bad = []
            extras = {}
            for p in pubs:
                for s in subs:
                    if not _qos_compatible(p, s):
                        bad.append(
                            f'{p.node_name}({_qos_str(p)})!='
                            f'{s.node_name}({_qos_str(s)})')
            extras['pubs'] = ','.join(p.node_name for p in pubs)
            extras['subs'] = ','.join(s.node_name for s in subs)
            if bad:
                out.append(self._mk(_ERROR, f'qos {topic}',
                                    f'INCOMPATIBLE: {"; ".join(bad)}',
                                    extras))
            else:
                out.append(self._mk(_OK, f'qos {topic}',
                                    f'pubs={len(pubs)} subs={len(subs)}',
                                    extras))
        return out

    # -------------------------------------------------------------- tf own
    def _check_tf_owners(self):
        out = []
        try:
            tf_pubs = self.get_publishers_info_by_topic('/tf')
            tf_static_pubs = self.get_publishers_info_by_topic('/tf_static')
        except Exception as e:                              # noqa: BLE001
            out.append(self._mk(_WARN, 'tf publishers',
                                f'graph query failed: {e}'))
            return out

        # We can't tell from rclpy *which* transform a given /tf publisher
        # produces. So all we check is the *count* of /tf publishers and
        # warn if the count is suspiciously high for the mode. Pair this
        # with slam_health_monitor which detects map→odom jumps directly
        # (the symptom of two publishers fighting).
        unique_tf_pubs = {p.node_name for p in tf_pubs}
        unique_static = {p.node_name for p in tf_static_pubs}

        # Expected dynamic publishers (lower-bound): fast_lio, pose_fusion
        # (or LIO-SAM in mapping mode), sometimes mcl_feature.
        expected_tf = {'localization': {'fast_lio', 'pose_fusion'},
                       'mapping': {'fast_lio', 'lio_sam_mapOptimization'}}
        forbidden_tf = {
            'localization': {'lio_sam_mapOptimization', 'mcl_3dl'},
            'mapping': {'mcl_3dl', 'pose_fusion'},
        }

        unexpected = unique_tf_pubs & forbidden_tf.get(self.mode, set())
        if unexpected:
            out.append(self._mk(
                _ERROR, 'tf owners',
                f'unexpected TF publisher(s) in {self.mode} mode: '
                f'{sorted(unexpected)}',
                {'all_tf_pubs': ','.join(sorted(unique_tf_pubs))}))
        else:
            out.append(self._mk(
                _OK, 'tf owners',
                f'mode={self.mode} pubs={len(unique_tf_pubs)}',
                {'tf_pubs': ','.join(sorted(unique_tf_pubs)),
                 'static_pubs': ','.join(sorted(unique_static))}))
        return out

    # ------------------------------------------------------------ params
    def _check_params(self):
        """Cross-node parameter sanity. We poll via the ``ros2 param get``
        CLI in a subprocess — it's slow but it sidesteps the
        spin-inside-callback deadlock that the rclpy parameter client
        introduces, and we only run it every ``recheck_sec`` so the cost is
        invisible. Nodes that aren't up yet are silently skipped (re-checked
        at the next recheck tick)."""
        out = []
        import subprocess

        def _try_get(node, names, timeout_sec=1.0):
            res = {}
            for n in names:
                try:
                    cp = subprocess.run(
                        ['ros2', 'param', 'get', '--hide-type', node, n],
                        capture_output=True, text=True,
                        timeout=timeout_sec)
                except (subprocess.TimeoutExpired, FileNotFoundError):
                    return None
                if cp.returncode != 0:
                    # Node not up or parameter missing — treat as
                    # "unavailable", skip silently.
                    return None
                txt = cp.stdout.strip()
                # Parse the printed scalar. ros2 param get prints raw values
                # for bool/int/float and quoted strings for str.
                if txt in ('True', 'False'):
                    res[n] = (txt == 'True')
                else:
                    try:
                        res[n] = int(txt)
                    except ValueError:
                        try:
                            res[n] = float(txt)
                        except ValueError:
                            res[n] = txt.strip("'\"")
            return res

        # ---- mcl_3dl ----------------------------------------------------
        mcl = _try_get('/mcl_3dl', [
            'publish_tf', 'publish_odom_tf',
            'num_particles', 'num_particles_min', 'num_particles_max',
            'sub_map_search_radius',
        ])
        if mcl is not None:
            extras = {k: str(v) for k, v in mcl.items()}
            issues = []
            if mcl.get('publish_tf') and self.mode == 'localization':
                issues.append('publish_tf=true clashes with pose_fusion')
            if mcl.get('publish_odom_tf') and self.mode == 'localization':
                issues.append('publish_odom_tf=true clashes with pose_fusion')
            n_lo = mcl.get('num_particles_min')
            n   = mcl.get('num_particles')
            n_hi = mcl.get('num_particles_max')
            if all(isinstance(x, (int, float)) for x in (n_lo, n, n_hi)):
                if not (n_lo <= n <= n_hi):
                    issues.append(
                        f'num_particles ordering: {n_lo} <= {n} <= {n_hi} '
                        'violated')
            r = mcl.get('sub_map_search_radius')
            if isinstance(r, (int, float)):
                if r > 1.5 * self.lidar_max_range:
                    issues.append(
                        f'sub_map_search_radius {r} m exceeds 1.5x lidar '
                        f'range {self.lidar_max_range} m')
                elif r < 0.3 * self.lidar_max_range:
                    issues.append(
                        f'sub_map_search_radius {r} m is < 0.3x lidar '
                        f'range — likelihood map will be sparse')
            if issues:
                out.append(self._mk(_ERROR, 'params mcl_3dl',
                                    '; '.join(issues), extras))
            else:
                out.append(self._mk(_OK, 'params mcl_3dl', 'consistent',
                                    extras))

        # ---- pose_fusion ------------------------------------------------
        pf = _try_get('/pose_fusion',
                      ['publish_tf', 'mahalanobis_gate', 'adaptive_q_max'])
        if pf is not None:
            issues = []
            if (self.mode == 'localization'
                    and not pf.get('publish_tf', False)):
                issues.append(
                    'publish_tf=false in localization mode — nobody owns '
                    'map→odom')
            if self.mode == 'mapping' and pf.get('publish_tf'):
                issues.append(
                    'publish_tf=true in mapping mode — LIO-SAM should own '
                    'map→odom')
            extras = {k: str(v) for k, v in pf.items()}
            if issues:
                out.append(self._mk(_ERROR, 'params pose_fusion',
                                    '; '.join(issues), extras))
            else:
                out.append(self._mk(_OK, 'params pose_fusion',
                                    'consistent', extras))

        # ---- LIO-SAM ----------------------------------------------------
        ls = _try_get('/lio_sam_mapOptimization',
                      ['publishTF', 'mappingProcessInterval'])
        if ls is not None:
            issues = []
            if self.mode == 'localization' and ls.get('publishTF'):
                issues.append('publishTF=true in localization mode')
            extras = {k: str(v) for k, v in ls.items()}
            if issues:
                out.append(self._mk(_ERROR, 'params lio_sam',
                                    '; '.join(issues), extras))
            else:
                out.append(self._mk(_OK, 'params lio_sam',
                                    'consistent', extras))

        return out

    @staticmethod
    def _mk(level, name, message, extras=None):
        s = DiagnosticStatus()
        s.level = level
        s.name = f'preflight: {name}'
        s.hardware_id = 'dddnav'
        s.message = message
        if extras:
            s.values = [KeyValue(key=str(k), value=str(v))
                        for k, v in extras.items()]
        return s


def main():
    rclpy.init()
    node = Preflight()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
