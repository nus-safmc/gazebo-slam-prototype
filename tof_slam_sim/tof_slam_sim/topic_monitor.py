#!/usr/bin/env python3
"""ROS 2 node that periodically checks critical topics for recent traffic."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Dict, List, Sequence, Set, Tuple

import rclpy
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    QoSPolicyKind,
)
try:
    from rclpy.event_handler import (
        QoSRequestedIncompatibleQoSInfo,
        SubscriptionEventCallbacks,
    )
except ImportError:  # pragma: no cover
    from rclpy.qos_event import (  # type: ignore[no-redef]
        QoSRequestedIncompatibleQoSInfo,
        SubscriptionEventCallbacks,
    )
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

try:  # Optional (only present in PX4 SITL track)
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
except Exception:  # pragma: no cover - defensive for non-PX4 environments
    OffboardControlMode = None  # type: ignore[assignment]
    TrajectorySetpoint = None  # type: ignore[assignment]
    VehicleOdometry = None  # type: ignore[assignment]
    VehicleStatus = None  # type: ignore[assignment]


def _is_finite(value: float) -> bool:
    return math.isfinite(float(value))


def _quat_to_rot(w: float, x: float, y: float, z: float) -> list[list[float]]:
    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z

    wx = w * x
    wy = w * y
    wz = w * z
    xy = x * y
    xz = x * z
    yz = y * z

    return [
        [ww + xx - yy - zz, 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), ww - xx + yy - zz, 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), ww - xx - yy + zz],
    ]


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _yaw_from_rot_enu_flu(r: list[list[float]]) -> float:
    return math.atan2(r[1][0], r[0][0])


def _yaw_enu_from_px4_q(q_wxyz: list[float]) -> float | None:
    if len(q_wxyz) != 4:
        return None
    w, x, y, z = (float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]))
    if not all(_is_finite(v) for v in (w, x, y, z)):
        return None

    # PX4: q is FRD(body) -> NED(world). We want yaw in ENU about +Z (up).
    r_ned_frd = _quat_to_rot(w, x, y, z)
    r_enu_ned = [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    r_frd_flu = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    r_enu_flu = _mat_mul(_mat_mul(r_enu_ned, r_ned_frd), r_frd_flu)
    return float(_yaw_from_rot_enu_flu(r_enu_flu))


DEFAULT_SENSOR_NAMES: Tuple[str, ...] = (
    'front',
    'front_right',
    'right',
    'back_right',
    'back',
    'back_left',
    'left',
    'front_left',
)

DEFAULT_TOPIC_SPECS: Tuple[str, ...] = (
    '/cmd_vel:geometry_msgs/msg/Twist:reliable',
    '/odom:nav_msgs/msg/Odometry:reliable',
    '/tf:tf2_msgs/msg/TFMessage:reliable',
    '/tf_static:tf2_msgs/msg/TFMessage:latched',
    '/map:nav_msgs/msg/OccupancyGrid:reliable',
    '/scan_merged:sensor_msgs/msg/LaserScan:reliable',
) + tuple(
    f'/scan/{name}:sensor_msgs/msg/LaserScan:best_effort'
    for name in DEFAULT_SENSOR_NAMES
)

DEFAULT_TRANSFORM_SPECS: Tuple[str, ...] = (
    'robot/map->robot/odom',
    'robot/odom->robot/base_link',
    'robot/map->robot/base_link',
)


@dataclass
class TopicState:
    name: str
    type_str: str
    latched: bool
    reliability: ReliabilityPolicy
    last_msg_time: Time | None = None
    total_count: int = 0
    count_at_last_report: int = 0
    incompatible_qos_total: int = 0
    last_incompatible_policy: QoSPolicyKind | None = None
    last_summary: str | None = None
    last_detail_error: str | None = None


@dataclass
class TransformState:
    parent: str
    child: str
    total_success: int = 0
    success_at_last_report: int = 0
    last_success_time: Time | None = None
    last_exception: str | None = None


def _parse_topic_spec(spec: str) -> Tuple[str, str, bool, ReliabilityPolicy]:
    """Parse a topic spec string into components."""
    parts = spec.split(':')
    if len(parts) < 2:
        raise ValueError(
            f'Invalid topic spec "{spec}". Expected "/name:pkg/msg/Type[:options]"'
        )

    topic = parts[0].strip()
    msg_type = parts[1].strip()

    options: Set[str] = set()
    if len(parts) >= 3:
        for opt in parts[2].split('+'):
            opt = opt.strip().lower()
            if opt:
                options.add(opt)

    latched = 'latched' in options
    if 'best_effort' in options and 'reliable' in options:
        raise ValueError(
            f'Topic spec "{spec}" cannot request both reliable and best_effort QoS.'
        )

    if 'best_effort' in options:
        reliability = ReliabilityPolicy.BEST_EFFORT
    elif 'reliable' in options or latched:
        reliability = ReliabilityPolicy.RELIABLE
    else:
        reliability = ReliabilityPolicy.BEST_EFFORT

    return topic, msg_type, latched, reliability


def _qos_profile(latched: bool, reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1 if latched else 10,
        reliability=reliability,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
        if latched
        else DurabilityPolicy.VOLATILE,
    )


def _ensure_list(value: Sequence[str] | str) -> List[str]:
    if isinstance(value, str):
        return [v.strip() for v in value.split(',') if v.strip()]
    return list(value)


class TopicMonitor(Node):
    """Check that critical topics receive fresh messages within a timeout window."""

    def __init__(self) -> None:
        super().__init__('topic_monitor')

        topic_specs_param = self.declare_parameter('topics', list(DEFAULT_TOPIC_SPECS))
        report_period_param = self.declare_parameter('report_period_sec', 5.0)
        stale_seconds_param = self.declare_parameter('stale_seconds', 5.0)
        transform_specs_param = self.declare_parameter(
            'required_transforms', list(DEFAULT_TRANSFORM_SPECS)
        )

        topic_specs = _ensure_list(topic_specs_param.value)
        report_period_sec = float(report_period_param.value)
        stale_seconds = float(stale_seconds_param.value)
        transform_specs = _ensure_list(transform_specs_param.value)

        self._stale_duration = Duration(seconds=stale_seconds)
        self._topic_states: Dict[str, TopicState] = {}
        self._clock: Clock = self.get_clock()
        self._tf_buffer: Buffer | None = None
        self._tf_listener: TransformListener | None = None
        self._transform_pairs: List[Tuple[str, str]] = []
        self._transform_states: Dict[Tuple[str, str], TransformState] = {}

        for spec in topic_specs:
            try:
                topic, type_str, latched, reliability = _parse_topic_spec(spec)
                msg_type = get_message(type_str)
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().error(f'Cannot monitor spec "{spec}": {exc}')
                continue

            state = TopicState(
                name=topic,
                type_str=type_str,
                latched=latched,
                reliability=reliability,
            )
            self._topic_states[topic] = state

            qos = _qos_profile(latched, reliability)

            def _make_callback(topic_name: str) -> Callable[[object], None]:
                def _cb(msg: object) -> None:
                    st = self._topic_states[topic_name]
                    st.last_msg_time = self._clock.now()
                    st.total_count += 1
                    self._update_details(st, msg)

                return _cb

            callbacks = SubscriptionEventCallbacks(
                incompatible_qos=self._make_incompatible_qos_cb(topic)
            )

            self.create_subscription(
                msg_type,
                topic,
                _make_callback(topic),
                qos,
                event_callbacks=callbacks,
            )
            qos_label = 'latched' if latched else reliability.name.lower()
            self.get_logger().info(
                f'Monitoring {topic} [{type_str}] ({qos_label})'
            )

        if not self._topic_states:
            self.get_logger().warn('No topics configured for monitoring.')

        if transform_specs:
            for spec in transform_specs:
                if '->' not in spec:
                    self.get_logger().error(
                        f'Invalid transform spec "{spec}". Expected "parent->child".'
                    )
                    continue
                parent, child = (part.strip() for part in spec.split('->', 1))
                if not parent or not child:
                    self.get_logger().error(
                        f'Invalid transform spec "{spec}". Parent/child cannot be empty.'
                    )
                    continue
                self._transform_pairs.append((parent, child))

            if self._transform_pairs:
                self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
                self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
                for parent, child in self._transform_pairs:
                    self._transform_states[(parent, child)] = TransformState(parent, child)
                self.get_logger().info(
                    'Tracking transforms: ' + ', '.join(
                        f'{p}->{c}' for p, c in self._transform_pairs
                    )
                )

        self._report_timer = self.create_timer(
            max(0.1, report_period_sec), self._report_status
        )

    def _update_details(self, state: TopicState, msg: object) -> None:
        """Attach lightweight, human-readable info about key messages."""

        try:
            if isinstance(msg, OccupancyGrid):
                width = int(msg.info.width)
                height = int(msg.info.height)
                resolution = float(msg.info.resolution)
                data_len = len(msg.data)
                expected = width * height

                frame_id = str(msg.header.frame_id)
                stamp = msg.header.stamp
                stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9

                state.last_summary = (
                    f'frame={frame_id} stamp={stamp_s:.3f}s '
                    f'w={width} h={height} res={resolution:.3f} len={data_len}'
                )

                problems: List[str] = []
                if width <= 0 or height <= 0:
                    problems.append('width/height<=0')
                if not math.isfinite(resolution) or resolution <= 0.0:
                    problems.append('bad resolution')
                if expected != data_len:
                    problems.append(f'len(data)!={expected}')

                state.last_detail_error = (
                    'MALFORMED ' + ', '.join(problems) if problems else None
                )
                return

            if isinstance(msg, LaserScan):
                frame_id = str(msg.header.frame_id)
                stamp = msg.header.stamp
                stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9

                ranges = list(msg.ranges)
                finite = [r for r in ranges if math.isfinite(r)]
                finite = [r for r in finite if msg.range_min <= r <= msg.range_max]

                state.last_summary = (
                    f'frame={frame_id} stamp={stamp_s:.3f}s '
                    f'n={len(ranges)} finite={len(finite)} '
                    f'range_min={msg.range_min:.2f} range_max={msg.range_max:.2f}'
                )

                if finite:
                    state.last_summary += f' min={min(finite):.2f} max={max(finite):.2f}'
                state.last_detail_error = None
                return

            if isinstance(msg, Odometry):
                frame_id = str(msg.header.frame_id)
                child = str(msg.child_frame_id)
                stamp = msg.header.stamp
                stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                p = msg.pose.pose.position

                state.last_summary = (
                    f'frame={frame_id}->{child} stamp={stamp_s:.3f}s '
                    f'pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})'
                )
                state.last_detail_error = None
                return

            if isinstance(msg, Twist):
                lx = float(msg.linear.x)
                ly = float(msg.linear.y)
                az = float(msg.angular.z)
                state.last_summary = f'lin=({lx:.2f},{ly:.2f}) yaw_rate={az:.2f}'
                state.last_detail_error = None
                return

            if isinstance(msg, PoseStamped):
                p = msg.pose.position
                state.last_summary = f'pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})'
                state.last_detail_error = None
                return

            if VehicleOdometry is not None and isinstance(msg, VehicleOdometry):
                pn = float(msg.position[0])
                pe = float(msg.position[1])
                pd = float(msg.position[2])
                pos_ok = all(_is_finite(v) for v in (pn, pe, pd))

                x_enu = pe
                y_enu = pn
                z_enu = -pd

                yaw_enu = _yaw_enu_from_px4_q(list(msg.q))
                yaw_txt = 'yaw_enu=?' if yaw_enu is None else f'yaw_enu={yaw_enu:.2f}'

                if pos_ok:
                    state.last_summary = f'pos_enu=({x_enu:.2f},{y_enu:.2f},{z_enu:.2f}) {yaw_txt}'
                    state.last_detail_error = None
                else:
                    state.last_summary = f'pos_enu=(invalid) {yaw_txt}'
                    state.last_detail_error = 'MALFORMED position has NaN/inf'
                return

            if VehicleStatus is not None and isinstance(msg, VehicleStatus):
                state.last_summary = (
                    f'nav_state={int(msg.nav_state)} arming_state={int(msg.arming_state)} '
                    f'failsafe={bool(msg.failsafe)} preflight_ok={bool(msg.pre_flight_checks_pass)}'
                )
                state.last_detail_error = None
                return

            if OffboardControlMode is not None and isinstance(msg, OffboardControlMode):
                state.last_summary = (
                    f'velocity={bool(msg.velocity)} position={bool(msg.position)} '
                    f'attitude={bool(msg.attitude)} body_rate={bool(msg.body_rate)}'
                )
                state.last_detail_error = None
                return

            if TrajectorySetpoint is not None and isinstance(msg, TrajectorySetpoint):
                v = msg.velocity
                state.last_summary = f'vel_ned=({v[0]:.2f},{v[1]:.2f},{v[2]:.2f}) yawspeed={float(msg.yawspeed):.2f}'
                state.last_detail_error = None
                return
        except Exception as exc:  # pragma: no cover - defensive
            state.last_summary = None
            state.last_detail_error = f'detail error: {exc}'

    def _make_incompatible_qos_cb(
        self, topic_name: str
    ) -> Callable[[QoSRequestedIncompatibleQoSInfo], None]:
        def _cb(event: QoSRequestedIncompatibleQoSInfo) -> None:
            state = self._topic_states.get(topic_name)
            if state is None:
                return
            state.incompatible_qos_total = event.total_count
            try:
                policy = QoSPolicyKind(event.last_policy_kind)
            except ValueError:
                policy = None
            state.last_incompatible_policy = policy
            policy_name = policy.name if policy is not None else str(event.last_policy_kind)
            self.get_logger().warning(
                f'QoS incompatibility for {topic_name}: requested {state.reliability.name} '
                f'but publisher offered incompatible policy ({policy_name}). '
                f'Total mismatches: {event.total_count}'
            )

        return _cb

    def _report_status(self) -> None:
        if not self._topic_states:
            return

        now = self._clock.now()
        report_lines: List[str] = []
        warn = False

        for state in self._topic_states.values():
            delta_count = state.total_count - state.count_at_last_report
            state.count_at_last_report = state.total_count

            if state.last_msg_time is None:
                status = 'NO MESSAGES RECEIVED'
                warn = True
            else:
                last_dt = now - state.last_msg_time
                seconds_since = last_dt.nanoseconds * 1e-9

                if state.latched:
                    if state.total_count == 0:
                        status = 'NO LATCHED MESSAGE RECEIVED'
                        warn = True
                    else:
                        status = (
                            f'OK (latched, last {seconds_since:.1f}s ago, '
                            f'total {state.total_count})'
                        )
                else:
                    stale = last_dt > self._stale_duration or delta_count == 0
                    if stale:
                        warn = True
                        if delta_count == 0:
                            status = (
                                f'STALE - no new messages in window '
                                f'(last {seconds_since:.1f}s ago)'
                            )
                        else:
                            status = (
                                f'STALE - last {seconds_since:.1f}s ago '
                                f'(+{delta_count} msgs)'
                            )
                    else:
                        status = (
                            f'OK - last {seconds_since:.1f}s ago '
                            f'(+{delta_count} msgs)'
                        )

            report_lines.append(f'{state.name}: {status}')
            if state.incompatible_qos_total > 0:
                policy = (
                    state.last_incompatible_policy.name
                    if state.last_incompatible_policy is not None
                    else 'UNKNOWN'
                )
                report_lines.append(
                    f'{state.name}: QoS mismatch detected (last policy={policy}, '
                    f'total={state.incompatible_qos_total})'
                )
                warn = True

        if self._tf_buffer is not None and self._transform_pairs:
            for parent, child in self._transform_pairs:
                state = self._transform_states[(parent, child)]
                try:
                    transform = self._tf_buffer.lookup_transform(
                        parent,
                        child,
                        Time(seconds=0.0),
                        timeout=Duration(seconds=0.5),
                    )
                    state.total_success += 1
                    stamp = Time.from_msg(transform.header.stamp)
                    if stamp.nanoseconds == 0:
                        state.last_success_time = self._clock.now()
                    else:
                        state.last_success_time = stamp
                    delta = state.total_success - state.success_at_last_report
                    state.success_at_last_report = state.total_success
                    state.last_exception = None
                    if state.last_success_time is None:
                        seconds_since = float('inf')
                    else:
                        last_dt = now - state.last_success_time
                        seconds_since = last_dt.nanoseconds * 1e-9
                    report_lines.append(
                        f'{parent}->{child}: OK - last {seconds_since:.1f}s ago '
                        f'(+{delta} lookups)'
                    )
                except TransformException as exc:
                    warn = True
                    state.last_exception = exc.__class__.__name__
                    delta = state.total_success - state.success_at_last_report
                    if state.last_success_time is None:
                        status = 'NO TRANSFORM RECEIVED'
                    else:
                        last_dt = now - state.last_success_time
                        seconds_since = last_dt.nanoseconds * 1e-9
                        status = f'last {seconds_since:.1f}s ago'
                    report_lines.append(
                        f'{parent}->{child}: MISSING ({state.last_exception}) '
                        f'(+{delta} lookups, {status})'
                    )

        # Add small content summaries for high-value topics.
        for topic_name in (
            '/map',
            '/scan_merged',
            '/odom',
            '/cmd_vel',
            '/model/x500_small_tof_0/pose',
            '/fmu/out/vehicle_odometry',
            '/fmu/out/vehicle_status',
            '/fmu/in/offboard_control_mode',
            '/fmu/in/trajectory_setpoint',
        ):
            state = self._topic_states.get(topic_name)
            if state is None:
                continue
            if state.last_summary is not None:
                report_lines.append(f'{topic_name}: {state.last_summary}')
            if state.last_detail_error is not None:
                report_lines.append(f'{topic_name}: {state.last_detail_error}')
                warn = True

        message = 'Topic status:\n' + '\n'.join(f'  - {line}' for line in report_lines)
        if warn:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['TopicMonitor', 'main']
