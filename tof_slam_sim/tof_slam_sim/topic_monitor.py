#!/usr/bin/env python3
"""ROS 2 node that periodically checks critical topics for recent traffic."""

from __future__ import annotations

from dataclasses import dataclass
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
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener, TransformException


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
                    del msg
                    st = self._topic_states[topic_name]
                    st.last_msg_time = self._clock.now()
                    st.total_count += 1

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
