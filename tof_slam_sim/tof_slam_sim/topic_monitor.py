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
)
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message


DEFAULT_SENSOR_NAMES = [
    'front',
    'front_right',
    'right',
    'back_right',
    'back',
    'back_left',
    'left',
    'front_left',
]

DEFAULT_TOPIC_SPECS = [
    '/cmd_vel:geometry_msgs/msg/Twist:reliable',
    '/odom:nav_msgs/msg/Odometry:reliable',
    '/tf:tf2_msgs/msg/TFMessage:reliable',
    '/tf_static:tf2_msgs/msg/TFMessage:latched',
] + [
    f'/scan/{name}:sensor_msgs/msg/LaserScan:best_effort'
    for name in DEFAULT_SENSOR_NAMES
]


@dataclass
class TopicState:
    name: str
    type_str: str
    latched: bool
    reliability: ReliabilityPolicy
    last_msg_time: Time | None = None
    total_count: int = 0
    count_at_last_report: int = 0


def _parse_topic_spec(spec: str) -> Tuple[str, str, bool, ReliabilityPolicy]:
    """Parse specs like '/scan/front:sensor_msgs/msg/LaserScan:best_effort'."""
    parts = spec.split(':')
    if len(parts) < 2:
        raise ValueError(f'Invalid topic spec "{spec}". Expected /name:pkg/msg/Type[:options]')

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
        raise ValueError(f'Topic spec "{spec}" cannot request both reliable and best_effort QoS.')

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
        durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE,
    )


class TopicMonitor(Node):
    """Check that critical topics receive fresh messages within a timeout window."""

    def __init__(self) -> None:
        super().__init__('topic_monitor')

        topic_specs = self.declare_parameter('topics', DEFAULT_TOPIC_SPECS).value
        report_period_sec = float(self.declare_parameter('report_period_sec', 5.0).value)
        stale_seconds = float(self.declare_parameter('stale_seconds', 5.0).value)

        self._stale_duration = Duration(seconds=stale_seconds)
        self._topic_states: Dict[str, TopicState] = {}

        self._clock: Clock = self.get_clock()

        for spec in topic_specs:
            try:
                topic, type_str, latched, reliability = _parse_topic_spec(spec)
                msg_type = get_message(type_str)
            except Exception as exc:
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

            def _make_callback(topic_name: str) -> Callable:
                def _cb(msg) -> None:
                    del msg  # unused
                    st = self._topic_states[topic_name]
                    st.last_msg_time = self._clock.now()
                    st.total_count += 1
                return _cb

            self.create_subscription(
                msg_type,
                topic,
                _make_callback(topic),
                qos
            )
            qos_label = 'latched' if latched else reliability.name.lower()
            self.get_logger().info(f'Monitoring {topic} [{type_str}] ({qos_label})')

        if not self._topic_states:
            self.get_logger().warn('No topics configured for monitoring.')

        self._report_timer = self.create_timer(report_period_sec, self._report_status)

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
                        status = f'OK (latched, last {seconds_since:.1f}s ago, total {state.total_count})'
                else:
                    stale = last_dt > self._stale_duration or delta_count == 0
                    if stale:
                        warn = True
                        if delta_count == 0:
                            status = f'STALE - no new messages in window (last {seconds_since:.1f}s ago)'
                        else:
                            status = f'STALE - last {seconds_since:.1f}s ago (+{delta_count} msgs)'
                    else:
                        status = f'OK - last {seconds_since:.1f}s ago (+{delta_count} msgs)'

            report_lines.append(f'{state.name}: {status}')

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
