#!/usr/bin/env python3
import argparse, json, os, signal, sys, time, logging
from datetime import datetime
from logging.handlers import RotatingFileHandler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict


class GenericLogMonitor(Node):
    def __init__(self, topics, log_path, flush_secs, qos_reliable, queue_depth):
        super().__init__('generic_log_monitor')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if qos_reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_depth
        )

        self.logger = logging.getLogger('ros_topic_logger')
        self.logger.setLevel(logging.INFO)
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        handler = RotatingFileHandler(log_path, maxBytes=20*1024*1024, backupCount=5)
        fmt = logging.Formatter('%(message)s')
        handler.setFormatter(fmt)
        self.logger.addHandler(handler)

        self._subs = []
        self._flush_secs = flush_secs
        self._last_flush = time.time()

        for name, type_str in topics:
            try:
                msg_type = get_message(type_str)
            except Exception as e:
                self.get_logger().error(f'Cannot import type "{type_str}" for topic "{name}": {e}')
                continue

            def make_cb(topic_name):
                def _cb(msg):
                    record = {
                        "stamp": self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9,
                        "iso_time": datetime.utcnow().isoformat() + 'Z',
                        "topic": topic_name,
                        "data": message_to_ordereddict(msg)
                    }
                    self.logger.info(json.dumps(record, separators=(',', ':')))
                    now = time.time()
                    if now - self._last_flush >= self._flush_secs:
                        for h in self.logger.handlers:
                            h.flush()
                        self._last_flush = now
                return _cb

            sub = self.create_subscription(msg_type, name, make_cb(name), qos)
            self._subs.append(sub)
            self.get_logger().info(f'Logging: {name} [{type_str}]')

        self.get_logger().info(f'Writing logs to: {log_path}')


def parse_topics(pairs):
    topics = []
    for p in pairs:
        if ':' not in p:
            raise ValueError(f'Invalid --topics entry: "{p}". Use /name:pkg/msg/Type')
        name, typ = p.split(':', 1)
        topics.append((name.strip(), typ.strip()))
    return topics


def main():
    ap = argparse.ArgumentParser(description='Generic ROS 2 Topic Logger')
    ap.add_argument('--topics', nargs='+', required=True,
                    help='List like /cmd_vel:geometry_msgs/msg/Twist /odom:nav_msgs/msg/Odometry')
    ap.add_argument('--log-dir', default=os.path.expanduser('~/.ros/monitor_logs'))
    ap.add_argument('--log-name', default='topic_log')
    ap.add_argument('--flush-secs', type=float, default=2.0)
    ap.add_argument('--reliable', action='store_true')
    ap.add_argument('--queue-depth', type=int, default=50)
    args = ap.parse_args()

    ts = datetime.utcnow().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(args.log_dir, f'{args.log_name}_{ts}.jsonl')

    rclpy.init()
    node = GenericLogMonitor(
        topics=parse_topics(args.topics),
        log_path=log_path,
        flush_secs=args.flush_secs,
        qos_reliable=bool(args.reliable),
        queue_depth=args.queue_depth
    )

    def _shutdown(signum, frame):
        node.get_logger().info('Shutting down logger...')
        for h in node.logger.handlers:
            try:
                h.flush()
            except:
                pass
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
