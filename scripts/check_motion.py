#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time
from typing import Optional

import rclpy
from dataclasses import dataclass

from nav_msgs.msg import Odometry


@dataclass(frozen=True)
class MotionStats:
    samples: int
    duration_sec: float
    displacement_m: float
    max_displacement_m: float
    path_length_m: float


def measure_motion(*, topic: str, duration_sec: float) -> Optional[MotionStats]:
    rclpy.init()
    node = rclpy.create_node('check_motion')

    samples: list[tuple[float, float]] = []

    def cb(msg: Odometry) -> None:
        p = msg.pose.pose.position
        samples.append((float(p.x), float(p.y)))

    node.create_subscription(Odometry, topic, cb, 10)

    start = time.time()
    while time.time() - start < duration_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 2:
        return None

    x0, y0 = samples[0]
    x1, y1 = samples[-1]
    displacement = float(math.hypot(x1 - x0, y1 - y0))

    max_disp = 0.0
    path_len = 0.0
    px, py = x0, y0
    for x, y in samples:
        max_disp = max(max_disp, float(math.hypot(x - x0, y - y0)))
        path_len += float(math.hypot(x - px, y - py))
        px, py = x, y

    return MotionStats(
        samples=len(samples),
        duration_sec=float(duration_sec),
        displacement_m=displacement,
        max_displacement_m=float(max_disp),
        path_length_m=float(path_len),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description='Measure /odom motion over time.')
    parser.add_argument('topic', nargs='?', default='/odom', help='Odometry topic to sample (default: /odom)')
    parser.add_argument(
        '--duration-sec',
        type=float,
        default=5.0,
        help='Sampling duration (wall time) in seconds (default: 5.0)',
    )
    args = parser.parse_args()

    stats = measure_motion(topic=str(args.topic), duration_sec=float(args.duration_sec))
    if stats is None:
        print(f'[motion] {args.topic}: no odom samples in {args.duration_sec:.1f}s')
        return 2
    print(
        f'[motion] {args.topic}: samples={stats.samples} duration={stats.duration_sec:.1f}s '
        f'displacement={stats.displacement_m:.3f}m max_displacement={stats.max_displacement_m:.3f}m '
        f'path_length={stats.path_length_m:.3f}m'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
