#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _border_indices(width: int, height: int, border_cells: int) -> set[int]:
    b = max(0, int(border_cells))
    if b == 0 or width <= 0 or height <= 0:
        return set()
    b = min(b, width, height)
    indices: set[int] = set()
    for iy in range(height):
        row = iy * width
        for ix in range(width):
            if ix < b or ix >= (width - b) or iy < b or iy >= (height - b):
                indices.add(row + ix)
    return indices


class _Once(Node):
    def __init__(self, topic: str, timeout_sec: float) -> None:
        super().__init__("inspect_map_once")
        self._topic = topic
        self._timeout_sec = timeout_sec
        self._msg: OccupancyGrid | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, topic, self._cb, qos)

    def _cb(self, msg: OccupancyGrid) -> None:
        if self._msg is None:
            self._msg = msg

    def wait(self) -> OccupancyGrid | None:
        end_ns = self.get_clock().now().nanoseconds + int(max(0.1, self._timeout_sec) * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_ns and self._msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._msg


def main() -> int:
    parser = argparse.ArgumentParser(description="Subscribe once to a nav_msgs/OccupancyGrid and print basic stats.")
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--border-cells", type=int, default=5)
    args = parser.parse_args()

    rclpy.init()
    node = _Once(args.topic, args.timeout_sec)
    msg = node.wait()

    if msg is None:
        print(f"[inspect_map_once] ERROR: no message received on {args.topic} within {args.timeout_sec:.1f}s")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    w = int(msg.info.width)
    h = int(msg.info.height)
    res = float(msg.info.resolution)
    ox = float(msg.info.origin.position.x)
    oy = float(msg.info.origin.position.y)
    stamp = msg.header.stamp

    data = list(msg.data)
    counts = Counter(data)
    total = max(1, len(data))

    border = _border_indices(w, h, args.border_cells)
    border_counts = Counter(data[i] for i in border) if border else Counter()

    def _fmt_common(c: Counter, keys: list[int]) -> str:
        parts = []
        for k in keys:
            parts.append(f"{k}:{c.get(k, 0)}")
        other = sum(v for kk, v in c.items() if kk not in keys)
        if other:
            parts.append(f"other:{other}")
        return " ".join(parts)

    print(
        f"[inspect_map_once] {args.topic}: w={w} h={h} res={res:.3f} "
        f"origin=({ox:.2f},{oy:.2f}) stamp={stamp.sec + stamp.nanosec * 1e-9:.3f}s total={total}"
    )
    print(
        "[inspect_map_once] counts "
        + _fmt_common(counts, [-1, 0, 100])
        + f" (unknown={counts.get(-1, 0)/total:.3f} free={counts.get(0, 0)/total:.3f} occ={counts.get(100, 0)/total:.3f})"
    )
    if border:
        print("[inspect_map_once] border " + _fmt_common(border_counts, [-1, 0, 100]))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

