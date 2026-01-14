#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray


@dataclass(frozen=True)
class _MarkerPose:
    robot: str
    x: float
    y: float


def _robot_to_model_index(robot: str) -> Optional[int]:
    if robot == "robot":
        return 0
    m = re.fullmatch(r"robot(\d+)", robot)
    if not m:
        return None
    idx = int(m.group(1))
    return idx - 1 if idx >= 2 else None


class _Checker(Node):
    def __init__(self, *, marker_topic: str) -> None:
        super().__init__("check_marker_alignment")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._marker_msg: Optional[MarkerArray] = None
        self.create_subscription(MarkerArray, marker_topic, self._on_markers, qos)

        self._tf_buffer = Buffer()
        # This node is actively spun while waiting, so don't spawn an extra TF thread.
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._gazebo: dict[str, PoseStamped] = {}
        self._gazebo_subs = []

    def subscribe_gazebo_pose(self, robot: str) -> None:
        idx = _robot_to_model_index(robot)
        if idx is None:
            return
        topic = f"/model/x500_small_tof_{idx}/pose"
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._gazebo_subs.append(
            self.create_subscription(PoseStamped, topic, lambda msg, r=robot: self._on_gz(r, msg), qos)
        )

    def _on_gz(self, robot: str, msg: PoseStamped) -> None:
        self._gazebo[robot] = msg

    def _on_markers(self, msg: MarkerArray) -> None:
        if self._marker_msg is None:
            self._marker_msg = msg

    def wait_markers(self, timeout_sec: float) -> Optional[MarkerArray]:
        end_ns = self.get_clock().now().nanoseconds + int(max(0.1, timeout_sec) * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_ns and self._marker_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._marker_msg

    def wait_gazebo(self, robots: list[str], timeout_sec: float) -> None:
        if not robots:
            return
        end_ns = self.get_clock().now().nanoseconds + int(max(0.1, timeout_sec) * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_ns:
            have = sum(1 for r in robots if r in self._gazebo)
            if have == len(robots):
                return
            rclpy.spin_once(self, timeout_sec=0.1)

    def lookup_xy(self, *, target: str, source: str, timeout_sec: float) -> Optional[tuple[float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException:
            return None
        return float(tf.transform.translation.x), float(tf.transform.translation.y)


def _extract_robot_poses(markers: MarkerArray) -> list[_MarkerPose]:
    # DroneHealthDashboard uses id = i*10 + {0,1,2} for arrow/label/trail.
    labels: dict[int, str] = {}
    arrows: dict[int, tuple[float, float]] = {}
    for m in markers.markers:
        idx = int(m.id) // 10
        if m.ns == "drone_label" and m.text:
            labels[idx] = str(m.text).split()[0]
        elif m.ns == "drone_pose":
            arrows[idx] = (float(m.pose.position.x), float(m.pose.position.y))

    out: list[_MarkerPose] = []
    for idx, robot in labels.items():
        xy = arrows.get(idx)
        if xy is None:
            continue
        out.append(_MarkerPose(robot=robot, x=xy[0], y=xy[1]))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify RViz marker poses match TF (and optionally Gazebo poses).")
    parser.add_argument("--marker-topic", default="/swarm/drone_markers")
    parser.add_argument("--map-frame", default="robot/map")
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--tf-timeout-sec", type=float, default=0.25)
    parser.add_argument("--tol-m", type=float, default=0.35)
    parser.add_argument("--check-gazebo", action="store_true", help="Also compare TF to Gazebo /model/*/pose topics.")
    args = parser.parse_args()

    rclpy.init()
    node = _Checker(marker_topic=str(args.marker_topic))

    try:
        markers = node.wait_markers(timeout_sec=float(args.timeout_sec))
        if markers is None:
            print(f"[marker_check] ERROR: no markers received on {args.marker_topic} within {args.timeout_sec:.1f}s")
            return 2

        poses = _extract_robot_poses(markers)
        if not poses:
            print("[marker_check] ERROR: no drone_pose/drone_label markers found in MarkerArray")
            return 2

        if args.check_gazebo:
            for p in poses:
                node.subscribe_gazebo_pose(p.robot)
            node.wait_gazebo([p.robot for p in poses], timeout_sec=float(args.timeout_sec))

        worst = 0.0
        ok = True
        for p in poses:
            base = f"{p.robot}/base_footprint"
            tf_xy = node.lookup_xy(
                target=str(args.map_frame),
                source=base,
                timeout_sec=float(args.tf_timeout_sec),
            )
            if tf_xy is None:
                print(f"[marker_check] ERROR: TF missing {args.map_frame} <- {base}")
                ok = False
                continue
            dx = float(tf_xy[0] - p.x)
            dy = float(tf_xy[1] - p.y)
            err = float(math.hypot(dx, dy))
            worst = max(worst, err)
            print(f"[marker_check] {p.robot}: marker=({p.x:.2f},{p.y:.2f}) tf=({tf_xy[0]:.2f},{tf_xy[1]:.2f}) err={err:.2f}m")
            if err > float(args.tol_m):
                ok = False

            if args.check_gazebo and p.robot in node._gazebo:
                gz = node._gazebo[p.robot].pose.position
                dgx = float(tf_xy[0] - float(gz.x))
                dgy = float(tf_xy[1] - float(gz.y))
                gerr = float(math.hypot(dgx, dgy))
                print(f"[marker_check] {p.robot}: tf_vs_gz err={gerr:.2f}m (gz=({gz.x:.2f},{gz.y:.2f}))")

        if not ok:
            print(f"[marker_check] ERROR: marker alignment exceeded tol={float(args.tol_m):.2f}m (worst={worst:.2f}m)")
            return 2

        print(f"[marker_check] OK: markers aligned with TF (worst={worst:.2f}m <= tol={float(args.tol_m):.2f}m)")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
