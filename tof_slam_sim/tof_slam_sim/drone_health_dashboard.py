#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _scan_min_range(scan: LaserScan) -> float:
    best = float('inf')
    rmin = float(scan.range_min)
    rmax = float(scan.range_max) if math.isfinite(scan.range_max) else 10.0
    for r in scan.ranges:
        if not math.isfinite(r) or r <= rmin:
            continue
        if r < best:
            best = r
    return best if math.isfinite(best) else rmax


@dataclass
class RobotHealth:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    have_odom: bool = False
    odom_stamp_s: float = 0.0

    cmd_speed: float = 0.0
    cmd_stamp_s: float = 0.0

    scan_min: float = float('inf')
    scan_stamp_s: float = 0.0

    sample_stamp_s: float = 0.0
    sample_x: float = 0.0
    sample_y: float = 0.0
    sample_init: bool = False

    stuck_hits: int = 0
    crashed: bool = False
    state: str = "INIT"

    trail: deque[tuple[float, float]] = field(default_factory=lambda: deque(maxlen=250))


class DroneHealthDashboard(Node):
    def __init__(self) -> None:
        super().__init__('drone_health_dashboard')

        self.declare_parameter('robots', ['robot', 'robot2', 'robot3', 'robot4'])
        self.declare_parameter('map_frame', 'robot/map')
        self.declare_parameter('ui', True)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_topic', '/swarm/drone_markers')
        self.declare_parameter('pose_topic', '/swarm/drone_poses')

        # Health thresholds.
        self.declare_parameter('odom_timeout_sec', 1.5)
        self.declare_parameter('cmd_timeout_sec', 1.0)
        self.declare_parameter('cmd_speed_thresh', 0.12)
        self.declare_parameter('stuck_check_sec', 2.0)
        self.declare_parameter('stuck_min_move_m', 0.18)
        self.declare_parameter('stuck_hits', 2)
        self.declare_parameter('crash_contact_dist_m', 0.14)
        self.declare_parameter('crash_latch', True)

        raw_robots = [str(r).strip() for r in self.get_parameter('robots').value if str(r).strip()]
        robots: list[str] = []
        seen: set[str] = set()
        for r in raw_robots:
            if r in seen:
                continue
            seen.add(r)
            robots.append(r)
        self._robots = robots or ['robot']

        self._map_frame = str(self.get_parameter('map_frame').value).strip() or 'robot/map'
        self._ui_requested = _truthy(self.get_parameter('ui').value)
        self._publish_markers = _truthy(self.get_parameter('publish_markers').value)
        self._marker_topic = str(self.get_parameter('marker_topic').value).strip() or '/swarm/drone_markers'
        self._pose_topic = str(self.get_parameter('pose_topic').value).strip() or '/swarm/drone_poses'

        self._odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self._cmd_speed_thresh = float(self.get_parameter('cmd_speed_thresh').value)
        self._stuck_check = float(self.get_parameter('stuck_check_sec').value)
        self._stuck_min_move = float(self.get_parameter('stuck_min_move_m').value)
        self._stuck_hits_limit = int(self.get_parameter('stuck_hits').value)
        self._crash_contact = float(self.get_parameter('crash_contact_dist_m').value)
        self._crash_latch = _truthy(self.get_parameter('crash_latch').value)

        self._lock = threading.Lock()
        self._health: dict[str, RobotHealth] = {r: RobotHealth() for r in self._robots}

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._subs = []
        for r in self._robots:
            odom_topic = '/odom' if r == 'robot' else f'/{r}/odom'
            cmd_topic = '/cmd_vel' if r == 'robot' else f'/{r}/cmd_vel'
            scan_topic = '/scan_merged' if r == 'robot' else f'/{r}/scan_merged'

            self._subs.append(self.create_subscription(Odometry, odom_topic, self._mk_odom_cb(r), odom_qos))
            self._subs.append(self.create_subscription(Twist, cmd_topic, self._mk_cmd_cb(r), cmd_qos))
            self._subs.append(self.create_subscription(LaserScan, scan_topic, self._mk_scan_cb(r), scan_qos))

        self._markers_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self._poses_pub = self.create_publisher(PoseArray, self._pose_topic, 10)

        self._timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Drone health dashboard online (robots={self._robots}, ui={self._ui_requested}, '
            f'markers={self._publish_markers} topic={self._marker_topic}).'
        )

    def ui_requested(self) -> bool:
        return self._ui_requested and _truthy(os.environ.get('DRONE_HEALTH_UI', '1'))

    def _mk_odom_cb(self, robot: str):
        def cb(msg: Odometry) -> None:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            with self._lock:
                h = self._health[robot]
                h.x = float(p.x)
                h.y = float(p.y)
                h.yaw = float(yaw)
                h.have_odom = True
                h.odom_stamp_s = now_s
                h.trail.append((h.x, h.y))

        return cb

    def _mk_cmd_cb(self, robot: str):
        def cb(msg: Twist) -> None:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            speed = math.hypot(float(msg.linear.x), float(msg.linear.y))
            with self._lock:
                h = self._health[robot]
                h.cmd_speed = speed
                h.cmd_stamp_s = now_s

        return cb

    def _mk_scan_cb(self, robot: str):
        def cb(msg: LaserScan) -> None:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            smin = _scan_min_range(msg)
            with self._lock:
                h = self._health[robot]
                h.scan_min = float(smin)
                h.scan_stamp_s = now_s

        return cb

    def _compute_state(self, now_s: float, robot: str, h: RobotHealth) -> None:
        if not h.have_odom or now_s - h.odom_stamp_s > self._odom_timeout:
            h.state = "NO_ODOM"
            return

        cmd_speed = h.cmd_speed if now_s - h.cmd_stamp_s <= self._cmd_timeout else 0.0
        scan_min = h.scan_min if now_s - h.scan_stamp_s <= self._odom_timeout else float('inf')

        if h.crashed and self._crash_latch:
            h.state = "CRASHED"
            return

        if not h.sample_init:
            h.sample_init = True
            h.sample_stamp_s = now_s
            h.sample_x = h.x
            h.sample_y = h.y
            h.state = "OK" if cmd_speed < self._cmd_speed_thresh else "MOVING"
            return

        if now_s - h.sample_stamp_s < self._stuck_check:
            if cmd_speed >= self._cmd_speed_thresh:
                h.state = "MOVING"
            else:
                h.state = "STATIONARY"
            return

        moved = math.hypot(h.x - h.sample_x, h.y - h.sample_y)
        h.sample_stamp_s = now_s
        h.sample_x = h.x
        h.sample_y = h.y

        if cmd_speed < self._cmd_speed_thresh:
            h.stuck_hits = 0
            h.state = "STATIONARY"
            return

        if moved < self._stuck_min_move:
            h.stuck_hits += 1
        else:
            h.stuck_hits = 0
            h.state = "MOVING"
            return

        if h.stuck_hits < self._stuck_hits_limit:
            h.state = "MOVING"
            return

        h.stuck_hits = 0
        if scan_min <= self._crash_contact:
            h.crashed = True
            h.state = "CRASHED"
            self.get_logger().error(
                f'{robot}: crash detected (stuck + scan_min={scan_min:.2f}m <= {self._crash_contact:.2f}m)'
            )
            return

        h.state = "STUCK"

    def _tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            for r in self._robots:
                self._compute_state(now_s, r, self._health[r])
            if self._publish_markers:
                markers = self._build_markers(now_s)
                poses = self._build_pose_array(now_s)
            else:
                markers = None
                poses = None

        if markers is not None:
            self._markers_pub.publish(markers)
        if poses is not None:
            self._poses_pub.publish(poses)

    def _color_for_state(self, state: str) -> tuple[float, float, float, float]:
        if state == "CRASHED":
            return 0.95, 0.20, 0.20, 0.95
        if state == "STUCK":
            return 0.95, 0.75, 0.20, 0.95
        if state == "MOVING":
            return 0.20, 0.85, 0.35, 0.95
        if state == "STATIONARY":
            return 0.60, 0.60, 0.65, 0.75
        if state == "NO_ODOM":
            return 0.65, 0.35, 0.90, 0.85
        return 0.20, 0.85, 0.35, 0.95

    def _build_markers(self, now_s: float) -> MarkerArray:
        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def _mk_base(marker_type: int, *, ns: str, marker_id: int) -> Marker:
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = ns
            m.id = marker_id
            m.type = marker_type
            m.action = Marker.ADD
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            return m

        for i, r in enumerate(self._robots):
            h = self._health[r]
            if not h.have_odom:
                continue

            rgba = self._color_for_state(h.state)
            qx, qy, qz, qw = _quat_from_yaw(h.yaw)

            arrow = _mk_base(Marker.ARROW, ns="drone_pose", marker_id=i * 10 + 0)
            arrow.pose.position.x = float(h.x)
            arrow.pose.position.y = float(h.y)
            arrow.pose.position.z = 0.10
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x = 0.70
            arrow.scale.y = 0.14
            arrow.scale.z = 0.14
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = rgba
            msg.markers.append(arrow)

            text = _mk_base(Marker.TEXT_VIEW_FACING, ns="drone_label", marker_id=i * 10 + 1)
            text.pose.position.x = float(h.x)
            text.pose.position.y = float(h.y)
            text.pose.position.z = 0.55
            text.scale.z = 0.30
            text.color.r, text.color.g, text.color.b, text.color.a = rgba
            text.text = f"{r} [{h.state}]"
            msg.markers.append(text)

            trail = _mk_base(Marker.LINE_STRIP, ns="drone_trail", marker_id=i * 10 + 2)
            trail.scale.x = 0.04
            trail.color.r, trail.color.g, trail.color.b, trail.color.a = rgba[0], rgba[1], rgba[2], 0.55
            trail.points = [Point(x=float(x), y=float(y), z=0.03) for x, y in h.trail]
            msg.markers.append(trail)

        return msg

    def _build_pose_array(self, now_s: float) -> PoseArray:
        _ = now_s
        msg = PoseArray()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for r in self._robots:
            h = self._health[r]
            if not h.have_odom:
                continue
            qx, qy, qz, qw = _quat_from_yaw(h.yaw)
            p = Pose()
            p.position.x = float(h.x)
            p.position.y = float(h.y)
            p.position.z = 0.0
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            msg.poses.append(p)
        return msg

    def _snapshot(self) -> dict[str, tuple[str, float, float, float, float, float]]:
        """Return {robot: (state, x, y, yaw, cmd_speed, scan_min)}."""
        with self._lock:
            out: dict[str, tuple[str, float, float, float, float, float]] = {}
            for r in self._robots:
                h = self._health[r]
                out[r] = (h.state, h.x, h.y, h.yaw, h.cmd_speed, h.scan_min)
            return out

    def run_ui(self) -> None:
        try:
            import tkinter as tk
            from tkinter import ttk
        except Exception as exc:
            self.get_logger().warning(f'UI disabled (tkinter unavailable): {exc}')
            return

        root = tk.Tk()
        root.title("Drone Health Dashboard")
        root.geometry("760x520")

        frm = ttk.Frame(root, padding=10)
        frm.pack(fill="both", expand=True)

        cols = ("robot", "state", "x", "y", "yaw_deg", "cmd_speed", "scan_min")
        tree = ttk.Treeview(frm, columns=cols, show="headings", height=18)
        tree.heading("robot", text="Robot")
        tree.heading("state", text="State")
        tree.heading("x", text="X")
        tree.heading("y", text="Y")
        tree.heading("yaw_deg", text="Yaw (deg)")
        tree.heading("cmd_speed", text="Cmd speed")
        tree.heading("scan_min", text="Scan min")

        tree.column("robot", width=90, anchor="w")
        tree.column("state", width=110, anchor="w")
        tree.column("x", width=90, anchor="e")
        tree.column("y", width=90, anchor="e")
        tree.column("yaw_deg", width=95, anchor="e")
        tree.column("cmd_speed", width=95, anchor="e")
        tree.column("scan_min", width=95, anchor="e")

        vsb = ttk.Scrollbar(frm, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=vsb.set)
        tree.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)

        tree.tag_configure("CRASHED", background="#ffb3b3")
        tree.tag_configure("STUCK", background="#ffe9a6")
        tree.tag_configure("MOVING", background="#c6f6d5")
        tree.tag_configure("STATIONARY", background="#e5e7eb")
        tree.tag_configure("NO_ODOM", background="#e9d5ff")

        for r in self._robots:
            tree.insert("", "end", iid=r, values=(r, "INIT", "-", "-", "-", "-", "-"), tags=("NO_ODOM",))

        status_var = tk.StringVar(value="")
        status = ttk.Label(frm, textvariable=status_var)
        status.grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))

        def refresh() -> None:
            snap = self._snapshot()
            crashed = 0
            stuck = 0
            for r, (state, x, y, yaw, cmd_speed, scan_min) in snap.items():
                if state == "CRASHED":
                    crashed += 1
                elif state == "STUCK":
                    stuck += 1
                tree.item(
                    r,
                    values=(
                        r,
                        state,
                        f"{x:.2f}",
                        f"{y:.2f}",
                        f"{math.degrees(yaw):.1f}",
                        f"{cmd_speed:.2f}",
                        f"{scan_min:.2f}" if math.isfinite(scan_min) else "-",
                    ),
                    tags=(state,),
                )
            status_var.set(f"Robots: {len(snap)}  |  crashed: {crashed}  stuck: {stuck}")
            root.after(250, refresh)

        def on_close() -> None:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            root.destroy()

        root.protocol("WM_DELETE_WINDOW", on_close)
        root.after(250, refresh)
        root.mainloop()


def main() -> None:
    rclpy.init()
    node = DroneHealthDashboard()
    try:
        if node.ui_requested():
            exec_ = SingleThreadedExecutor()
            exec_.add_node(node)
            spin_thread = threading.Thread(target=exec_.spin, daemon=True)
            spin_thread.start()
            node.run_ui()
            exec_.shutdown()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['DroneHealthDashboard', 'main']
