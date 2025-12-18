#!/usr/bin/env python3
from __future__ import annotations

import math
from array import array
from dataclasses import dataclass
from typing import Iterable, Optional

import rclpy
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _bresenham(ix0: int, iy0: int, ix1: int, iy1: int) -> Iterable[tuple[int, int]]:
    dx = abs(ix1 - ix0)
    dy = abs(iy1 - iy0)
    sx = 1 if ix0 < ix1 else -1
    sy = 1 if iy0 < iy1 else -1
    err = dx - dy
    x = ix0
    y = iy0
    while True:
        yield x, y
        if x == ix1 and y == iy1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


@dataclass(frozen=True)
class _MapSpec:
    res: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    width: int
    height: int


class SwarmMapFuser(Node):
    """Build a single global OccupancyGrid by raytracing multiple LaserScans.

    This is intended for multi-robot mapping in simulation where robot odometry
    is effectively ground truth. It avoids SLAM drift and keeps RViz topics
    compatible by publishing `/map` and `/map_updates`.
    """

    def __init__(self) -> None:
        super().__init__('swarm_map_fuser')

        self.declare_parameter('map_frame', 'robot/map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('update_topic', '/map_updates')
        self.declare_parameter(
            'robots',
            [''],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            'scan_topics',
            [
                '/scan_merged',
                '/robot2/scan_merged',
                '/robot3/scan_merged',
                '/robot4/scan_merged',
            ],
        )
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('min_x', -10.0)
        self.declare_parameter('max_x', 10.0)
        self.declare_parameter('min_y', -10.0)
        self.declare_parameter('max_y', 10.0)
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('seed_keepout', True)
        self.declare_parameter('keepout_margin_m', 0.2)
        self.declare_parameter('filter_robot_radius_m', 0.35)
        self.declare_parameter('free_update', -1)
        self.declare_parameter('occ_update', 4)
        self.declare_parameter('min_log_odds', -25)
        self.declare_parameter('max_log_odds', 25)
        self.declare_parameter('free_threshold', -2)
        self.declare_parameter('occ_threshold', 6)
        self.declare_parameter('max_range_override', 0.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.update_topic = str(self.get_parameter('update_topic').value)
        scan_topics = list(self.get_parameter('scan_topics').value)

        robots = [str(r).strip() for r in self.get_parameter('robots').value if str(r).strip()]
        if not robots:
            derived: set[str] = set()
            for t in scan_topics:
                topic = str(t).strip()
                if not topic:
                    continue
                if topic.startswith('/robot'):
                    parts = topic.strip('/').split('/', 1)
                    if parts and parts[0].startswith('robot'):
                        derived.add(parts[0])
                else:
                    derived.add('robot')
            robots = sorted(derived) if derived else ['robot']
        self._robots = list(robots)
        self._robot_base_frames = {r: f'{r}/base_footprint' for r in self._robots}

        self._filter_robot_radius_m = float(self.get_parameter('filter_robot_radius_m').value)
        if self._filter_robot_radius_m < 0.0:
            self._filter_robot_radius_m = 0.0

        res = float(self.get_parameter('resolution').value)
        min_x = float(self.get_parameter('min_x').value)
        max_x = float(self.get_parameter('max_x').value)
        min_y = float(self.get_parameter('min_y').value)
        max_y = float(self.get_parameter('max_y').value)
        if res <= 0.0:
            raise ValueError('resolution must be > 0')
        if max_x <= min_x or max_y <= min_y:
            raise ValueError('invalid map bounds')
        width = int(math.ceil((max_x - min_x) / res))
        height = int(math.ceil((max_y - min_y) / res))
        self._spec = _MapSpec(
            res=res,
            min_x=min_x,
            max_x=max_x,
            min_y=min_y,
            max_y=max_y,
            width=width,
            height=height,
        )

        self._free_update = int(self.get_parameter('free_update').value)
        self._occ_update = int(self.get_parameter('occ_update').value)
        self._min_lo = int(self.get_parameter('min_log_odds').value)
        self._max_lo = int(self.get_parameter('max_log_odds').value)
        self._free_thresh = int(self.get_parameter('free_threshold').value)
        self._occ_thresh = int(self.get_parameter('occ_threshold').value)
        self._max_range_override = float(self.get_parameter('max_range_override').value)

        size = self._spec.width * self._spec.height
        self._log_odds = array('h', [0]) * size
        self._observed = bytearray(size)
        # Cells that are permanently occupied (e.g. perimeter keepout band).
        # This prevents the raytracer from accidentally "clearing" the border.
        self._keepout = bytearray(size)
        self._dirty = False
        self._published_once = False

        seed_keepout = bool(self.get_parameter('seed_keepout').value)
        keepout_margin = float(self.get_parameter('keepout_margin_m').value)
        if seed_keepout and keepout_margin > 0.0:
            self._seed_keepout(margin_m=keepout_margin)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)

        upd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._update_pub = self.create_publisher(OccupancyGridUpdate, self.update_topic, upd_qos)

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
        )
        for t in scan_topics:
            topic = str(t).strip()
            if not topic:
                continue
            self.create_subscription(LaserScan, topic, self._mk_scan_cb(topic), scan_qos)

        period = float(self.get_parameter('publish_period_sec').value)
        period = 0.5 if period <= 0.0 else period
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Global map fuser online: frame={self.map_frame} grid={width}x{height} '
            f'res={res:.3f}m scans={len(scan_topics)} robots={robots}'
        )

    def _seed_keepout(self, *, margin_m: float) -> None:
        keep_min_x = self._spec.min_x + margin_m
        keep_max_x = self._spec.max_x - margin_m
        keep_min_y = self._spec.min_y + margin_m
        keep_max_y = self._spec.max_y - margin_m
        if keep_max_x <= keep_min_x or keep_max_y <= keep_min_y:
            self.get_logger().warning('Keepout margin is too large; skipping keepout seeding.')
            return

        occ = self._max_lo
        seeded = 0
        for iy in range(self._spec.height):
            y = self._spec.min_y + (iy + 0.5) * self._spec.res
            out_y = y < keep_min_y or y > keep_max_y
            row = iy * self._spec.width
            for ix in range(self._spec.width):
                x = self._spec.min_x + (ix + 0.5) * self._spec.res
                if out_y or x < keep_min_x or x > keep_max_x:
                    idx = row + ix
                    self._observed[idx] = 1
                    self._log_odds[idx] = occ
                    self._keepout[idx] = 1
                    seeded += 1

        self._dirty = True
        self.get_logger().info(
            f'Seeded keepout border: margin={margin_m:.2f}m '
            f'keepout=({keep_min_x:.2f},{keep_max_x:.2f})x({keep_min_y:.2f},{keep_max_y:.2f}) '
            f'cells={seeded}'
        )

    def _idx(self, ix: int, iy: int) -> int:
        return iy * self._spec.width + ix

    def _in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self._spec.width and 0 <= iy < self._spec.height

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        ix = int((x - self._spec.min_x) / self._spec.res)
        iy = int((y - self._spec.min_y) / self._spec.res)
        if ix < 0:
            ix = 0
        elif ix >= self._spec.width:
            ix = self._spec.width - 1
        if iy < 0:
            iy = 0
        elif iy >= self._spec.height:
            iy = self._spec.height - 1
        return ix, iy

    def _ray_exit_distance(self, ox: float, oy: float, dx: float, dy: float) -> float:
        # Distance until ray exits the axis-aligned map bounds.
        tx = math.inf
        if abs(dx) > 1e-12:
            if dx > 0.0:
                tx = (self._spec.max_x - ox) / dx
            else:
                tx = (self._spec.min_x - ox) / dx
        ty = math.inf
        if abs(dy) > 1e-12:
            if dy > 0.0:
                ty = (self._spec.max_y - oy) / dy
            else:
                ty = (self._spec.min_y - oy) / dy
        return max(0.0, min(tx, ty))

    def _apply_update(self, ix: int, iy: int, delta: int) -> None:
        if not self._in_bounds(ix, iy):
            return
        idx = self._idx(ix, iy)
        if self._keepout[idx]:
            return
        self._observed[idx] = 1
        v = int(self._log_odds[idx]) + int(delta)
        if v < self._min_lo:
            v = self._min_lo
        elif v > self._max_lo:
            v = self._max_lo
        self._log_odds[idx] = v

    def _mk_scan_cb(self, topic: str):
        def _cb(msg: LaserScan) -> None:
            frame = str(msg.header.frame_id).strip()
            if not frame:
                return

            try:
                tf = self._tf_buffer.lookup_transform(
                    self.map_frame,
                    frame,
                    Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException:
                return

            ox = float(tf.transform.translation.x)
            oy = float(tf.transform.translation.y)
            q = tf.transform.rotation
            base_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

            if not (self._spec.min_x <= ox <= self._spec.max_x and self._spec.min_y <= oy <= self._spec.max_y):
                return

            start_ix, start_iy = self._world_to_cell(ox, oy)

            max_range = float(msg.range_max)
            if self._max_range_override > 0.0:
                max_range = self._max_range_override
            max_range = max(0.1, max_range)

            # Filter out OTHER robots so they don't get fused into the static map.
            # Do not filter the scanning robot itself; otherwise nearby walls/pillars
            # disappear right when we need them most (leading to collisions).
            scan_robot = frame.split('/', 1)[0] if '/' in frame else ''
            robot_xy: list[tuple[float, float]] = []
            if self._filter_robot_radius_m > 0.0 and self._robot_base_frames:
                stamp = Time.from_msg(msg.header.stamp)
                for name, base in self._robot_base_frames.items():
                    if name == scan_robot:
                        continue
                    try:
                        rtf = self._tf_buffer.lookup_transform(
                            self.map_frame,
                            base,
                            stamp,
                            timeout=Duration(seconds=0.05),
                        )
                    except TransformException:
                        continue
                    robot_xy.append((float(rtf.transform.translation.x), float(rtf.transform.translation.y)))
            rr2 = self._filter_robot_radius_m * self._filter_robot_radius_m

            a = float(msg.angle_min)
            inc = float(msg.angle_increment) if msg.angle_increment != 0.0 else 0.0
            if inc == 0.0:
                return

            # Update map by raytracing each beam.
            for i, r in enumerate(msg.ranges):
                if r is None or math.isnan(r):
                    continue

                ang = base_yaw + a + float(i) * inc
                dx = math.cos(ang)
                dy = math.sin(ang)

                r_in = float(r)
                if not math.isfinite(r_in) or r_in <= 0.0:
                    r_use = max_range
                    hit = False
                else:
                    r_use = min(r_in, max_range)
                    # Treat "max range" returns as no-hit.
                    hit = r_use < max_range * 0.995

                # Clip to map bounds along the ray.
                exit_dist = self._ray_exit_distance(ox, oy, dx, dy)
                if exit_dist <= 0.0:
                    continue
                if r_use > exit_dist:
                    r_use = exit_dist
                    hit = False

                ex = ox + dx * r_use
                ey = oy + dy * r_use
                end_ix, end_iy = self._world_to_cell(ex, ey)

                # Don't fuse other robots into the static map (it causes "start occupied"
                # and can wedge planners/controllers). Treat hits near robot bodies as
                # non-occupied endpoints: raytrace free up to that point but don't mark it.
                if hit and rr2 > 0.0 and robot_xy:
                    for rx, ry in robot_xy:
                        ddx = ex - rx
                        ddy = ey - ry
                        if (ddx * ddx + ddy * ddy) <= rr2:
                            hit = False
                            break

                # Avoid marking the sensor origin cell as occupied when a hit falls within
                # the same grid cell as the robot. This can happen with very short ranges
                # (e.g. after a collision) and will cause Nav2 "Start occupied" failures.
                if end_ix == start_ix and end_iy == start_iy:
                    self._apply_update(start_ix, start_iy, self._free_update)
                    if hit:
                        step_ix = start_ix + (1 if dx > 0.0 else (-1 if dx < 0.0 else 0))
                        step_iy = start_iy + (1 if dy > 0.0 else (-1 if dy < 0.0 else 0))
                        if step_ix != start_ix or step_iy != start_iy:
                            self._apply_update(step_ix, step_iy, self._occ_update)
                    continue

                # Free space along the ray (excluding the endpoint).
                last = None
                for cell in _bresenham(start_ix, start_iy, end_ix, end_iy):
                    last = cell
                    if cell == (end_ix, end_iy):
                        break
                    self._apply_update(cell[0], cell[1], self._free_update)
                if hit and last is not None:
                    self._apply_update(last[0], last[1], self._occ_update)

            self._dirty = True

        return _cb

    def _build_map(self) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = float(self._spec.res)
        msg.info.width = int(self._spec.width)
        msg.info.height = int(self._spec.height)
        msg.info.origin.position.x = float(self._spec.min_x)
        msg.info.origin.position.y = float(self._spec.min_y)
        msg.info.origin.orientation.w = 1.0

        data: list[int] = [0] * (self._spec.width * self._spec.height)
        for i in range(len(data)):
            if self._keepout[i]:
                data[i] = 100
                continue
            if not self._observed[i]:
                data[i] = -1
                continue
            lo = int(self._log_odds[i])
            # Prefer a fast, coarse map for navigation: once a cell is observed,
            # treat it as free unless we have strong evidence it's occupied.
            data[i] = 100 if lo >= self._occ_thresh else 0
        msg.data = data
        return msg

    def _publish(self) -> None:
        if not self._dirty and self._published_once:
            return

        grid = self._build_map()
        self._map_pub.publish(grid)
        self._published_once = True

        upd = OccupancyGridUpdate()
        upd.header = grid.header
        upd.x = 0
        upd.y = 0
        upd.width = grid.info.width
        upd.height = grid.info.height
        upd.data = grid.data
        self._update_pub.publish(upd)

        self._dirty = False


def main() -> None:
    rclpy.init()
    node = SwarmMapFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['SwarmMapFuser', 'main']
