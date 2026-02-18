#!/usr/bin/env python3
"""
Frontier Server Node

Centralized frontier detection from the merged occupancy grid.
Publishes frontier targets for allocation by the goal allocator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header


@dataclass(frozen=True)
class FrontierTarget:
    """Represents a detected frontier target."""
    id: str
    centroid: tuple[float, float]  # (x, y) in map frame
    size: int  # number of cells
    timestamp: float


@dataclass(frozen=True)
class MapMeta:
    """Map metadata for coordinate transformations."""
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    @property
    def cos_o(self) -> float:
        return math.cos(self.origin_yaw)

    @property
    def sin_o(self) -> float:
        return math.sin(self.origin_yaw)

    def cell_to_world(self, ix: int, iy: int) -> tuple[float, float]:
        """Convert grid cell coordinates to world coordinates."""
        lx = (ix + 0.5) * self.resolution
        ly = (iy + 0.5) * self.resolution
        wx = self.origin_x + self.cos_o * lx - self.sin_o * ly
        wy = self.origin_y + self.sin_o * lx + self.cos_o * ly
        return wx, wy

    def world_to_cell(self, wx: float, wy: float) -> tuple[int, int]:
        """Convert world coordinates to grid cell coordinates."""
        dx = wx - self.origin_x
        dy = wy - self.origin_y
        lx = self.cos_o * dx + self.sin_o * dy
        ly = -self.sin_o * dx + self.cos_o * dy
        return int(lx / self.resolution), int(ly / self.resolution)


class FrontierServer(Node):
    """Centralized frontier detection node."""

    def __init__(self):
        super().__init__('frontier_server')

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('frontiers_topic', '/swarm/frontiers')
        self.declare_parameter('free_threshold', 30)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('min_frontier_cluster_size', 6)
        self.declare_parameter('frontier_ttl_sec', 10.0)
        self.declare_parameter('update_rate_hz', 2.0)

        # Get parameters
        self.map_topic = self.get_parameter('map_topic').value
        self.frontiers_topic = self.get_parameter('frontiers_topic').value
        self.free_thresh = self.get_parameter('free_threshold').value
        self.occupied_thresh = self.get_parameter('occupied_threshold').value
        self.min_cluster_size = self.get_parameter('min_frontier_cluster_size').value
        self.frontier_ttl = self.get_parameter('frontier_ttl_sec').value
        self.update_rate = self.get_parameter('update_rate_hz').value

        # State
        self.map: Optional[OccupancyGrid] = None
        self.frontiers: dict[str, FrontierTarget] = {}
        self.last_publish_time = 0.0

        # QoS for map subscription
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers and subscribers
        self.frontiers_pub = self.create_publisher(
            PoseStamped,  # TODO: Create custom FrontierArray message
            self.frontiers_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos
        )

        # Timer for periodic frontier detection
        self.timer = self.create_timer(1.0 / self.update_rate, self._timer_callback)

        self.get_logger().info(
            f'FrontierServer initialized: map_topic={self.map_topic}, '
            f'frontiers_topic={self.frontiers_topic}'
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Store the latest map for processing."""
        self.map = msg

    def _timer_callback(self) -> None:
        """Periodic frontier detection and publishing."""
        if self.map is None:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - getattr(self, '_last_no_map_log', 0.0) > 10.0:
                self._last_no_map_log = now_s
                self.get_logger().warn(
                    f'Waiting for map on {self.map_topic} ...'
                )
            return

        # Detect frontiers
        new_frontiers = self._detect_frontiers(self.map)

        # Update frontier set with TTL
        current_time = self.get_clock().now().nanoseconds * 1e-9
        self._update_frontiers(new_frontiers, current_time)

        # Publish frontiers periodically
        if current_time - self.last_publish_time > 0.5:  # 2 Hz publish rate
            self._publish_frontiers()
            self.last_publish_time = current_time

    def _detect_frontiers(self, map_msg: OccupancyGrid) -> list[FrontierTarget]:
        """Detect frontier targets from the occupancy grid."""
        meta = self._extract_meta(map_msg)
        if meta is None:
            return []

        size = meta.width * meta.height
        data = map_msg.data
        if len(data) < size:
            return []

        # Classify cells
        unknown = bytearray(size)
        free = bytearray(size)
        obstacle = bytearray(size)

        for i in range(size):
            v = int(data[i])
            if v == -1:
                unknown[i] = 1
            elif v <= self.free_thresh:
                free[i] = 1
            elif v >= self.occupied_thresh:
                obstacle[i] = 1

        # Find frontier cells (free cells adjacent to unknown)
        frontier = bytearray(size)
        frontier_indices = []

        neighbors_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))

        for iy in range(1, meta.height - 1):
            row = iy * meta.width
            for ix in range(1, meta.width - 1):
                idx = row + ix
                if not free[idx]:
                    continue

                # Check if adjacent to unknown
                for dx, dy in neighbors_4:
                    nidx = (iy + dy) * meta.width + (ix + dx)
                    if unknown[nidx]:
                        frontier[idx] = 1
                        frontier_indices.append(idx)
                        break

        if not frontier_indices:
            return []

        # Cluster frontier cells
        visited = bytearray(size)
        clusters = []

        for seed in frontier_indices:
            if visited[seed]:
                continue

            cluster = []
            queue = [seed]
            visited[seed] = 1

            while queue:
                current = queue.pop(0)
                cluster.append(current)
                cx = current % meta.width
                cy = current // meta.width

                # 8-connected clustering
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = cx + dx, cy + dy
                        if nx <= 0 or nx >= meta.width - 1 or ny <= 0 or ny >= meta.height - 1:
                            continue
                        nidx = ny * meta.width + nx
                        if frontier[nidx] and not visited[nidx]:
                            visited[nidx] = 1
                            queue.append(nidx)

            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)

        # Convert clusters to FrontierTarget objects
        current_time = self.get_clock().now().nanoseconds * 1e-9
        targets = []

        for cluster in clusters:
            # Compute centroid
            sx, sy = 0.0, 0.0
            for cell_idx in cluster:
                sx += cell_idx % meta.width
                sy += cell_idx // meta.width

            cx = sx / len(cluster)
            cy = sy / len(cluster)

            # Convert to world coordinates
            wx, wy = meta.cell_to_world(int(cx), int(cy))

            # Generate stable ID based on centroid
            frontier_id = f"f_{int(cx)}_{int(cy)}"

            target = FrontierTarget(
                id=frontier_id,
                centroid=(wx, wy),
                size=len(cluster),
                timestamp=current_time
            )
            targets.append(target)

        return targets

    def _update_frontiers(self, new_frontiers: list[FrontierTarget], current_time: float) -> None:
        """Update the frontier set with TTL management."""
        # Mark existing frontiers as seen
        seen_ids = {f.id for f in new_frontiers}

        # Remove expired frontiers
        expired = []
        for fid, frontier in self.frontiers.items():
            if current_time - frontier.timestamp > self.frontier_ttl:
                expired.append(fid)

        for fid in expired:
            del self.frontiers[fid]

        # Add/update new frontiers
        for frontier in new_frontiers:
            self.frontiers[frontier.id] = frontier

        # Log changes
        if expired or new_frontiers:
            self.get_logger().info(
                f'Frontiers updated: {len(new_frontiers)} new, {len(expired)} expired, '
                f'total={len(self.frontiers)}'
            )

    def _publish_frontiers(self) -> None:
        """Publish current frontiers."""
        # TODO: Create proper FrontierArray message type
        # For now, publish as PoseStamped array (one message per frontier)
        for frontier in self.frontiers.values():
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'robot/map'  # Map frame

            pose_msg.pose.position.x = frontier.centroid[0]
            pose_msg.pose.position.y = frontier.centroid[1]
            pose_msg.pose.position.z = 0.0

            # Use orientation to encode size (temporary hack)
            pose_msg.pose.orientation.w = float(frontier.size)

            self.frontiers_pub.publish(pose_msg)

    def _extract_meta(self, msg: OccupancyGrid) -> Optional[MapMeta]:
        """Extract map metadata."""
        info = msg.info
        width = int(info.width)
        height = int(info.height)
        res = float(info.resolution)

        if width <= 0 or height <= 0 or res <= 0.0:
            return None

        o = info.origin
        yaw = math.atan2(2.0 * (o.orientation.w * o.orientation.z + o.orientation.x * o.orientation.y),
                        1.0 - 2.0 * (o.orientation.y * o.orientation.y + o.orientation.z * o.orientation.z))

        return MapMeta(
            width=width,
            height=height,
            resolution=res,
            origin_x=float(o.position.x),
            origin_y=float(o.position.y),
            origin_yaw=yaw,
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()