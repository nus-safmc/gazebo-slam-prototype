#!/usr/bin/env python3
"""
Goal Allocator Node

Assigns frontier targets to available drones using a greedy assignment algorithm.
Subscribes to frontiers and drone states, publishes assignments with coordinates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


@dataclass(frozen=True)
class FrontierTarget:
    """Frontier target data."""
    id: str
    centroid: tuple[float, float]
    size: int
    timestamp: float


@dataclass(frozen=True)
class DroneState:
    """Drone state information."""
    robot_id: str
    pose: tuple[float, float, float]  # (x, y, yaw)
    fsm_state: str
    current_assignment: Optional[str]
    timestamp: float


@dataclass(frozen=True)
class Assignment:
    """Drone to frontier assignment."""
    drone_id: str
    frontier_id: str
    centroid: tuple[float, float]
    timestamp: float


class GoalAllocator(Node):
    """Centralized goal assignment node."""

    def __init__(self):
        super().__init__('goal_allocator')

        self.declare_parameter('frontiers_topic', '/swarm/frontiers')
        self.declare_parameter('assignments_topic', '/swarm/assignments')
        self.declare_parameter('drone_states_topic', '/swarm/drone_states')
        self.declare_parameter('gain_weight', 1.0)
        self.declare_parameter('cost_weight', 0.35)
        self.declare_parameter('assignment_timeout_sec', 30.0)
        self.declare_parameter('update_rate_hz', 2.0)

        self.frontiers_topic = self.get_parameter('frontiers_topic').value
        self.assignments_topic = self.get_parameter('assignments_topic').value
        self.drone_states_topic = self.get_parameter('drone_states_topic').value
        self.gain_weight = self.get_parameter('gain_weight').value
        self.cost_weight = self.get_parameter('cost_weight').value
        self.assignment_timeout = self.get_parameter('assignment_timeout_sec').value
        self.update_rate = self.get_parameter('update_rate_hz').value

        self.frontiers: dict[str, FrontierTarget] = {}
        self.drone_states: dict[str, DroneState] = {}
        self.assignments: dict[str, Assignment] = {}  # drone_id -> assignment
        self.last_update_time = 0.0

        self.assignments_pub = self.create_publisher(
            String,
            self.assignments_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
        )

        self.create_subscription(
            PoseStamped,
            self.frontiers_topic,
            self._frontiers_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
        )

        self.create_subscription(
            String,
            self.drone_states_topic,
            self._drone_states_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
        )

        self.timer = self.create_timer(1.0 / self.update_rate, self._timer_callback)

        self.get_logger().info(
            f'GoalAllocator initialized: frontiers_topic={self.frontiers_topic}'
        )

    def _frontiers_callback(self, msg: PoseStamped) -> None:
        """Process incoming frontier data."""
        frontier_id = f"f_{int(msg.pose.position.x)}_{int(msg.pose.position.y)}"
        centroid = (msg.pose.position.x, msg.pose.position.y)
        size = int(msg.pose.orientation.w)

        current_time = self.get_clock().now().nanoseconds * 1e-9
        frontier = FrontierTarget(
            id=frontier_id,
            centroid=centroid,
            size=size,
            timestamp=current_time,
        )
        self.frontiers[frontier.id] = frontier

    def _drone_states_callback(self, msg: String) -> None:
        """Process incoming drone state data.

        Expected format: "robot:STATE:x,y,yaw:frontier_id_or_None"
        """
        try:
            parts = msg.data.split(':')
            if len(parts) >= 4:
                robot_id = parts[0]
                fsm_state = parts[1]
                pose_str = parts[2].split(',')
                pose = (float(pose_str[0]), float(pose_str[1]), float(pose_str[2]))
                current_assignment = parts[3] if parts[3] != 'None' else None

                current_time = self.get_clock().now().nanoseconds * 1e-9
                self.drone_states[robot_id] = DroneState(
                    robot_id=robot_id,
                    pose=pose,
                    fsm_state=fsm_state,
                    current_assignment=current_assignment,
                    timestamp=current_time,
                )
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse drone state: {msg.data} ({e})')

    def _timer_callback(self) -> None:
        """Periodic goal allocation."""
        current_time = self.get_clock().now().nanoseconds * 1e-9

        expired = [
            did for did, a in self.assignments.items()
            if current_time - a.timestamp > self.assignment_timeout
        ]
        for did in expired:
            self.get_logger().info(f'Assignment expired for {did}')
            del self.assignments[did]

        new_assignments = self._compute_assignments()
        for assignment in new_assignments:
            self.assignments[assignment.drone_id] = assignment

        self._publish_assignments(new_assignments)

        if current_time - self.last_update_time > 5.0:
            self._log_status()
            self.last_update_time = current_time

    def _compute_assignments(self) -> list[Assignment]:
        """Compute optimal drone-to-frontier assignments using greedy algorithm."""
        available_drones = [
            d for d in self.drone_states.values()
            if d.fsm_state == 'AVAILABLE' and d.robot_id not in self.assignments
        ]

        assigned_frontiers = {a.frontier_id for a in self.assignments.values()}
        available_frontiers = [
            f for f in self.frontiers.values()
            if f.id not in assigned_frontiers
        ]

        if not available_drones or not available_frontiers:
            return []

        assignments: list[Assignment] = []
        remaining_drones = list(available_drones)
        remaining_frontiers = list(available_frontiers)

        while remaining_drones and remaining_frontiers:
            best_score = -float('inf')
            best_pair = None

            for drone in remaining_drones:
                for frontier in remaining_frontiers:
                    score = self._compute_assignment_score(drone, frontier)
                    if score > best_score:
                        best_score = score
                        best_pair = (drone, frontier)

            if best_pair is None:
                break

            drone, frontier = best_pair
            current_time = self.get_clock().now().nanoseconds * 1e-9
            assignments.append(Assignment(
                drone_id=drone.robot_id,
                frontier_id=frontier.id,
                centroid=frontier.centroid,
                timestamp=current_time,
            ))

            remaining_drones.remove(drone)
            remaining_frontiers.remove(frontier)

        return assignments

    def _compute_assignment_score(self, drone: DroneState, frontier: FrontierTarget) -> float:
        """Compute assignment score (higher is better)."""
        dx = frontier.centroid[0] - drone.pose[0]
        dy = frontier.centroid[1] - drone.pose[1]
        distance = math.sqrt(dx * dx + dy * dy)
        return self.gain_weight * float(frontier.size) - self.cost_weight * distance

    def _publish_assignments(self, new_assignments: list[Assignment]) -> None:
        """Publish newly computed assignments with coordinates."""
        for assignment in new_assignments:
            msg = String()
            cx, cy = assignment.centroid
            msg.data = f"{assignment.drone_id}:{assignment.frontier_id}:{cx:.3f}:{cy:.3f}"
            self.assignments_pub.publish(msg)
            self.get_logger().info(
                f'Assigned {assignment.drone_id} -> {assignment.frontier_id} '
                f'at ({cx:.1f}, {cy:.1f})'
            )

    def _log_status(self) -> None:
        """Log current allocation status."""
        self.get_logger().info(
            f'Status: {len(self.drone_states)} drones, '
            f'{len(self.frontiers)} frontiers, {len(self.assignments)} assignments'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalAllocator()
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
