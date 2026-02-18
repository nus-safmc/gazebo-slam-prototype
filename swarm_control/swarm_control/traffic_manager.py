#!/usr/bin/env python3
"""
Traffic Manager Node

Stub implementation for future collision avoidance and traffic coordination.
Currently a passthrough that logs proximity warnings.
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class TrafficManager(Node):
    """Traffic coordination and collision avoidance stub."""

    def __init__(self):
        super().__init__('traffic_manager')

        # Parameters
        self.declare_parameter('desired_goals_topic', '/swarm/desired_goals')
        self.declare_parameter('safe_goals_topic', '/swarm/safe_goals')
        self.declare_parameter('drone_states_topic', '/swarm/drone_states')
        self.declare_parameter('collision_distance_threshold', 1.5)
        self.declare_parameter('proximity_log_threshold', 3.0)
        self.declare_parameter('update_rate_hz', 2.0)

        # Get parameters
        self.desired_goals_topic = self.get_parameter('desired_goals_topic').value
        self.safe_goals_topic = self.get_parameter('safe_goals_topic').value
        self.drone_states_topic = self.get_parameter('drone_states_topic').value
        self.collision_threshold = self.get_parameter('collision_distance_threshold').value
        self.proximity_threshold = self.get_parameter('proximity_log_threshold').value
        self.update_rate = self.get_parameter('update_rate_hz').value

        # State
        self.drone_poses: Dict[str, Tuple[float, float, float]] = {}
        self.desired_goals: Dict[str, Tuple[float, float, float]] = {}

        # Publishers
        self.safe_goals_pub = self.create_publisher(
            PoseStamped,  # TODO: Create proper GoalArray message
            self.safe_goals_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )

        # Subscribers
        self.create_subscription(
            PoseStamped,  # TODO: Subscribe to GoalArray
            self.desired_goals_topic,
            self._desired_goals_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )

        self.create_subscription(
            String,  # TODO: Subscribe to DroneStateArray
            self.drone_states_topic,
            self._drone_states_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )

        # Timer for traffic monitoring
        self.timer = self.create_timer(1.0 / self.update_rate, self._monitor_traffic)

        self.get_logger().info(
            f'TrafficManager initialized: collision_threshold={self.collision_threshold}m, '
            f'proximity_threshold={self.proximity_threshold}m'
        )

    def _desired_goals_callback(self, msg: PoseStamped) -> None:
        """Process desired goals from goal allocator."""
        # TODO: Parse proper GoalArray message
        # Temporary: extract from PoseStamped
        robot_id = "unknown"  # TODO: Include robot ID in message
        goal = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        self.desired_goals[robot_id] = goal

    def _drone_states_callback(self, msg: String) -> None:
        """Process drone state updates."""
        # TODO: Parse proper DroneStateArray message
        # Temporary: expect "robot:fsm_state:x,y,yaw:assignment" format
        try:
            parts = msg.data.split(':')
            if len(parts) >= 3:
                robot_id = parts[0]
                pose_str = parts[2].split(',')
                if len(pose_str) >= 3:
                    pose = (float(pose_str[0]), float(pose_str[1]), float(pose_str[2]))
                    self.drone_poses[robot_id] = pose

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse drone state: {msg.data} ({e})')

    def _monitor_traffic(self) -> None:
        """Monitor drone proximity and potential collisions."""
        if len(self.drone_poses) < 2:
            return

        # Skip monitoring when all poses are still at the origin (TF not ready)
        _ORIGIN_EPS = 0.01
        valid_poses = sum(
            1 for p in self.drone_poses.values()
            if abs(p[0]) > _ORIGIN_EPS or abs(p[1]) > _ORIGIN_EPS
        )
        if valid_poses == 0:
            return

        robot_ids = list(self.drone_poses.keys())
        warnings = []
        critical = []

        for i, robot_a in enumerate(robot_ids):
            pose_a = self.drone_poses[robot_a]
            for robot_b in robot_ids[i+1:]:
                pose_b = self.drone_poses[robot_b]

                distance = math.sqrt(
                    (pose_a[0] - pose_b[0])**2 +
                    (pose_a[1] - pose_b[1])**2 +
                    (pose_a[2] - pose_b[2])**2
                )

                if distance < self.collision_threshold:
                    critical.append(f'{robot_a}-{robot_b}: {distance:.2f}m')
                elif distance < self.proximity_threshold:
                    warnings.append(f'{robot_a}-{robot_b}: {distance:.2f}m')

        if critical:
            self.get_logger().error(f'CRITICAL proximity: {", ".join(critical)}')

        if warnings:
            self.get_logger().warn(f'Close proximity: {", ".join(warnings)}')

        # Publish safe goals (currently passthrough)
        self._publish_safe_goals()

    def _publish_safe_goals(self) -> None:
        """Publish collision-free goals (currently passthrough)."""
        for robot_id, goal in self.desired_goals.items():
            # TODO: Apply traffic management logic here
            # For now, just republish as-is

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'robot/map'
            pose_msg.pose.position.x = goal[0]
            pose_msg.pose.position.y = goal[1]
            pose_msg.pose.position.z = goal[2]

            self.safe_goals_pub.publish(pose_msg)

    def get_traffic_status(self) -> Dict:
        """Get current traffic status for monitoring."""
        return {
            'drone_count': len(self.drone_poses),
            'poses': dict(self.drone_poses),
            'desired_goals': dict(self.desired_goals),
            'collision_threshold': self.collision_threshold,
            'proximity_threshold': self.proximity_threshold
        }


def main(args=None):
    rclpy.init(args=args)
    node = TrafficManager()
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