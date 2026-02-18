#!/usr/bin/env python3
"""
Drone Executor Node

Per-drone FSM that consumes centralized assignments and executes them via Nav2.
Maintains drone state and reports status to the swarm system.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
from rclpy.time import Time


class DroneState(Enum):
    BOOT = "BOOT"
    PREFLIGHT = "PREFLIGHT"
    ARMED = "ARMED"
    TAKING_OFF = "TAKING_OFF"
    STAGING = "STAGING"
    AVAILABLE = "AVAILABLE"
    EXECUTING_GOAL = "EXECUTING_GOAL"
    RECOVERY = "RECOVERY"
    RETURNING = "RETURNING"
    LANDING = "LANDING"
    LANDED = "LANDED"
    EMERGENCY = "EMERGENCY"


@dataclass
class Assignment:
    frontier_id: str
    target_pose: tuple[float, float]  # (x, y)
    timestamp: float


class DroneExecutor(Node):
    """Per-drone execution FSM with Nav2 integration."""

    def __init__(self):
        super().__init__('drone_executor')

        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('assignments_topic', '/swarm/assignments')
        self.declare_parameter('drone_states_topic', '/swarm/drone_states')
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('recovery_timeout_sec', 15.0)
        self.declare_parameter('cruise_altitude', 1.2)
        self.declare_parameter('goal_frame', 'robot/map')

        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.assignments_topic = self.get_parameter('assignments_topic').value
        self.drone_states_topic = self.get_parameter('drone_states_topic').value
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value
        self.recovery_timeout = self.get_parameter('recovery_timeout_sec').value
        self.cruise_altitude = self.get_parameter('cruise_altitude').value
        self.goal_frame = self.get_parameter('goal_frame').value

        self.robot_name = self.robot_namespace.strip('/') or 'robot'

        # FSM state
        self.current_state = DroneState.BOOT
        self.current_assignment: Optional[Assignment] = None
        self.goal_handle = None
        self.result_future = None
        self.state_start_time = self.get_clock().now()
        self.last_pose: Optional[tuple[float, float, float]] = None
        self.consecutive_failures = 0
        self._nav2_ready = False

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client (resolves per-namespace, e.g. /robot2/navigate_to_pose)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.drone_state_pub = self.create_publisher(
            String,
            self.drone_states_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
        )

        # Subscribers
        self.create_subscription(
            String,
            self.assignments_topic,
            self._assignment_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
        )

        self.timer = self.create_timer(0.5, self._fsm_tick)

        self.get_logger().info(
            f'DroneExecutor initialized: robot={self.robot_name}, '
            f'namespace={self.robot_namespace}'
        )

    # ── Assignment handling ──────────────────────────────────────────

    def _assignment_callback(self, msg: String) -> None:
        """Parse assignment: 'robot_id:frontier_id:cx:cy'"""
        try:
            parts = msg.data.split(':')
            if len(parts) < 4:
                return
            assigned_robot = parts[0]
            if assigned_robot != self.robot_name:
                return

            frontier_id = parts[1]
            cx = float(parts[2])
            cy = float(parts[3])

            if self.current_state != DroneState.AVAILABLE:
                self.get_logger().debug(
                    f'Ignoring assignment {frontier_id} (state={self.current_state.value})'
                )
                return

            self.current_assignment = Assignment(
                frontier_id=frontier_id,
                target_pose=(cx, cy),
                timestamp=self.get_clock().now().nanoseconds * 1e-9,
            )
            self.get_logger().info(f'Accepted assignment: {frontier_id} at ({cx:.1f}, {cy:.1f})')

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse assignment: {msg.data} ({e})')

    # ── FSM tick ─────────────────────────────────────────────────────

    def _fsm_tick(self) -> None:
        self._update_pose()

        handler = {
            DroneState.BOOT: self._handle_boot,
            DroneState.PREFLIGHT: self._handle_preflight,
            DroneState.ARMED: self._handle_armed,
            DroneState.TAKING_OFF: self._handle_taking_off,
            DroneState.STAGING: self._handle_staging,
            DroneState.AVAILABLE: self._handle_available,
            DroneState.EXECUTING_GOAL: self._handle_executing_goal,
            DroneState.RECOVERY: self._handle_recovery,
            DroneState.RETURNING: self._handle_returning,
            DroneState.LANDING: self._handle_landing,
            DroneState.LANDED: self._handle_landed,
            DroneState.EMERGENCY: self._handle_emergency,
        }.get(self.current_state)

        if handler:
            handler()

        self._publish_state()

    # ── Pose ─────────────────────────────────────────────────────────

    def _update_pose(self) -> None:
        try:
            t = self.tf_buffer.lookup_transform(
                self.goal_frame,
                f'{self.robot_name}/base_link',
                Time(),
                timeout=Duration(seconds=0.1),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.last_pose = (x, y, yaw)
        except TransformException:
            pass

    # ── State handlers ───────────────────────────────────────────────

    def _handle_boot(self) -> None:
        """Wait for Nav2 action server."""
        if not self._nav2_ready:
            self._nav2_ready = self.nav_client.wait_for_server(timeout_sec=0.0)

        if self._nav2_ready:
            self._transition_to(DroneState.PREFLIGHT)
            return

        elapsed = self._elapsed_in_state()
        if elapsed > 60.0:
            self.get_logger().error('Nav2 not ready after 60 s, entering EMERGENCY')
            self._transition_to(DroneState.EMERGENCY)

    def _handle_preflight(self) -> None:
        self._transition_to(DroneState.ARMED)

    def _handle_armed(self) -> None:
        if self._elapsed_in_state() > 2.0:
            self._transition_to(DroneState.TAKING_OFF)

    def _handle_taking_off(self) -> None:
        """In simulation, skip physical takeoff and go straight to staging."""
        self._transition_to(DroneState.STAGING)

    def _handle_staging(self) -> None:
        """Brief pause before becoming available for assignments."""
        if self._elapsed_in_state() > 1.0:
            self._transition_to(DroneState.AVAILABLE)

    def _handle_available(self) -> None:
        if self.current_assignment is not None:
            cx, cy = self.current_assignment.target_pose
            self._send_nav_goal(cx, cy)
            self._transition_to(DroneState.EXECUTING_GOAL)

    def _handle_executing_goal(self) -> None:
        if self._is_nav_idle():
            self.get_logger().info(
                f'Goal reached: {self.current_assignment.frontier_id}'
            )
            self.current_assignment = None
            self.consecutive_failures = 0
            self._transition_to(DroneState.AVAILABLE)
            return

        if self._elapsed_in_state() > self.goal_timeout:
            self.get_logger().warn(f'Goal timeout after {self.goal_timeout:.0f}s')
            self._cancel_nav_goal()
            self._transition_to(DroneState.RECOVERY)

    def _handle_recovery(self) -> None:
        if self._elapsed_in_state() > self.recovery_timeout:
            self.consecutive_failures += 1
            if self.consecutive_failures >= 3:
                self.get_logger().error('Too many failures, entering EMERGENCY')
                self._transition_to(DroneState.EMERGENCY)
            else:
                self.get_logger().info('Recovery done, returning to AVAILABLE')
                self.current_assignment = None
                self._transition_to(DroneState.AVAILABLE)

    def _handle_returning(self) -> None:
        pass

    def _handle_landing(self) -> None:
        pass

    def _handle_landed(self) -> None:
        pass

    def _handle_emergency(self) -> None:
        pass

    # ── Nav2 helpers ─────────────────────────────────────────────────

    def _send_nav_goal(self, gx: float, gy: float) -> None:
        """Send a NavigateToPose goal in the shared map frame."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0

        # Face toward the goal from current position
        if self.last_pose:
            yaw = math.atan2(gy - self.last_pose[1], gx - self.last_pose[0])
        else:
            yaw = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending nav goal: ({gx:.2f}, {gy:.2f})')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Nav2 rejected goal')
                self._transition_to(DroneState.RECOVERY)
                return
            self.goal_handle = goal_handle
            self.result_future = goal_handle.get_result_async()
            self.result_future.add_done_callback(self._goal_result_callback)
        except Exception as e:
            self.get_logger().error(f'Goal response error: {e}')
            self._transition_to(DroneState.RECOVERY)

    def _goal_result_callback(self, future) -> None:
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded')
            elif result.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn('Navigation aborted by Nav2')
                self._transition_to(DroneState.RECOVERY)
            elif result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Navigation goal canceled')
            else:
                self.get_logger().info(f'Navigation result status={result.status}')
        except Exception as e:
            self.get_logger().error(f'Goal result error: {e}')
            self._transition_to(DroneState.RECOVERY)
        finally:
            self.goal_handle = None
            self.result_future = None

    def _cancel_nav_goal(self) -> None:
        if self.goal_handle is not None:
            self.get_logger().info('Canceling active Nav2 goal')
            self.goal_handle.cancel_goal_async()

    def _is_nav_idle(self) -> bool:
        return self.goal_handle is None and self.result_future is None

    # ── Utilities ────────────────────────────────────────────────────

    def _elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self.state_start_time).nanoseconds * 1e-9

    def _transition_to(self, new_state: DroneState) -> None:
        if self.current_state != new_state:
            self.get_logger().info(
                f'State: {self.current_state.value} -> {new_state.value}'
            )
            self.current_state = new_state
            self.state_start_time = self.get_clock().now()

    def _publish_state(self) -> None:
        """Publish 'robot:STATE:x,y,yaw:frontier_id'"""
        pose_str = "0.00,0.00,0.00"
        if self.last_pose:
            pose_str = f"{self.last_pose[0]:.2f},{self.last_pose[1]:.2f},{self.last_pose[2]:.2f}"
        assignment_str = self.current_assignment.frontier_id if self.current_assignment else "None"
        msg = String()
        msg.data = f"{self.robot_name}:{self.current_state.value}:{pose_str}:{assignment_str}"
        self.drone_state_pub.publish(msg)

    def destroy_node(self):
        """Cancel any active Nav2 goal before shutdown."""
        self._cancel_nav_goal()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneExecutor()
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
