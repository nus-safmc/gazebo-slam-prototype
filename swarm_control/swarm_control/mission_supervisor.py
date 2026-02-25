#!/usr/bin/env python3
"""
Mission Supervisor Node

Global FSM for mission orchestration and takeoff window management.
Coordinates the overall swarm mission state.
"""

from __future__ import annotations

from enum import Enum
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String


class MissionState(Enum):
    IDLE = "IDLE"
    READY = "READY"
    TAKEOFF_WINDOW_A = "TAKEOFF_WINDOW_A"
    TAKEOFF_WINDOW_B = "TAKEOFF_WINDOW_B"
    RUNNING = "RUNNING"
    RETURN_AND_LAND = "RETURN_AND_LAND"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


class MissionSupervisor(Node):
    """Global mission coordinator."""

    def __init__(self):
        super().__init__('mission_supervisor')

        self.declare_parameter('mission_state_topic', '/swarm/mission_state')
        self.declare_parameter('drone_states_topic', '/swarm/drone_states')
        self.declare_parameter('expected_drones', ['robot', 'robot2', 'robot3', 'robot4', 'robot5', 'robot6', 'robot7', 'robot8'])
        self.declare_parameter('takeoff_window_duration_sec', 30.0)
        self.declare_parameter('auto_start_mission', True)

        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.drone_states_topic = self.get_parameter('drone_states_topic').value
        self.expected_drones = self.get_parameter('expected_drones').value
        self.takeoff_window_duration = self.get_parameter('takeoff_window_duration_sec').value
        self.auto_start = self.get_parameter('auto_start_mission').value

        self.current_state = MissionState.IDLE
        self.state_start_time = self.get_clock().now()
        self.drone_states: Dict[str, str] = {}
        self.takeoff_groups: Dict[str, List[str]] = {}

        self.mission_state_pub = self.create_publisher(
            String,
            self.mission_state_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1),
        )

        self.create_subscription(
            String,
            self.drone_states_topic,
            self._drone_states_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
        )

        self.timer = self.create_timer(1.0, self._fsm_tick)
        self._initialize_takeoff_groups()

        self.get_logger().info(
            f'MissionSupervisor initialized: expected_drones={len(self.expected_drones)}'
        )

    def _initialize_takeoff_groups(self) -> None:
        mid = len(self.expected_drones) // 2
        self.takeoff_groups['A'] = self.expected_drones[:mid]
        self.takeoff_groups['B'] = self.expected_drones[mid:]
        self.get_logger().info(
            f'Takeoff groups: A={self.takeoff_groups["A"]}, B={self.takeoff_groups["B"]}'
        )

    def _drone_states_callback(self, msg: String) -> None:
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2:
                self.drone_states[parts[0]] = parts[1]
        except ValueError as e:
            self.get_logger().warn(f'Failed to parse drone state: {msg.data} ({e})')

    # ── FSM ──────────────────────────────────────────────────────────

    def _fsm_tick(self) -> None:
        handler = {
            MissionState.IDLE: self._handle_idle,
            MissionState.READY: self._handle_ready,
            MissionState.TAKEOFF_WINDOW_A: lambda: self._handle_takeoff_window('A'),
            MissionState.TAKEOFF_WINDOW_B: lambda: self._handle_takeoff_window('B'),
            MissionState.RUNNING: self._handle_running,
            MissionState.RETURN_AND_LAND: self._handle_return_and_land,
            MissionState.COMPLETE: self._handle_complete,
            MissionState.ABORT: self._handle_abort,
        }.get(self.current_state)

        if handler:
            handler()

        self._publish_mission_state()

    def _handle_idle(self) -> None:
        _READY_STATES = {'ARMED', 'TAKING_OFF', 'STAGING', 'AVAILABLE', 'EXECUTING_GOAL'}
        ready_drones = [
            rid for rid, st in self.drone_states.items() if st in _READY_STATES
        ]

        if len(ready_drones) >= len(self.expected_drones):
            self.get_logger().info(f'All {len(ready_drones)} drones ready')
            self._transition_to(MissionState.READY)
        elif self.auto_start:
            if self._elapsed_in_state() > 15.0:
                self.get_logger().warn(
                    f'Auto-starting with {len(self.drone_states)}/{len(self.expected_drones)} drones'
                )
                self._transition_to(MissionState.READY)

    def _handle_ready(self) -> None:
        self._transition_to(MissionState.TAKEOFF_WINDOW_A)

    def _handle_takeoff_window(self, window: str) -> None:
        elapsed = self._elapsed_in_state()

        if elapsed > self.takeoff_window_duration:
            self.get_logger().info(f'Takeoff window {window} expired')
            next_state = MissionState.TAKEOFF_WINDOW_B if window == 'A' else MissionState.RUNNING
            self._transition_to(next_state)
            return

        window_drones = self.takeoff_groups[window]
        staged = [
            rid for rid in window_drones
            if self.drone_states.get(rid) in ('STAGING', 'AVAILABLE', 'EXECUTING_GOAL')
        ]

        if len(staged) >= len(window_drones):
            self.get_logger().info(f'All drones in window {window} staged')
            next_state = MissionState.TAKEOFF_WINDOW_B if window == 'A' else MissionState.RUNNING
            self._transition_to(next_state)

    def _handle_running(self) -> None:
        emergency_drones = [
            rid for rid, st in self.drone_states.items() if st == 'EMERGENCY'
        ]
        if emergency_drones:
            self.get_logger().error(f'Emergency detected: {emergency_drones}')
            self._transition_to(MissionState.ABORT)

    def _handle_return_and_land(self) -> None:
        landed = [rid for rid, st in self.drone_states.items() if st == 'LANDED']
        if len(landed) >= len(self.expected_drones):
            self._transition_to(MissionState.COMPLETE)

    def _handle_complete(self) -> None:
        pass

    def _handle_abort(self) -> None:
        pass

    # ── Helpers ──────────────────────────────────────────────────────

    def _elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self.state_start_time).nanoseconds * 1e-9

    def _transition_to(self, new_state: MissionState) -> None:
        if self.current_state != new_state:
            self.get_logger().info(
                f'Mission: {self.current_state.value} -> {new_state.value}'
            )
            self.current_state = new_state
            self.state_start_time = self.get_clock().now()

    def _publish_mission_state(self) -> None:
        msg = String()
        msg.data = self.current_state.value
        self.mission_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
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
