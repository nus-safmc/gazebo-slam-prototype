#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
import re
import math

class SwarmMissionManager(Node):
    def __init__(self):
        super().__init__('swarm_mission_manager')

        # 1. Parameter and ID identification
        self.declare_parameter('num_robots', 1) 
        self.total_robots = self.get_parameter('num_robots').value
        
        ns = self.get_namespace().strip('/')
        match = re.search(r'robot(\d+)', ns)
        # Identify robot: "robot" -> ID 1, "robot2" -> ID 2, etc.
        self.robot_id = int(match.group(1)) if (match and ns != 'robot') else 1
        self.robot_name = ns if ns else 'robot'

        # 2. Frame Configuration (Found via view_frames output)
        self.global_map_frame = 'robot/map' 
        self.base_frame = f'{self.robot_name}/base_link'
        self.cruise_alt = 1.2

        # 3. Setup TF and Action Client
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 4. State Management
        self.mission_started = False
        self.start_time = self.get_clock().now()
        
        # Check readiness every 2.0 seconds
        self.timer = self.create_timer(2.0, self.tick)
        
        self.get_logger().info(f'[{self.robot_name}] Mission Manager online. Root frame: {self.global_map_frame}')

    def get_my_spawn_x(self):
        """ Lookup current X position in the global coordinate system """
        try:
            now = rclpy.time.Time()
            # Lookup transform using the correct frame discovered in logs
            trans = self.tf_buffer.lookup_transform(
                self.global_map_frame, 
                self.base_frame, 
                now, 
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return trans.transform.translation.x
        except Exception as e:
            # self.get_logger().debug(f"TF lookup failed: {str(e)}")
            return None

    def tick(self):
        """ Periodic check to start mission when TF and Nav2 are ready """
        if self.mission_started:
            return

        x0 = self.get_my_spawn_x()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # Handle TF initialization delay
        if x0 is None:
            if elapsed < 20.0:
                self.get_logger().info(f'[{self.robot_name}] Waiting for TF: {self.global_map_frame} -> {self.base_frame}...')
                return
            else:
                self.get_logger().error(f'[{self.robot_name}] TF Timeout. Using ID-based placement fallback.')
                x0 = -8.5 + (self.robot_id - 1) * (17.0 / self.total_robots)

        # Ensure Nav2 action server is ready
        if not self._nav_client.server_is_ready():
            self.get_logger().info(f'[{self.robot_name}] Waiting for NavigateToPose server...')
            return

        # Success: start movement
        target_x, target_y = self.calculate_dispersal_pose(x0)
        self.send_global_goal(target_x, target_y)

    # In swarm_mission_manager.py
    def calculate_dispersal_pose(self, x0):
        # Keep the X spread
        tx = x0 * 1.1
        tx = max(-9.0, min(9.0, tx))

        # ENHANCED DEPTH: Push center drones much further north
        dist_from_center = abs(x0)
        # Increase the multiplier from 8.0 to 12.0 or 14.0
        depth_offset = max(1.0, 14.0 * (1.0 - (dist_from_center / 9.5)))
        
        # Starting from -4.0, a center drone will now try to reach Y = +10.0
        ty = -4.0 + depth_offset 

        return tx, ty

    def send_global_goal(self, tx, ty):
        """ Send the dispersal goal to Nav2 with correct frame nesting """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = tx
        goal_msg.pose.pose.position.y = ty
        goal_msg.pose.pose.position.z = self.cruise_alt
        
        # Face forward (orientation: yaw 1.57)
        goal_msg.pose.pose.orientation.z = 0.707
        goal_msg.pose.pose.orientation.w = 0.707

        self.get_logger().info(f'[{self.robot_name}] Requesting dispersal to X={tx:.1f}, Y={ty:.1f}')
        self._nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        """ Check if Nav2 accepted the goal or is still in Inactive state """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{self.robot_name}] Goal REJECTED. Nav2 stack might be initializing.')
            return
        
        self.mission_started = True
        self.get_logger().info(f'[{self.robot_name}] Dispersal goal accepted. Moving now.')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        """ Goal reached or aborted; lock lane and start searching """
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'[{self.robot_name}] Global target reached. Switching to Local search.')
        else:
            self.get_logger().warn(f'[{self.robot_name}] Global path blocked (Status {status}). Starting Local search now.')
        
        self.activate_local_exploration()

    def activate_local_exploration(self):
        """ Constrain Frontier Search to a specific X-axis lane """
        x0 = self.get_my_spawn_x() or (-8.5 + (self.robot_id - 1) * (1.2))
        
        # Lane restriction: each drone gets a 1.5m wide segment
        lane_width = 1.5 
        my_min_x = x0 - (lane_width / 2.0)
        my_max_x = x0 + (lane_width / 2.0)

        srv_name = 'nav2_frontier_explorer/set_parameters'
        client = self.create_client(SetParameters, srv_name)
        
        if client.wait_for_service(timeout_sec=5.0):
            req = SetParameters.Request()
            params = [
                Parameter(name='enabled', value=ParameterValue(type=4, bool_value=True)),
                Parameter(name='arena_min_x', value=ParameterValue(type=3, double_value=my_min_x)),
                Parameter(name='arena_max_x', value=ParameterValue(type=3, double_value=my_max_x))
            ]
            req.parameters = params
            client.call_async(req)
            self.get_logger().error(f'[{self.robot_name}] Explorer activated. Restricted lane: {my_min_x:.1f} to {my_max_x:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = SwarmMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()