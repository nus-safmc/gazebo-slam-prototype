#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import math
from typing import Optional


class TofToScan(Node):
    #Concatenates depth images from all ToF sensors, publishes as single LaserScan message

    def __init__(self):
        super().__init__('TofToScan')

        self.depth_topic_prefix = (
            self.declare_parameter('depth_topic_prefix', 'depth')
            .get_parameter_value()
            .string_value
            .strip()
        )
        if not self.depth_topic_prefix:
            self.depth_topic_prefix = 'depth'
        self.depth_topic_prefix = self.depth_topic_prefix.rstrip('/')

        self.output_frame = (
            self.declare_parameter('output_frame', 'robot/base_link')
            .get_parameter_value()
            .string_value
        )
        self.output_topic = (
            self.declare_parameter('output_topic', 'scan_merged')
            .get_parameter_value()
            .string_value
        )
        self.viz_output_topic = (
            self.declare_parameter('viz_output_topic', '')
            .get_parameter_value()
            .string_value
            .strip()
        )
        publish_hz = (
            self.declare_parameter('publish_hz', 10.0)
            .get_parameter_value()
            .double_value
        )
        self.publish_hz = float(publish_hz)
        if self.publish_hz <= 0.0:
            self.publish_hz = 10.0

        # QoS profile for depth data
        depth_qos = QoSProfile(
            # ros_gz_bridge publishes depth images as RELIABLE by default; match it so
            # the ToF pipeline receives sensor data consistently.
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.h_fov = math.pi/4.0
        self.v_fov = math.pi/4.0
        self.output_num_points = int(self.declare_parameter('output_num_points', 360).value)
        self.output_num_points = max(8, self.output_num_points)
        self.output_fill_bins = bool(self.declare_parameter('output_fill_bins', True).value)
        self.ring_radius_m = float(
            self.declare_parameter('ring_radius_m', 0.05).get_parameter_value().double_value
        )
        self.minrange = float(
            self.declare_parameter('min_range_m', 0.25).get_parameter_value().double_value
        )
        self.maxrange = float(
            self.declare_parameter('max_range_m', 6.0).get_parameter_value().double_value
        )
        self.minrange = max(0.0, self.minrange)
        self.maxrange = max(self.minrange + 0.01, self.maxrange)
        self.roi_row_start = int(self.declare_parameter('roi_row_start', 2).value)
        self.roi_row_end = int(self.declare_parameter('roi_row_end', 6).value)
        self.column_reduce = (
            self.declare_parameter('column_reduce', 'min')
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )
        self.min_valid_per_column = int(self.declare_parameter('min_valid_per_column', 1).value)
        self.min_valid_per_column = max(1, self.min_valid_per_column)
        self.far_clip_margin_m = float(self.declare_parameter('far_clip_margin_m', 0.01).value)
        self.far_clip_margin_m = max(0.0, self.far_clip_margin_m)
        # `LaserScan.range_max` is treated by Karto (used by slam_toolbox) as a *hard* maximum
        # range: readings >= maxRange are ignored entirely.
        #
        # We represent "no return" beams as a finite *threshold* range that clears free space
        # without creating a phantom obstacle ring. To ensure those beams are not ignored, keep:
        #   no_return_range < scan.range_max
        self.range_max_margin_m = float(self.declare_parameter('range_max_margin_m', 0.20).value)
        self.range_max_margin_m = max(0.0, self.range_max_margin_m)
        self.no_return_range_m = float(
            self.declare_parameter('no_return_range_m', 0.0).get_parameter_value().double_value
        )
        # If true, include "no return" beams as a finite threshold distance instead of +inf.
        # slam_toolbox uses Karto, which ignores readings >= LaserScan.range_max; we therefore
        # publish no-return beams below range_max so Karto can ray-trace free space.
        self.no_return_as_range_max = bool(
            self.declare_parameter('no_return_as_range_max', False).value
        )

        # Optional angular-domain filtering to reduce single-beam speckle from sparse ToF scans.
        self.angular_filter_window = int(self.declare_parameter('angular_filter_window', 1).value)
        self.angular_filter_window = max(1, self.angular_filter_window)
        if self.angular_filter_window % 2 == 0:
            self.angular_filter_window += 1
        self.angular_outlier_thresh_m = float(
            self.declare_parameter('angular_outlier_thresh_m', 0.6).value
        )
        self.angular_outlier_thresh_m = max(0.0, self.angular_outlier_thresh_m)
        self.angular_fill_gaps = bool(self.declare_parameter('angular_fill_gaps', False).value)
        self.angular_fill_min_neighbors = int(
            self.declare_parameter('angular_fill_min_neighbors', 3).value
        )
        self.angular_fill_min_neighbors = max(1, self.angular_fill_min_neighbors)
        self.angular_fill_spread_max_m = float(
            self.declare_parameter('angular_fill_spread_max_m', 0.5).value
        )
        self.angular_fill_spread_max_m = max(0.0, self.angular_fill_spread_max_m)

        # Optional temporal smoothing (median) over the last N scans to reduce flicker.
        self.temporal_window = int(self.declare_parameter('temporal_window', 1).value)
        self.temporal_window = max(1, self.temporal_window)
        self._temporal: list[list[float]] = []

        self.cv_bridge = CvBridge()
        self._scaled_mm_sensors: set[int] = set()

        self._latest: list[Optional[Image]] = [None] * 8

        for i in range(8):
            topic = f'{self.depth_topic_prefix}/tof_{i+1}'

            def _make_cb(index: int):
                def _cb(msg: Image) -> None:
                    self._latest[index] = msg

                return _cb

            self.create_subscription(Image, topic, _make_cb(i), depth_qos)

        scan_qos = QoSProfile(
            # LaserScan is high-rate sensor data; BEST_EFFORT matches common ROS sensor QoS
            # (and allows downstream consumers like AutoPilot to subscribe reliably).
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(LaserScan, self.output_topic, scan_qos)
        self.viz_pub = None
        if self.viz_output_topic:
            self.viz_pub = self.create_publisher(LaserScan, self.viz_output_topic, scan_qos)

        period_sec = 1.0 / max(0.1, self.publish_hz)
        self.create_timer(period_sec, self._publish_scan)

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        # Wrap to [-pi, pi)
        return float((float(angle) + math.pi) % (2.0 * math.pi) - math.pi)

    def _angular_filter(self, ranges: list[float]) -> list[float]:
        window = int(self.angular_filter_window)
        if window <= 1 or len(ranges) < 3:
            return ranges

        half = window // 2
        n = len(ranges)
        out = list(ranges)
        for i in range(n):
            neigh: list[float] = []
            for off in range(-half, half + 1):
                val = ranges[(i + off) % n]
                if math.isfinite(val):
                    neigh.append(float(val))
            if not neigh:
                continue

            med = float(np.median(np.asarray(neigh, dtype=np.float32)))
            cur = ranges[i]
            if math.isfinite(cur):
                if abs(float(cur) - med) > float(self.angular_outlier_thresh_m):
                    out[i] = med
            else:
                if not self.angular_fill_gaps:
                    continue
                if len(neigh) < self.angular_fill_min_neighbors:
                    continue
                if (max(neigh) - min(neigh)) > float(self.angular_fill_spread_max_m):
                    continue
                out[i] = med
        return out

    def _publish_scan(self) -> None:
        images = self._latest

        # Avoid publishing "empty" scans before any sensor data arrives. This prevents downstream
        # mappers (slam_toolbox / swarm_map_fuser) from publishing a pre-sized all-unknown map at
        # startup.
        if all(img is None for img in images):
            return

        try:
            merged_scan = LaserScan()
            # Stamp with the oldest contributing sensor image stamp to reduce TF extrapolation
            # into the future (a common cause of message_filter queue overflows).
            oldest_ns: Optional[int] = None
            oldest_stamp = None
            for img in images:
                if img is None:
                    continue
                stamp = img.header.stamp
                if stamp.sec == 0 and stamp.nanosec == 0:
                    continue
                stamp_ns = self._stamp_ns(stamp)
                if oldest_ns is None or stamp_ns < oldest_ns:
                    oldest_ns = stamp_ns
                    oldest_stamp = stamp

            merged_scan.header.stamp = (
                oldest_stamp if oldest_stamp is not None else self.get_clock().now().to_msg()
            )
            merged_scan.header.frame_id = self.output_frame
            # Use the conventional LaserScan angle convention: [-pi, +pi).
            # Some consumers (notably RViz displays and certain filters) behave better with
            # this standard range than [0, 2*pi).
            merged_scan.angle_min = -math.pi

            num_points = int(self.output_num_points)
            merged_scan.angle_increment = (2.0 * math.pi) / float(num_points)
            merged_scan.angle_max = merged_scan.angle_min + merged_scan.angle_increment * (num_points - 1)
            merged_scan.time_increment = 0.0
            merged_scan.scan_time = 1.0 / max(0.1, self.publish_hz) if self.publish_hz > 0.0 else 1.0 / 30.0
            merged_scan.range_min = self.minrange
            # Ranges are expressed from the base_link origin, but individual sensors are offset
            # from the origin by ring_radius_m. Account for that so valid points near the
            # sensor far-clip remain within [range_min, range_max].
            base_max = float(self.maxrange + self.ring_radius_m)
            merged_scan.range_max = float(base_max + self.range_max_margin_m)
            # Default the no-return threshold to the physical ToF max range from the base origin.
            no_return_range = (
                float(self.no_return_range_m) if self.no_return_range_m > 0.0 else base_max
            )
            no_return_range = min(no_return_range, float(merged_scan.range_max - 1e-3))

            ranges = [float('inf')] * num_points
            bin_width = self.h_fov / 8.0
            half_bin = 0.5 * bin_width

            for sensor_index in range(8):
                depth_image = images[sensor_index]
                if depth_image is None:
                    continue
                sensor_angle = sensor_index * (math.pi / 4.0)
                sx = self.ring_radius_m * math.cos(sensor_angle)
                sy = self.ring_radius_m * math.sin(sensor_angle)
                image = self.cv_bridge.imgmsg_to_cv2(depth_image)
                rows = int(image.shape[0]) if hasattr(image, 'shape') else 0
                row_start = max(0, min(rows, int(self.roi_row_start)))
                row_end = max(row_start + 1, min(rows, int(self.roi_row_end)))

                # Some Gazebo depth images come through as millimeters (e.g. uint16),
                # others are already meters (float32). Auto-detect by magnitude.
                roi = image[row_start:row_end, :8] if rows > 0 else image
                try:
                    finite_roi = roi[np.isfinite(roi)]
                    if finite_roi.size > 0:
                        med = float(np.median(finite_roi))
                        if med > (self.maxrange * 10.0):
                            image = image.astype(np.float32) * 0.001
                            if sensor_index not in self._scaled_mm_sensors:
                                self._scaled_mm_sensors.add(sensor_index)
                                self.get_logger().info(
                                    f'/depth/tof_{sensor_index+1} appears to be in millimeters; scaling to meters.'
                                )
                except Exception:
                    pass

                img_ranges = []
                far_clip_threshold = max(self.minrange, self.maxrange - float(self.far_clip_margin_m))
                for col in range(8):
                    col_depths = image[row_start:row_end, col]
                    valid_depths = col_depths[
                        (col_depths >= self.minrange) & 
                        (col_depths <= far_clip_threshold)
                    ]
                    if len(valid_depths) >= self.min_valid_per_column:
                        reduce = self.column_reduce
                        if reduce == 'max':
                            img_ranges.append(float(np.max(valid_depths)))
                        elif reduce == 'median':
                            img_ranges.append(float(np.median(valid_depths)))
                        elif reduce.startswith('p') and reduce[1:].isdigit():
                            p = max(0, min(100, int(reduce[1:])))
                            img_ranges.append(float(np.percentile(valid_depths, p)))
                        else:
                            img_ranges.append(float(np.min(valid_depths)))
                    else:
                        img_ranges.append(float('inf'))
                
                # Map each range to the merged scan
                for col_index, r in enumerate(img_ranges):
                    far_clip_threshold = float(self.maxrange - float(self.far_clip_margin_m))
                    is_no_return = (not math.isfinite(r)) or float(r) >= far_clip_threshold
                    if is_no_return and not self.no_return_as_range_max:
                        continue
                    r = float(r) if math.isfinite(r) else float('inf')

                    # Horizontal beam angle relative to this sensor (centered bins).
                    theta_rel = (self.h_fov * 0.5) - (float(col_index) + 0.5) * (self.h_fov / 8.0)
                    theta = self._wrap_pi(sensor_angle + theta_rel)

                    # Convert this sensor measurement into an equivalent point in the base frame,
                    # then re-bin by the point's bearing from the base origin.
                    #
                    # This corrects the parallax introduced by the 5cm sensor ring: if we bin purely by
                    # the sensor beam angle (ignoring the offset), close obstacles can shift by multiple
                    # degrees, producing the characteristic "spoke" / smeared maps.
                    if is_no_return:
                        r_used = float(self.maxrange)
                    else:
                        r_used = float(r)

                    px = sx + r_used * math.cos(theta)
                    py = sy + r_used * math.sin(theta)
                    base_angle = self._wrap_pi(math.atan2(py, px))
                    base_range = float(math.hypot(px, py))

                    if not is_no_return:
                        if (not math.isfinite(base_range)) or base_range < float(merged_scan.range_min):
                            continue
                        if base_range > float(merged_scan.range_max):
                            continue

                    if self.output_fill_bins:
                        theta_start = self._wrap_pi(base_angle - half_bin)
                        theta_end = self._wrap_pi(base_angle + half_bin)

                        def _fill_span(a0: float, a1: float) -> None:
                            idx0 = int(math.floor((a0 - merged_scan.angle_min) / merged_scan.angle_increment))
                            idx1 = int(math.floor((a1 - merged_scan.angle_min) / merged_scan.angle_increment))
                            idx0 = max(0, min(num_points - 1, idx0))
                            idx1 = max(0, min(num_points - 1, idx1))
                            for idx in range(idx0, idx1 + 1):
                                if is_no_return:
                                    # For "no return" beams, publish a finite threshold (not `range_max`).
                                    # Karto uses `range_threshold` to ray-trace free space without marking
                                    # an obstacle at the end, but it ignores readings >= maxRange.
                                    ranges[idx] = min(ranges[idx], no_return_range)
                                else:
                                    ranges[idx] = min(ranges[idx], base_range)

                        if theta_start <= theta_end:
                            _fill_span(theta_start, theta_end)
                        else:
                            _fill_span(theta_start, math.pi - 1e-9)
                            _fill_span(-math.pi, theta_end)
                    else:
                        if is_no_return:
                            r_center = no_return_range
                        else:
                            r_center = base_range

                        idx = int((base_angle - merged_scan.angle_min) / merged_scan.angle_increment)
                        if 0 <= idx < num_points:
                            ranges[idx] = min(ranges[idx], r_center)

            ranges = self._angular_filter(ranges)

            if self.temporal_window > 1:
                self._temporal.append(list(ranges))
                if len(self._temporal) > self.temporal_window:
                    self._temporal.pop(0)

                # Median of finite ranges across history (leave inf when no finite samples).
                smoothed: list[float] = []
                for i in range(num_points):
                    vals = [float(r[i]) for r in self._temporal if math.isfinite(r[i])]
                    if not vals:
                        smoothed.append(float('inf'))
                    else:
                        smoothed.append(float(np.median(np.asarray(vals, dtype=np.float32))))
                ranges = smoothed
            merged_scan.ranges = ranges
            self.pub.publish(merged_scan)

            if self.viz_pub is not None:
                viz = LaserScan()
                viz.header = merged_scan.header
                viz.header.frame_id = merged_scan.header.frame_id
                viz.angle_min = merged_scan.angle_min
                viz.angle_max = merged_scan.angle_max
                viz.angle_increment = merged_scan.angle_increment
                viz.time_increment = merged_scan.time_increment
                viz.scan_time = merged_scan.scan_time
                viz.range_min = merged_scan.range_min
                viz.range_max = merged_scan.range_max
                # RViz only draws finite ranges; substitute "no return" with range_max for display.
                viz.ranges = [
                    (viz.range_max if (not math.isfinite(val)) else float(val)) for val in merged_scan.ranges
                ]
                viz.intensities = list(merged_scan.intensities)
                self.viz_pub.publish(viz)
        
        except Exception as e:
            self.get_logger().error(f'Error merging scans: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TofToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
