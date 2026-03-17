#!/usr/bin/env python3
"""
Scan Filter - Removes tracked person cone from LiDAR for SLAM.
Also masks out blocked/invalid LiDAR regions.
Subscribes to /scan and motor angle, publishes /scan_filtered.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory


class ScanFilterNode(Node):

    def __init__(self):
        super().__init__('scan_filter')

        # Parameters - Person cone masking
        self.declare_parameter('cone_half_angle_deg', 15.0)  # ±degrees to mask
        self.declare_parameter('filter_max_range', 4.0)      # Only filter close readings
        self.declare_parameter('joint_name', 'gix')

        # Parameters - Valid LiDAR FOV (back is blocked)
        # These define the VALID range, everything outside is masked
        # Default: front 270° (-135° to +135°), back 90° blocked
        self.declare_parameter('lidar_min_angle_deg', -100.0)
        self.declare_parameter('lidar_max_angle_deg', 100.0)

        self.cone_half_rad = math.radians(
            self.get_parameter('cone_half_angle_deg').value
        )
        self.filter_max_range = self.get_parameter('filter_max_range').value
        self.joint_name = self.get_parameter('joint_name').value

        self.lidar_min_angle = math.radians(
            self.get_parameter('lidar_min_angle_deg').value
        )
        self.lidar_max_angle = math.radians(
            self.get_parameter('lidar_max_angle_deg').value
        )

        # State
        self.motor_angle: float | None = None

        # Subscriptions
        self.create_subscription(
            JointTrajectory,
            '/gix_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        # Publisher - filtered scan for SLAM
        self.filtered_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        self.get_logger().info(
            f'Scan filter started | '
            f'valid FOV: [{math.degrees(self.lidar_min_angle):.0f}°, '
            f'{math.degrees(self.lidar_max_angle):.0f}°] | '
            f'person mask: ±{math.degrees(self.cone_half_rad):.0f}°'
        )

    def trajectory_callback(self, msg: JointTrajectory):
        """Get current tracking angle from camera/motor."""
        if self.joint_name not in msg.joint_names:
            return
        if not msg.points:
            return

        idx = msg.joint_names.index(self.joint_name)
        if idx >= len(msg.points[0].positions):
            return

        self.motor_angle = msg.points[0].positions[idx]

    def scan_callback(self, msg: LaserScan):
        """Filter scan and publish."""
        # Copy the scan message
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = list(msg.ranges)
        filtered.intensities = list(msg.intensities) if msg.intensities else []

        # Convert motor angle to LiDAR frame (if tracking)
        mask_center = None
        if self.motor_angle is not None:
            mask_center = -self.motor_angle  # CW+ to CCW+

        for i in range(len(filtered.ranges)):
            beam_angle = msg.angle_min + i * msg.angle_increment
            beam_angle_norm = self._normalize_angle(beam_angle)
            
            # 1. Mask readings outside valid LiDAR FOV (blocked regions)
            if not self._is_in_valid_fov(beam_angle_norm):
                filtered.ranges[i] = float('inf')
                continue

            # 2. Mask person cone (only for close readings)
            if mask_center is not None:
                r = filtered.ranges[i]
                
                # Skip invalid readings
                if math.isnan(r) or math.isinf(r):
                    continue
                
                # Only mask close readings (the person)
                if r <= self.filter_max_range:
                    angle_diff = self._normalize_angle(beam_angle_norm - mask_center)
                    if abs(angle_diff) <= self.cone_half_rad:
                        filtered.ranges[i] = float('inf')

        self.filtered_pub.publish(filtered)

    def _is_in_valid_fov(self, angle: float) -> bool:
        """Check if angle is within valid LiDAR FOV."""
        # Handle wrap-around if min > max (e.g., 135° to -135° meaning front is valid)
        if self.lidar_min_angle <= self.lidar_max_angle:
            # Normal case: valid range is continuous
            return self.lidar_min_angle <= angle <= self.lidar_max_angle
        else:
            # Wrapped case: valid range spans across ±π
            # e.g., min=135°, max=-135° means BACK is blocked
            return angle >= self.lidar_min_angle or angle <= self.lidar_max_angle

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()