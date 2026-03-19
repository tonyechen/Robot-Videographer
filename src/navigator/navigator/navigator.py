#!/usr/bin/env python3
"""
Navigator Node (PID)
--------------------
Subscribes to:
  - /gix_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
  - /scan                             (sensor_msgs/LaserScan)

Publishes:
  - /cmd_vel  (geometry_msgs/Twist) — direct velocity control

Logic:
  1. Read motor angle (direction to person).
  2. Find distance to person in LiDAR cone around that angle.
  3. Linear PID: drive forward/back to maintain threshold_distance.
  4. Angular PID: turn to keep motor angle centred (track person direction).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('gix_navigator')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('span_deg', 10.0)           # ±degrees around motor angle
        self.declare_parameter('threshold_distance', 1.5)  # metres to maintain
        self.declare_parameter('joint_name', 'gix')
        self.declare_parameter('scan_topic', '/scan')

        # Linear PID
        self.declare_parameter('kp_linear', 0.3)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.05)

        # Angular PID
        self.declare_parameter('kp_angular', 0.5)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.05)

        # Speed limits and bias
        self.declare_parameter('max_linear', 0.15)
        self.declare_parameter('max_angular', 0.5)             # absolute angular speed limit
        self.declare_parameter('angular_bias', 0.0)             # offset to go straight
        self.declare_parameter('linear_deadband', 0.2)         # metres — stop within this of threshold

        self.span_rad              = math.radians(self.get_parameter('span_deg').value)
        self.threshold_dist        = self.get_parameter('threshold_distance').value
        self.joint_name            = self.get_parameter('joint_name').value
        self.scan_topic            = self.get_parameter('scan_topic').value
        self.kp_linear             = self.get_parameter('kp_linear').value
        self.ki_linear             = self.get_parameter('ki_linear').value
        self.kd_linear             = self.get_parameter('kd_linear').value
        self.kp_angular            = self.get_parameter('kp_angular').value
        self.ki_angular            = self.get_parameter('ki_angular').value
        self.kd_angular            = self.get_parameter('kd_angular').value
        self.max_linear            = self.get_parameter('max_linear').value
        self.max_angular           = self.get_parameter('max_angular').value
        self.angular_bias          = self.get_parameter('angular_bias').value
        self.linear_deadband       = self.get_parameter('linear_deadband').value

        # ── State ─────────────────────────────────────────────────────────────
        self.motor_angle_rad: float | None = None
        self.latest_scan: LaserScan | None = None

        self.linear_integral    = 0.0
        self.linear_prev_error  = 0.0
        self.angular_integral   = 0.0
        self.angular_prev_error = 0.0
        self.last_time: float | None = None

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.traj_sub = self.create_subscription(
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
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Control loop at 10 Hz ─────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'GIX Navigator (PID) started | span=±{math.degrees(self.span_rad):.1f}° '
            f'| threshold={self.threshold_dist} m '
            f'| max_linear={self.max_linear} m/s '
            f'| angular bias={self.angular_bias} max=±{self.max_angular} rad/s'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def trajectory_callback(self, msg: JointTrajectory):
        if self.joint_name not in msg.joint_names:
            return
        if not msg.points:
            return
        joint_idx = msg.joint_names.index(self.joint_name)
        point = msg.points[0]
        if joint_idx >= len(point.positions):
            return
        self.motor_angle_rad = point.positions[joint_idx]

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    # ── Control Loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        if self.motor_angle_rad is None or self.latest_scan is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time if self.last_time is not None else 0.1
        self.last_time = now
        if dt <= 0.0:
            dt = 0.1

        motor_angle = self.motor_angle_rad
        # motor positive = camera faces LEFT = person is to the LEFT
        # LiDAR is CCW-positive, so left = positive → pass motor_angle directly
        min_distance = self._min_distance_in_cone(
            self.latest_scan, motor_angle, self.span_rad
        )

        if min_distance is None:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().warn('No valid range in motor cone — stopping.')
            return

        # ── Linear PID ────────────────────────────────────────────────────────
        # positive error = too far → drive forward
        linear_error = min_distance - self.threshold_dist
        if abs(linear_error) < self.linear_deadband:
            linear_vel = 0.0
            self.linear_integral = 0.0
            self.linear_prev_error = 0.0
        else:
            self.linear_integral += linear_error * dt
            linear_derivative = (linear_error - self.linear_prev_error) / dt
            linear_vel = (
                self.kp_linear * linear_error
                + self.ki_linear * self.linear_integral
                + self.kd_linear * linear_derivative
            )
            self.linear_prev_error = linear_error
            linear_vel = max(-self.max_linear, min(self.max_linear, linear_vel))

        # ── Angular PID ───────────────────────────────────────────────────────
        # motor_angle > 0 = person to the LEFT → turn left = positive angular vel
        angular_error = motor_angle
        self.angular_integral += angular_error * dt
        angular_derivative = (angular_error - self.angular_prev_error) / dt
        pid_angular = (
            self.kp_angular * angular_error
            + self.ki_angular * self.angular_integral
            + self.kd_angular * angular_derivative
        )
        self.angular_prev_error = angular_error
        # Apply hardware bias and clamp to absolute speed limits
        angular_vel = self.angular_bias + pid_angular
        angular_vel = max(-self.max_angular, min(self.max_angular, angular_vel))

        self.get_logger().info(
            f'dist={min_distance:.2f}m err={linear_error:+.2f}m '
            f'v={linear_vel:+.2f}m/s motor={math.degrees(motor_angle):+.1f}° '
            f'w={angular_vel:+.2f}rad/s'
        )

        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _min_distance_in_cone(
        scan: LaserScan,
        center_angle: float,
        half_span: float,
    ) -> float | None:
        lo = center_angle - half_span
        hi = center_angle + half_span
        min_dist = None

        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            angle_norm = math.atan2(math.sin(angle), math.cos(angle))
            lo_norm    = math.atan2(math.sin(lo),    math.cos(lo))
            hi_norm    = math.atan2(math.sin(hi),    math.cos(hi))

            if lo_norm <= hi_norm:
                in_cone = lo_norm <= angle_norm <= hi_norm
            else:
                in_cone = angle_norm >= lo_norm or angle_norm <= hi_norm

            if in_cone:
                if min_dist is None or r < min_dist:
                    min_dist = r

        return min_dist


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
