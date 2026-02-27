#!/usr/bin/env python3
"""
Navigator Node
------------------
Subscribes to:
  - /gix_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
  - /scan                             (sensor_msgs/LaserScan)

Publishes:
  - /goal_pose                        (geometry_msgs/PoseStamped)  — Nav2 goal

Logic:
  1. Read the motor angle from the joint trajectory (radians, 0 = forward,
     ±π = ±180°).
  2. Read the LiDAR scan and find the minimum distance in the cone
     [motor_angle - span, motor_angle + span] (default span = 10°).
  3. Publish a Nav2 PoseStamped goal at (distance - threshold_distance)
     in that direction.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs 


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('gix_navigator')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('span_deg', 2.0)          # ±degrees around motor angle
        self.declare_parameter('threshold_distance', 0.5) # metres to stay back
        self.declare_parameter('joint_name', 'gix')       # joint to look for
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('frame_id', 'map')         # Nav2 goal frame

        self.span_rad         = math.radians(self.get_parameter('span_deg').value)
        self.threshold_dist   = self.get_parameter('threshold_distance').value
        self.joint_name       = self.get_parameter('joint_name').value
        self.scan_topic       = self.get_parameter('scan_topic').value
        self.goal_topic       = self.get_parameter('goal_topic').value
        self.frame_id         = self.get_parameter('frame_id').value

        # ── State ─────────────────────────────────────────────────────────────
        self.motor_angle_rad: float | None = None   # latest commanded angle
        self.latest_scan: LaserScan | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.get_logger().info(
            f'GIX Navigator started | span=±{math.degrees(self.span_rad):.1f}° '
            f'| threshold={self.threshold_dist} m'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def trajectory_callback(self, msg: JointTrajectory):
        """Extract the motor angle from the first trajectory point."""
        if self.joint_name not in msg.joint_names:
            self.get_logger().warn(
                f"Joint '{self.joint_name}' not found in message. "
                f"Available joints: {msg.joint_names}"
            )
            return

        if not msg.points:
            self.get_logger().warn('Received JointTrajectory with no points.')
            return

        joint_idx = msg.joint_names.index(self.joint_name)
        point = msg.points[0]

        if joint_idx >= len(point.positions):
            self.get_logger().warn('Position index out of range for joint.')
            return

        self.motor_angle_rad = point.positions[joint_idx]
        self.get_logger().info(
            f'Motor angle updated: {math.degrees(self.motor_angle_rad):.2f}°'
        )

        # Immediately attempt to compute and publish goal if we have a scan
        self.try_publish_goal()

    def scan_callback(self, msg: LaserScan):
        """Cache the latest scan."""
        self.latest_scan = msg

    # ── Core Logic ────────────────────────────────────────────────────────────

    def try_publish_goal(self):
        """Compute closest obstacle in the motor cone and publish a Nav2 goal."""
        if self.motor_angle_rad is None or self.latest_scan is None:
            return

        scan = self.latest_scan
        motor_angle = self.motor_angle_rad

        # ── Find minimum distance in the cone ──────────────────────────────
        # had to negate motor_angle because LiDAR angles are CCW-positive but motor is CW-positive
        min_distance = self._min_distance_in_cone(scan, -motor_angle, self.span_rad)

        if min_distance is None:
            self.get_logger().warn(
                'No valid range reading found in the motor direction cone.'
            )
            return

        goal_distance = min_distance - self.threshold_dist

        if goal_distance <= 0.0:
            self.get_logger().warn(
                f'Obstacle too close ({min_distance:.2f} m). '
                f'Goal distance would be {goal_distance:.2f} m — skipping.'
            )
            return

        self.get_logger().info(
            f'Obstacle at {min_distance:.2f} m in direction '
            f'{math.degrees(motor_angle):.1f}° → publishing goal at '
            f'{goal_distance:.2f} m'
        )

        # ── Build goal pose ────────────────────────────────────────────────
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id

        # Get robot's current pose in map frame
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Could not get robot transform: {e}')
            return

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        # Get robot's current yaw from quaternion
        q = tf.transform.rotation
        robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Compute goal in map frame
        yaw = robot_yaw - motor_angle  # CW motor relative to robot heading
        goal.pose.position.x = robot_x + goal_distance * math.cos(yaw)
        goal.pose.position.y = robot_y + goal_distance * math.sin(yaw)

        half_yaw = yaw / 2.0
        goal.pose.orientation.z = math.sin(half_yaw)
        goal.pose.orientation.w = math.cos(half_yaw)

        self.get_logger().info(
        f'robot_x={robot_x:.2f} robot_y={robot_y:.2f} '
        f'robot_yaw={math.degrees(robot_yaw):.2f}° '
        f'motor={math.degrees(motor_angle):.2f}° '
        f'goal_yaw={math.degrees(yaw):.2f}°'
)
        self.goal_pub.publish(goal)

    @staticmethod
    def _min_distance_in_cone(
        scan: LaserScan,
        center_angle: float,
        half_span: float,
    ) -> float | None:
        """
        Return the minimum valid range in [center_angle - half_span,
        center_angle + half_span].

        The LaserScan angles are relative to the robot's forward direction
        (same convention as the motor: 0 = forward, positive = CCW).
        """
        lo = center_angle - half_span
        hi = center_angle + half_span

        min_dist = None

        for i, r in enumerate(scan.ranges):
            # Skip invalid readings
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # Normalise angle to [-π, π] to handle wrap-around
            angle_norm = math.atan2(math.sin(angle), math.cos(angle))
            lo_norm    = math.atan2(math.sin(lo),    math.cos(lo))
            hi_norm    = math.atan2(math.sin(hi),    math.cos(hi))

            # Check if angle_norm is within [lo_norm, hi_norm]
            # (works even when the cone wraps through ±π)
            if lo_norm <= hi_norm:
                in_cone = lo_norm <= angle_norm <= hi_norm
            else:
                # Wrapped: lo > hi in normalized space
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