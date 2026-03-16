import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from yolo_msgs.msg import DetectionArray as YoloDetectionArray


class YoloPersonTracker(Node):
    """
    Pan a camera joint to keep the best 'person' detection centered.
    Uses PID control with filtering for smooth tracking.
    """

    def __init__(self):
        super().__init__('yolo_person_tracker')

        # ---- Parameters ----
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('controller_topic', '/gix_controller/joint_trajectory')
        self.declare_parameter('joint_name', 'gix')
        self.declare_parameter('label', 'person')
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('deadband_px', 12)

        # PID gains
        self.declare_parameter('kp', 0.001)       # Reduced from 0.0035
        self.declare_parameter('ki', 0.0)      # Small integral gain
        self.declare_parameter('kd', 0.000)       # Derivative to dampen oscillation

        self.declare_parameter('max_step', 0.05)  # Reduced from 0.08
        self.declare_parameter('cmd_time_sec', 0.15)
        self.declare_parameter('min_angle', -1.6)
        self.declare_parameter('max_angle', 1.6)
        self.declare_parameter('lost_timeout_sec', 2.0)
        self.declare_parameter('return_to_center_on_lost', False)
        self.declare_parameter('center_angle', 0.0)

        # Smoothing parameters
        self.declare_parameter('detection_smoothing', 0.3)  # Low-pass filter alpha (0-1, lower = smoother)
        self.declare_parameter('integral_max', 50.0)        # Anti-windup limit

        # ---- Load params ----
        self.detections_topic = self.get_parameter('detections_topic').value
        self.controller_topic = self.get_parameter('controller_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        self.label = self.get_parameter('label').value
        self.min_score = float(self.get_parameter('min_score').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.deadband_px = int(self.get_parameter('deadband_px').value)

        # PID gains
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        self.max_step = float(self.get_parameter('max_step').value)
        self.cmd_time_sec = float(self.get_parameter('cmd_time_sec').value)
        self.min_angle = float(self.get_parameter('min_angle').value)
        self.max_angle = float(self.get_parameter('max_angle').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout_sec').value)
        self.return_on_lost = bool(self.get_parameter('return_to_center_on_lost').value)
        self.center_angle = float(self.get_parameter('center_angle').value)

        self.detection_smoothing = float(self.get_parameter('detection_smoothing').value)
        self.integral_max = float(self.get_parameter('integral_max').value)

        # ---- State ----
        self._last_seen_time = 0.0
        self._target_cx_px = None
        self._smoothed_cx_px = None  # Filtered detection position
        self._current_angle = self.center_angle

        # PID state
        self._integral = 0.0
        self._last_err_px = 0.0
        self._last_control_time = time.time()

        # ---- Debug counters ----
        self._detection_msg_count = 0
        self._matching_detection_count = 0
        self._control_step_count = 0
        self._trajectory_publish_count = 0
        self._last_status_time = time.time()

        # ---- Pub/Sub ----
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)
        self.sub = self.create_subscription(
            YoloDetectionArray, self.detections_topic, self.on_detections, 10
        )

        # ---- Timer loop ----
        period = 1.0 / max(self.control_rate_hz, 1e-6)
        self.timer = self.create_timer(period, self.control_step)

        # ---- Status timer (every 3 seconds) ----
        self.status_timer = self.create_timer(3.0, self.log_status)

        self.get_logger().info(
            f"=== YoloPersonTracker Started (PID) ===\n"
            f"  Tracking label: '{self.label}'\n"
            f"  Detections topic: {self.detections_topic}\n"
            f"  Controller topic: {self.controller_topic}\n"
            f"  Joint name: {self.joint_name}\n"
            f"  Image width: {self.image_width}px\n"
            f"  Min score: {self.min_score}\n"
            f"  Deadband: {self.deadband_px}px\n"
            f"  PID: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}\n"
            f"  Max step: {self.max_step} rad\n"
            f"  Angle limits: [{self.min_angle}, {self.max_angle}] rad\n"
            f"  Control rate: {self.control_rate_hz} Hz\n"
            f"  Detection smoothing: {self.detection_smoothing}"
        )

    def log_status(self):
        """Periodic status log to show system health."""
        now = time.time()
        self._last_status_time = now
        seen_ago = now - self._last_seen_time if self._last_seen_time > 0 else float('inf')

        self.get_logger().info(
            f"[STATUS] Det msgs: {self._detection_msg_count}, "
            f"Matches: {self._matching_detection_count}, "
            f"Traj published: {self._trajectory_publish_count}, "
            f"Last seen: {seen_ago:.1f}s ago, "
            f"Angle: {self._current_angle:.3f} rad, "
            f"Integral: {self._integral:.1f}"
        )

        # Reset counters
        self._detection_msg_count = 0
        self._matching_detection_count = 0
        self._control_step_count = 0
        self._trajectory_publish_count = 0

    def on_detections(self, msg: YoloDetectionArray):
        self._detection_msg_count += 1

        if len(msg.detections) == 0:
            return

        # Choose best matching detection by score
        best_score = -1.0
        best_cx = None

        for det in msg.detections:
            if det.class_name != self.label:
                continue
            if float(det.score) < self.min_score:
                continue
            try:
                cx = float(det.bbox.center.position.x)
            except Exception:
                continue

            if float(det.score) > best_score:
                best_score = float(det.score)
                best_cx = cx

        if best_cx is not None:
            self._target_cx_px = best_cx
            self._last_seen_time = time.time()
            self._matching_detection_count += 1

            # Low-pass filter on detection position to reduce jitter
            if self._smoothed_cx_px is None:
                self._smoothed_cx_px = best_cx
            else:
                alpha = self.detection_smoothing
                self._smoothed_cx_px = alpha * best_cx + (1 - alpha) * self._smoothed_cx_px

            self.get_logger().debug(
                f"[DETECT] ✓ '{self.label}' raw_x={best_cx:.1f}, smoothed_x={self._smoothed_cx_px:.1f} (score={best_score:.2f})"
            )

    def control_step(self):
        self._control_step_count += 1
        now = time.time()
        dt = now - self._last_control_time
        self._last_control_time = now

        seen_recently = (now - self._last_seen_time) <= self.lost_timeout

        if not seen_recently:
            if self._target_cx_px is not None:
                self.get_logger().warn(f"[CONTROL] Target lost!")

            # Reset state when target lost
            self._target_cx_px = None
            self._smoothed_cx_px = None
            self._integral = 0.0
            self._last_err_px = 0.0

            if self.return_on_lost:
                self._move_to(self.center_angle)
            return

        if self._smoothed_cx_px is None:
            return

        center_px = 0.5 * self.image_width
        err_px = float(self._smoothed_cx_px) - center_px

        # Check deadband - also reset integral when in deadband to prevent windup
        if abs(err_px) <= self.deadband_px:
            self._integral = 0.0  # Reset integral when centered
            self._last_err_px = err_px
            return

        # ---- PID Control ----
        # Proportional
        p_term = self.kp * err_px

        # Integral (with anti-windup)
        self._integral += err_px * dt
        self._integral = max(-self.integral_max, min(self.integral_max, self._integral))
        i_term = self.ki * self._integral

        # Derivative (on error, with dt protection)
        if dt > 0.001:
            d_term = self.kd * (err_px - self._last_err_px) / dt
        else:
            d_term = 0.0
        self._last_err_px = err_px

        # Combined PID output (negative because negative rad = camera turns right)
        delta = -(p_term + i_term + d_term)

        # Clamp step
        delta = max(-self.max_step, min(self.max_step, delta))

        new_angle = self._current_angle + delta
        clamped_angle = max(self.min_angle, min(self.max_angle, new_angle))

        self.get_logger().info(
            f"[PID] err={err_px:.1f}px, P={p_term:.4f}, I={i_term:.4f}, D={d_term:.4f}, "
            f"delta={delta:.4f}, angle: {self._current_angle:.3f} -> {clamped_angle:.3f}"
        )

        self._move_to(clamped_angle)

    def _move_to(self, angle: float):
        self._current_angle = float(angle)

        traj = JointTrajectory()
        traj.joint_names = [self.joint_name]

        pt = JointTrajectoryPoint()
        pt.positions = [self._current_angle]
        sec = int(self.cmd_time_sec)
        nsec = int((self.cmd_time_sec - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = nsec
        traj.points = [pt]

        self.traj_pub.publish(traj)
        self._trajectory_publish_count += 1


def main():
    rclpy.init()
    node = YoloPersonTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()