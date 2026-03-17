import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from yolo_msgs.msg import DetectionArray as YoloDetectionArray


class YoloPersonTracker(Node):
    """
    Pan a camera joint to keep a locked 'person' detection centered.
    Uses PID control on the pixel error to drive the joint angle.
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
        self.declare_parameter('deadband_px', 30)
        self.declare_parameter('cmd_time_sec', 0.15)
        self.declare_parameter('min_angle', -1.6)
        self.declare_parameter('max_angle', 1.6)
        self.declare_parameter('center_angle', 0.0)
        self.declare_parameter('lost_timeout_sec', 5.0)
        self.declare_parameter('return_to_center_on_lost', False)
        self.declare_parameter('lock_proximity_px', 400)
        self.declare_parameter('detection_smoothing', 0.3)
        self.declare_parameter('kp', 0.003)   # proportional gain
        self.declare_parameter('ki', 0.0)     # integral gain
        self.declare_parameter('kd', 0.01)    # derivative gain — dampens overshoot
        self.declare_parameter('max_step', 0.05)          # max angle change per update (rad)
        self.declare_parameter('integral_limit', 30.0)    # clamp integral accumulation (px)

        # ---- Load params ----
        self.detections_topic = self.get_parameter('detections_topic').value
        self.controller_topic = self.get_parameter('controller_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        self.label = self.get_parameter('label').value
        self.min_score = float(self.get_parameter('min_score').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.deadband_px = int(self.get_parameter('deadband_px').value)
        self.cmd_time_sec = float(self.get_parameter('cmd_time_sec').value)
        self.min_angle = float(self.get_parameter('min_angle').value)
        self.max_angle = float(self.get_parameter('max_angle').value)
        self.center_angle = float(self.get_parameter('center_angle').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout_sec').value)
        self.return_on_lost = bool(self.get_parameter('return_to_center_on_lost').value)
        self.lock_proximity_px = int(self.get_parameter('lock_proximity_px').value)
        self.detection_smoothing = float(self.get_parameter('detection_smoothing').value)
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_step = float(self.get_parameter('max_step').value)
        self.integral_limit = float(self.get_parameter('integral_limit').value)

        # ---- State ----
        self._last_seen_time = 0.0
        self._smoothed_cx_px = None
        self._current_angle = self.center_angle

        # ---- PID state ----
        self._prev_err_px = 0.0
        self._integral_px = 0.0
        self._prev_time = None

        # ---- Lock-on state ----
        self._locked = False
        self._locked_cx = None
        self._locked_cy = None

        # ---- Debug counters ----
        self._detection_msg_count = 0
        self._matching_detection_count = 0
        self._trajectory_publish_count = 0

        # ---- Pub/Sub ----
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)
        self.sub = self.create_subscription(
            YoloDetectionArray, self.detections_topic, self.on_detections, 10
        )

        self.status_timer = self.create_timer(3.0, self.log_status)

        self.get_logger().info(
            f"=== YoloPersonTracker Started (PID) ===\n"
            f"  Tracking label: '{self.label}'\n"
            f"  kp={self.kp}, ki={self.ki}, kd={self.kd}\n"
            f"  max_step={self.max_step} rad, deadband={self.deadband_px}px\n"
            f"  Angle limits: [{self.min_angle}, {self.max_angle}] rad"
        )

    def _reset_pid(self):
        self._prev_err_px = 0.0
        self._integral_px = 0.0
        self._prev_time = None

    def log_status(self):
        now = time.time()
        seen_ago = now - self._last_seen_time if self._last_seen_time > 0 else float('inf')

        if self._locked and (now - self._last_seen_time) > self.lost_timeout:
            self.get_logger().warn(
                f"[CONTROL] Target lost after {self.lost_timeout:.1f}s — unlocking."
            )
            self._locked = False
            self._locked_cx = None
            self._locked_cy = None
            self._smoothed_cx_px = None
            self._reset_pid()
            if self.return_on_lost:
                self._move_to(self.center_angle)

        lock_str = f"LOCKED @ ({self._locked_cx:.0f},{self._locked_cy:.0f})" if self._locked else "UNLOCKED"
        self.get_logger().info(
            f"[STATUS] {lock_str} | Det msgs: {self._detection_msg_count}, "
            f"Matches: {self._matching_detection_count}, "
            f"Traj published: {self._trajectory_publish_count}, "
            f"Last seen: {seen_ago:.1f}s ago, "
            f"Angle: {self._current_angle:.3f} rad"
        )

        self._detection_msg_count = 0
        self._matching_detection_count = 0
        self._trajectory_publish_count = 0

    def on_detections(self, msg: YoloDetectionArray):
        self._detection_msg_count += 1

        if len(msg.detections) == 0:
            return

        candidates = []
        for det in msg.detections:
            if det.class_name != self.label:
                continue
            if float(det.score) < self.min_score:
                continue
            try:
                cx = float(det.bbox.center.position.x)
                cy = float(det.bbox.center.position.y)
                area = float(det.bbox.size.x) * float(det.bbox.size.y)
            except Exception:
                continue
            candidates.append((cx, cy, float(det.score), area))

        if not candidates:
            return

        if not self._locked:
            center_x = 0.5 * self.image_width
            best = min(candidates, key=lambda c: abs(c[0] - center_x))
            chosen_cx, chosen_cy, chosen_score, _ = best
            self._locked = True
            self._reset_pid()
            self.get_logger().info(
                f"[LOCK] Acquired target at x={chosen_cx:.1f}, y={chosen_cy:.1f} "
                f"(score={chosen_score:.2f})"
            )
        else:
            def dist(c):
                return ((c[0] - self._locked_cx) ** 2 + (c[1] - self._locked_cy) ** 2) ** 0.5
            chosen_cx, chosen_cy, chosen_score, _ = min(candidates, key=dist)

        self._locked_cx = chosen_cx
        self._locked_cy = chosen_cy
        self._last_seen_time = time.time()
        self._matching_detection_count += 1

        # Low-pass filter
        if self._smoothed_cx_px is None:
            self._smoothed_cx_px = chosen_cx
        else:
            alpha = self.detection_smoothing
            self._smoothed_cx_px = alpha * chosen_cx + (1 - alpha) * self._smoothed_cx_px

        # ---- PID control ----
        center_px = 0.5 * self.image_width
        err_px = self._smoothed_cx_px - center_px

        if abs(err_px) <= self.deadband_px:
            self._reset_pid()
            return

        now = time.time()
        dt = (now - self._prev_time) if self._prev_time is not None else 0.1
        dt = max(0.01, min(dt, 1.0))  # clamp dt to sane range
        self._prev_time = now

        # Integral with anti-windup
        self._integral_px += err_px * dt
        self._integral_px = max(-self.integral_limit, min(self.integral_limit, self._integral_px))

        # Derivative
        d_err = (err_px - self._prev_err_px) / dt
        self._prev_err_px = err_px

        # PID output — negative because right of center (positive err) → decrease angle
        step = -(self.kp * err_px + self.ki * self._integral_px + self.kd * d_err)
        step = max(-self.max_step, min(self.max_step, step))

        target_angle = self._current_angle + step
        target_angle = max(self.min_angle, min(self.max_angle, target_angle))

        self.get_logger().info(
            f"[PID] err={err_px:.1f}px  P={self.kp*err_px:.4f}  "
            f"I={self.ki*self._integral_px:.4f}  D={self.kd*d_err:.4f}  "
            f"step={step:.3f} -> angle={target_angle:.3f} rad"
        )

        self._move_to(target_angle)

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
