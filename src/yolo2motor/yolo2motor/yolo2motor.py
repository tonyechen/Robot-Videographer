import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# mgonzs13/yolo_ros message types
from yolo_msgs.msg import DetectionArray as YoloDetectionArray


class YoloPersonTracker(Node):
    """
    Pan a camera joint to keep the best 'person' detection centered.
    Uses yolo_ros messages:
      det.class_name
      det.score
      det.bbox.center.position.x   (pixel)
    """

    def __init__(self):
        super().__init__('yolo_person_tracker')

        # ---- Parameters ----
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('controller_topic', '/gix_controller/joint_trajectory')
        self.declare_parameter('joint_name', 'gix')

        # Detection filter
        self.declare_parameter('label', 'person')
        self.declare_parameter('min_score', 0.5)

        # Camera geometry (must match camera stream)
        self.declare_parameter('image_width', 640)

        # Control loop
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('deadband_px', 12)

        # P controller: radians per pixel
        self.declare_parameter('kp', 0.0035)

        # Safety / smoothing
        self.declare_parameter('max_step', 0.08)          # max rad per update
        self.declare_parameter('cmd_time_sec', 0.15)      # trajectory time_from_start
        self.declare_parameter('min_angle', -1.6)
        self.declare_parameter('max_angle', 1.6)

        # Lost target behavior
        self.declare_parameter('lost_timeout_sec', 0.7)
        self.declare_parameter('return_to_center_on_lost', False)
        self.declare_parameter('center_angle', 0.0)

        # ---- Load params ----
        self.detections_topic = self.get_parameter('detections_topic').value
        self.controller_topic = self.get_parameter('controller_topic').value
        self.joint_name = self.get_parameter('joint_name').value

        self.label = self.get_parameter('label').value
        self.min_score = float(self.get_parameter('min_score').value)

        self.image_width = int(self.get_parameter('image_width').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.deadband_px = int(self.get_parameter('deadband_px').value)
        self.kp = float(self.get_parameter('kp').value)

        self.max_step = float(self.get_parameter('max_step').value)
        self.cmd_time_sec = float(self.get_parameter('cmd_time_sec').value)
        self.min_angle = float(self.get_parameter('min_angle').value)
        self.max_angle = float(self.get_parameter('max_angle').value)

        self.lost_timeout = float(self.get_parameter('lost_timeout_sec').value)
        self.return_on_lost = bool(self.get_parameter('return_to_center_on_lost').value)
        self.center_angle = float(self.get_parameter('center_angle').value)

        # ---- State ----
        self._last_seen_time = 0.0
        self._target_cx_px = None
        self._current_angle = self.center_angle

        # ---- Pub/Sub ----
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)
        self.sub = self.create_subscription(
            YoloDetectionArray, self.detections_topic, self.on_detections, 10
        )

        # ---- Timer loop ----
        period = 1.0 / max(self.control_rate_hz, 1e-6)
        self.timer = self.create_timer(period, self.control_step)

        self.get_logger().info(
            f"Tracking '{self.label}' on {self.detections_topic}; "
            f"publishing JointTrajectory to {self.controller_topic}; "
            f"image_width={self.image_width}, kp={self.kp}, rate={self.control_rate_hz}Hz"
        )

    def on_detections(self, msg: YoloDetectionArray):
        # Choose best matching detection by score
        best_score = -1.0
        best_cx = None

        for det in msg.detections:
            if det.class_name != self.label:
                continue
            if float(det.score) < self.min_score:
                continue

            # mgonzs13/yolo_ros bbox center x (pixels)
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

    def control_step(self):
        now = time.time()
        seen_recently = (now - self._last_seen_time) <= self.lost_timeout

        if not seen_recently:
            self._target_cx_px = None
            if self.return_on_lost:
                self._move_to(self.center_angle)
            return

        if self._target_cx_px is None:
            return

        center_px = 0.5 * self.image_width
        err_px = float(self._target_cx_px) - center_px  # + if target is to the right

        # Ignore tiny error to avoid jitter
        if abs(err_px) <= self.deadband_px:
            return

        # P control: pixels -> radians
        delta = self.kp * err_px

        # Clamp step
        if delta > self.max_step:
            delta = self.max_step
        elif delta < -self.max_step:
            delta = -self.max_step

        # If your joint moves the wrong way, flip sign:
        # delta = -delta

        new_angle = self._current_angle + delta
        new_angle = max(self.min_angle, min(self.max_angle, new_angle))
        self._move_to(new_angle)

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
