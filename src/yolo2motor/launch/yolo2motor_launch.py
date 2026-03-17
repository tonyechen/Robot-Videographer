from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo2motor',
            executable='yolo2motor',
            name='yolo2motor',
            output='screen',
            parameters=[{
                # topics / joint
                'detections_topic': '/yolo/detections',
                'controller_topic': '/gix_controller/joint_trajectory',
                'joint_name': 'gix',

                # track humans
                'label': 'person',
                'min_score': 0.5,

                # must match camera width
                'image_width': 640,

                # control tuning
                'deadband_px': 30,
                'cmd_time_sec': 0.15,

                # PID gains
                'kp': 0.008,             # proportional — drives toward center
                'ki': 0.0,               # integral — disabled until P+D is tuned
                'kd': 0.05,              # derivative — dampens overshoot
                'max_step': 0.06,        # max angle change per update (rad)
                'integral_limit': 30.0,  # anti-windup clamp (px)

                # joint limits
                'min_angle': -1.6,
                'max_angle': 1.6,
                'center_angle': 0.0,

                # behavior if person disappears
                'lost_timeout_sec': 5.0,
                'return_to_center_on_lost': False,

                # tracking
                'detection_smoothing': 0.4,
                'lock_proximity_px': 400,
            }]
        )
    ])
