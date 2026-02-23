from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1) Include the TurtleBot3 GIX hardware launch
    tb3_gix_bringup_share = get_package_share_directory('turtlebot3_gix_bringup')
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb3_gix_bringup_share, 'launch', 'hardware.launch.py'])
        )
    )

    # 2) USB camera node
    image_width = 640
    image_height = 360

    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': image_width,
            'image_height': image_height,
        }],
        remappings=[
            ('/image_raw', '/camera/rgb/image_raw'),
        ],
    )

    # 3) YOLO -> motor tracker
    yolo2motor = Node(
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
            'image_width': image_width,

            # control tuning
            'control_rate_hz': 10.0,
            'deadband_px': 12,
            'kp': 0.0035,          # rad/pixel (tune)
            'max_step': 0.08,
            'cmd_time_sec': 0.15,

            # joint limits
            'min_angle': -1.6,
            'max_angle': 1.6,

            # behavior if person disappears
            'lost_timeout_sec': 0.7,
            'return_to_center_on_lost': False,
            'center_angle': 0.0,
        }]
    )

    return LaunchDescription([
        hardware_launch,
        usb_cam,
        yolo2motor,
    ])
