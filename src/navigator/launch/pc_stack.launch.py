"""
PC Stack Launch File
--------------------
Starts the full PC-side pipeline in order:
  1. filtered_scan + SLAM + Nav2 + RViz (starts immediately)
  2. navigator     (Nav2 goal publisher, delayed 5 s)
  3. yolo          (YOLO detection, delayed 7 s)
  4. yolo2motor    (detection → motor angle, delayed 9 s)

Hardware (camera, LiDAR, motors) is launched separately on the robot via SSH.

Usage:
    ros2 launch navigator pc_stack.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    yolo_bringup_dir = get_package_share_directory('yolo_bringup')

    # ── 1. filtered_scan + SLAM + Nav2 + RViz (starts immediately) ───────────
    filtered_scan_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('filtered_scan'),
                'launch',
                'filtered_scan.launch.py',
            )
        )
    )

    # ── 2. navigator (5 s after start) ───────────────────────────────────────
    navigator_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='navigator',
                executable='navigator',
                name='gix_navigator',
                output='screen',
            )
        ]
    )

    # ── 3. YOLO (7 s after start) ─────────────────────────────────────────────
    yolo_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(yolo_bringup_dir, 'launch', 'yolov11.launch.py')
                ),
                launch_arguments={
                    'input_image_topic': '/image_raw',
                    'namespace': 'yolo',
                    'model': 'yolo11m.pt',
                    'device': 'cuda:0',
                    'enable': 'True',
                    'threshold': '0.5',
                    'image_reliability': '1',
                }.items()
            )
        ]
    )

    # ── 4. yolo2motor (9 s after start) ──────────────────────────────────────
    yolo2motor_node = TimerAction(
        period=9.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('yolo2motor'),
                        'launch',
                        'yolo2motor_launch.py',
                    )
                )
            )
        ]
    )

    return LaunchDescription([
        filtered_scan_node,
        navigator_node,
        yolo_launch,
        yolo2motor_node,
    ])
