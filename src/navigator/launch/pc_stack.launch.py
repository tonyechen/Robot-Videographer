"""
PC Stack Launch File
--------------------
Starts the full PC-side pipeline in order:
  1. yolo          (YOLO detection)
  2. yolo2motor    (detection → motor angle, delayed 2 s)
  3. filtered_scan + SLAM + Nav2 + RViz (delayed 4 s)
  4. navigator     (Nav2 goal publisher, delayed 9 s = filtered_scan + 5 s)

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

    # ── 1. YOLO (starts immediately) ─────────────────────────────────────────
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_bringup_dir, 'launch', 'yolov11.launch.py')
        ),
        launch_arguments={
            'input_image_topic': '/image_raw',
        }.items()
    )

    # ── 2. yolo2motor (2 s after YOLO) ───────────────────────────────────────
    yolo2motor_node = TimerAction(
        period=2.0,
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

    # ── 3. filtered_scan + SLAM + Nav2 + RViz (4 s after start) ─────────────
    filtered_scan_node = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('filtered_scan'),
                        'launch',
                        'filtered_scan.launch.py',
                    )
                )
            )
        ]
    )

    # ── 4. navigator (4 s + 5 s = 9 s after start) ───────────────────────────
    navigator_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='navigator',
                executable='navigator',
                name='gix_navigator',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        yolo_launch,
        yolo2motor_node,
        filtered_scan_node,
        navigator_node,
    ])
