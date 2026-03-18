"""
Launch file for person-following robot with filtered scan.
Starts: scan_filter -> SLAM toolbox -> Nav2 + RViz

Usage:
    ros2 launch filtered_scan navigation_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    filtered_scan_dir = get_package_share_directory('filtered_scan')
    turtlebot3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    
    # Path to our custom nav2 params
    nav2_params_file = os.path.join(filtered_scan_dir, 'config', 'nav2_params_filtered.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # 1. Scan filter node
    scan_filter_node = Node(
        package='filtered_scan',
        executable='filtered_scan',
        name='filtered_scan',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. SLAM Toolbox (delayed 2 seconds to let scan_filter start)
    slam_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[{
                    'scan_topic': '/scan_filtered',
                    'use_sim_time': use_sim_time,
                    'base_frame': 'base_footprint',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                }]
            )
        ]
    )

    # 3. Nav2 + RViz via turtlebot3_navigation2 (delayed 5 seconds)
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_nav2_dir, 'launch', 'navigation2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam': 'False',
                    'map': 'none',
                    'params_file': nav2_params_file,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        scan_filter_node,
        slam_node,
        nav2_launch,
    ])