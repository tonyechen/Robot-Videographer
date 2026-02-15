# Robot-Videographer

Install dependencies for Ros2:
```cmd
rosdep install --from-paths src --ignore-src -r -y
```

Launching gix bringup:
```cmd
ros2 launch turtlebot3_gix_bringup hardware.launch.py
```

Motor Testing
```cmd
ros2 topic pub --once /gix_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['gix'], points: [{positions: [1.0], time_from_start: {sec: 2}}]}"
```

launch camera at 1080p 30fps:
```cmd
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video0" \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p framerate:=30.0 \
  -p pixel_format:="mjpeg2rgb"
```

launch camera for yolo (on turtlebot3):
```cmd
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video0" \
  -p image_width:=640 \
  -p image_height:=360 \
  -p framerate:=30.0 \
  -p pixel_format:="mjpeg2rgb"
```
launch yolo on pc:
```cmd
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/image_raw
```