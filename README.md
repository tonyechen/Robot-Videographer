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

## yolo2motor — Person Tracking

`yolo2motor` subscribes to YOLO detections and pans the camera joint to keep a detected person centered in frame using a P controller.

**How it works:**
1. YOLO publishes detections on `/yolo/detections`
2. `yolo2motor` picks the highest-confidence `person` detection
3. It calculates the pixel error between the detection center and the image center
4. It sends a `JointTrajectory` command to `/gix_controller/joint_trajectory` to correct the angle

### Launch everything at once (hardware + camera + tracker):
```cmd
ros2 launch yolo2motor yolo2motor_launch.py
```

> **Note:** YOLO must be running separately on your PC:
> ```cmd
> ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/camera/rgb/image_raw
> ```

### Or run just the tracker node:
```cmd
ros2 run yolo2motor yolo2motor
```

### Key parameters (tunable at launch):
| Parameter | Default | Description |
|---|---|---|
| `label` | `person` | Object class to track |
| `min_score` | `0.5` | Minimum detection confidence |
| `image_width` | `640` | Must match camera stream width |
| `kp` | `0.0035` | P gain (radians per pixel) |
| `deadband_px` | `12` | Ignore errors smaller than this (pixels) |
| `max_step` | `0.08` | Max joint movement per update (radians) |
| `min_angle` / `max_angle` | `-1.6` / `1.6` | Joint travel limits (radians) |
| `lost_timeout_sec` | `0.7` | Seconds before target is considered lost |
| `return_to_center_on_lost` | `false` | Return to center when target is lost |