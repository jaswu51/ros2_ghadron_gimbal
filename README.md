# ROS2 Gimbal Control Workspace
This is used for Gremsy G-Hadron Gimbal
## Build and Run the Workspace
```bash
cd ros2_gremsy_gimbal_control
colcon build --cmake-args -DGHADRON=1
source install/setup.bash
```

ros2 run gimbal_angle_control gimbal_angle_control_node
ros2 run gimbal_status gimbal_status_node
ros2 run image_publisher image_publisher_node rtsp://10.3.1.124:8554/ghadron
ros2 run stream_publisher stream_node --ros-args -p rtsp_url:="rtsp://10.3.1.124:8554/ghadron" -p width:=1280 -p height:=720
ros2 run yolo_detection yolo_detection_node
ros2 run human_tracking tracking_node
ros2 run image_viewer image_viewer_node


## Function moduels

### Gimbal bringup
launch all the packages in the gimbal system
```bash
ros2 launch gimbal_bringup gimbal_system.launch.py
```
The rest of the nodes are automatically launched by gimbal_system.launch.py

### Gimbal Angle Control
```bash
ros2 run gimbal_angle_control gimbal_angle_control_node
```
Send control commands (pitch=0, row=0, yaw=90).
The range of the pitch is -90 pointing down to 90 pointing up.
The range of the yaw is -120 to the left to 120 to the right. But the gimbal may lock if you switch the direction too fast like -120 to 120 directly. It is better to do it incrementally like by 10 degrees each time.
```bash
ros2 topic pub /gimbal_angles geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 90.0}"
```

### Get gimbal status
```bash
ros2 run gimbal_status gimbal_status_node
```

View topic information
```bash
ros2 topic echo /gimbal_attitude
```
### eo image
```bash
ros2 run eo_image eo_image_node
```
### ir image
```bash
ros2 run ir_image ir_image_node
```

### Zoom control for ir camera
```bash
ros2 run ir_zoom ir_zoom_node
```
set zoom range 50% for ir camera, 0 means no zoom in, 100 means 8x zoom in.
```bash
ros2 topic pub /ir_zoom/range std_msgs/msg/Float32 "data: 50.0" # 0.0% to 100.0%
```

### Zoom control for eo camera
```bash
ros2 run eo_zoom eo_zoom_node
```
set zoom range 50% for eo camera, 0 means no zoom in, 100 means 12x zoom in.
```bash
ros2 topic pub /eo_zoom/range std_msgs/msg/Float32 "data: 50.0" # 0.0% to 100.0%
```

### YOLO detection
```bash
ros2 run yolo_detection yolo_detection_node
```
check the published topics
```bash
ros2 topic info /detection_box
ros2 topic info /detection_box_center
```
### Human tracking
```bash
ros2 run human_tracking tracking_node
```
check the published topics
```bash
ros2 topic info /gimbal_angles
```
## Other functions
### Check the streaming video
image_publisher is a node that publishes the streaming video to the topic /image_raw, image_viewer is a node that subscribes to the topic /image_raw and displays the video. We don't use them for now because the streaming video contains ir and eo overlapped and not being able to display separately.
```bash
ros2 run image_publisher image_publisher_node rtsp://10.3.1.124:8554/ghadron
ffmpeg -y -i rtsp://10.3.1.124:8554/ghadron 1 do.jpg
ros2 topic echo /image_raw
```
## Warnings


### hard-coded path prefix
needed to be changed to if code is migrated to other machine
```bash
/home/dtc-mrsd/Downloads/ros2_gimbal_ws/
```



# 构建包
cd ros2_gimbal_ws
colcon build --packages-select stream_publisher

# 源一下环境
source install/setup.bash

# 运行节点
ros2 run stream_publisher stream_node

# 使用参数运行
ros2 run stream_publisher stream_node --ros-args -p rtsp_url:="rtsp://10.3.1.124:8554/ghadron" -p width:=1280 -p height:=720