from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from datetime import datetime

def generate_launch_description():
    # 获取当前日期和时间
    now = datetime.now()
    
    # 按日创建文件夹
    day_folder = now.strftime("%Y-%m-%d")
    
    # 创建详细的时间戳文件名（年月日_时分秒）
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    
    # 构建完整路径: 基础路径/日期/年月日_时分秒
    base_dir = "/home/dtc-mrsd/Downloads/ros2_ghadron_gimbal/mcap_recording"
    day_dir = os.path.join(base_dir, day_folder)
    
    # 确保日期文件夹存在
    os.makedirs(day_dir, exist_ok=True)
    
    # 最终记录路径
    recording_path = os.path.join(day_dir, f"recording_{timestamp}")
    
    return LaunchDescription([
        LogInfo(msg=['Starting Gimbal System...']),
        
        # 云台控制节点
        Node(
            package='gimbal_angle_control',
            executable='gimbal_angle_control_node',
            name='gimbal_angle_control_node',
            output='screen'
        ),
        
        # 云台状态节点
        Node(
            package='gimbal_status',
            executable='gimbal_status_node',
            name='gimbal_status_node',
            output='screen'
        ),
        
        # 视频流发布节点
        Node(
            package='stream_publisher',
            executable='stream_node',
            name='stream_node',
            parameters=[{
                'rtsp_url': 'rtsp://10.3.1.124:8554/ghadron',
                'width': 1280,
                'height': 720
            }],
            output='screen'
        ),
        
        # 图像查看节点
        Node(
            package='image_viewer',
            executable='image_viewer_node',
            name='image_viewer_node',
            output='screen'
        ),

        # YOLO检测节点
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_detection_node',
            output='screen'
        ),
        
        # 人物追踪节点
        Node(
            package='human_tracking',
            executable='tracking_node',
            name='tracking_node',
            output='screen'
        ),
        
        # 数据记录
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '-o', recording_path,
                 '--storage', 'mcap',
                 '--max-bag-duration', '60',
                 '/image_raw',
                 '/detection_box',
                 '/gimbal_attitude',
                 '/gimbal_angles',
                 '/waypoint_waiting'],
            output='screen'
        ),
        
        LogInfo(msg=['All nodes have been started. Recording data to: ' + recording_path])
    ]) 