from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
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
        
        LogInfo(msg=['All nodes have been started.'])
    ]) 