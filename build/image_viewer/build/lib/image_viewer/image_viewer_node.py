#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        
        # QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        
        # 创建订阅者
        self.image_sub = message_filters.Subscriber(
            self, Image, 'image_raw', qos_profile=qos)
            
        self.center_sub = message_filters.Subscriber(
            self, Point, 'bounding_box_center', qos_profile=qos)
        
        self.bounding_box_sub = message_filters.Subscriber(
            self, Point, 'bounding_box', qos_profile=qos)
        
        # 时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.bounding_box_sub, self.center_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.combined_callback)
        
        # 单独的图像回调
        self.image_only_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_only_callback,
            qos
        )
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_time = None
        
        # 创建显示窗口
        cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
        
    def add_timestamps(self, cv_image, msg_time, current_time):
        """添加时间戳到图像上"""
        # 消息时间戳
        msg_timestamp = f"Image Time: {msg_time:.3f}"
        cv2.putText(cv_image, msg_timestamp, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 当前时间戳
        now_timestamp = f"Current Time: {current_time:.3f}"
        cv2.putText(cv_image, now_timestamp, (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 计算延迟
        delay = current_time - msg_time
        delay_text = f"Delay: {delay:.3f}s"
        cv2.putText(cv_image, delay_text, (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
    def image_only_callback(self, image_msg):
        try:
            # 获取消息时间戳
            msg_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
            # 获取当前时间
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 添加时间戳
            self.add_timestamps(cv_image, msg_time, current_time)
            
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
            
    def combined_callback(self, image_msg, center_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 获取时间戳
            msg_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 添加时间戳
            self.add_timestamps(cv_image, msg_time, current_time)
            
            # 绘制目标中心
            center_x = int(center_msg.x)
            center_y = int(center_msg.y)
            cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.putText(cv_image, f"Center ({center_x}, {center_y})",
                       (center_x + 10, center_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in combined callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = ImageViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()