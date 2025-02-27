#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
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
            
        self.detection_sub = message_filters.Subscriber(
            self, Detection2DArray, 'detection_box', qos_profile=qos)
        
        # 时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub],
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
            
    def combined_callback(self, image_msg, detection_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 获取时间戳
            msg_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 添加时间戳
            self.add_timestamps(cv_image, msg_time, current_time)
            
            # 添加日志显示检测框数量
            self.get_logger().info(f'接收到 {len(detection_msg.detections)} 个检测框')
            
            # 绘制检测框
            if detection_msg.detections:
                for i, detection in enumerate(detection_msg.detections):
                    # 从检测框中获取中心点和尺寸
                    center_x = int(detection.bbox.center.position.x)
                    center_y = int(detection.bbox.center.position.y)
                    width = int(detection.bbox.size_x)
                    height = int(detection.bbox.size_y)
                    
                    # 获取置信度（从theta字段）
                    confidence = detection.bbox.center.theta
                    
                    # 计算边界框的左上角和右下角坐标
                    x1 = int(center_x - width / 2)
                    y1 = int(center_y - height / 2)
                    x2 = int(center_x + width / 2)
                    y2 = int(center_y + height / 2)
                    
                    # 确保坐标在图像范围内
                    h, w = cv_image.shape[:2]
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(w-1, x2)
                    y2 = min(h-1, y2)
                    
                    # 根据置信度调整颜色（置信度越高，颜色越绿）
                    # 从红(0,0,255)到绿(0,255,0)
                    green = int(255 * confidence)
                    red = int(255 * (1 - confidence))
                    color = (0, green, red)
                    
                    # 绘制边界框
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                    
                    # 显示置信度
                    conf_text = f"Person: {confidence:.2f}"
                    cv2.putText(cv_image, conf_text, (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    
                    # 显示中心点坐标
                    center_text = f"({center_x}, {center_y})"
                    cv2.putText(cv_image, center_text, (center_x + 10, center_y),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # 记录检测信息到日志
                    self.get_logger().info(f'检测框 {i}: 置信度={confidence:.2f}, 位置=({x1},{y1})-({x2},{y2})')
            else:
                # 当没有检测到物体时，显示"No people detected"文字
                h, w = cv_image.shape[:2]
                # 计算文本位置 - 在图像中央
                text = "No people detected"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1.5
                thickness = 3
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                text_x = (w - text_size[0]) // 2
                text_y = (h + text_size[1]) // 2
                
                # 添加半透明背景使文字更清晰
                # 先绘制一个黑色矩形作为背景
                padding = 20  # 文字周围的填充
                cv2.rectangle(cv_image, 
                            (text_x - padding, text_y - text_size[1] - padding),
                            (text_x + text_size[0] + padding, text_y + padding),
                            (0, 0, 0), -1)  # -1表示填充矩形
                
                # 绘制文字 (红色)
                cv2.putText(cv_image, text, (text_x, text_y),
                          font, font_scale, (0, 0, 255), thickness)
                
                self.get_logger().info('没有检测到物体，显示"No people detected"')
            
            # 显示最终图像
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'绘制检测框时出错: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

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