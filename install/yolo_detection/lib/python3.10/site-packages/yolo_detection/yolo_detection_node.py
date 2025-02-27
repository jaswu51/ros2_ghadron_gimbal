#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # 只保留检测框发布者
        self.detection_pub = self.create_publisher(Detection2DArray, 'detection_box', 10)
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        
    def image_callback(self, msg):
        try:
            # 转换ROS图像消息到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 运行YOLO检测
            results = self.model(cv_image)
            
            # 创建检测消息数组
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # 处理检测结果
            if len(results) > 0:
                result = results[0]
                boxes = result.boxes
                
                if len(boxes) > 0:
                    # 找到最大的边界框
                    areas = []
                    for box in boxes:
                        x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                        area = (x2 - x1) * (y2 - y1)
                        areas.append(area)
                    
                    largest_box_idx = np.argmax(areas)
                    box = boxes[largest_box_idx]
                    
                    # 获取边界框坐标并转换为float类型
                    x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                    
                    # 创建Detection2D消息
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # 设置边界框中心和大小
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    detection.bbox.center.theta = 0.0
                    
                    # 添加类别ID
                    if hasattr(box, 'cls'):
                        detection.id = str(int(box.cls[0]))
                    
                    detection_array.detections.append(detection)
            
            # 发布检测结果
            self.detection_pub.publish(detection_array)
            
        except Exception as e:
            self.get_logger().error(f'Error in detection: {str(e)}')
            
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YoloDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 