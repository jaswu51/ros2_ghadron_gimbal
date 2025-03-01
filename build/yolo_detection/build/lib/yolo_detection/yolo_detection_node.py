#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os

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
        
        # 加载 YOLOv11x-pose 模型
        self.get_logger().info('正在加载 YOLOv11x-pose 模型...')
        model_path = "models/yolo11x-pose.pt"  # 确保路径正确
        
        # 检查模型文件是否存在
        if not os.path.exists(model_path):
            self.get_logger().error(f"模型文件 {model_path} 不存在！")
            # 可选：提供下载指令
            self.get_logger().info("请先下载模型: wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x-pose.pt -O models/yolo11x-pose.pt")
            raise FileNotFoundError(f"模型文件 {model_path} 不存在")
        
        # 加载模型
        self.model = YOLO(model_path)
        self.get_logger().info('YOLOv11x-pose 模型加载成功')
        
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
                
                # 只保留人类检测结果 (类别ID为0)
                person_boxes = []
                for box in boxes:
                    if hasattr(box, 'cls') and int(box.cls[0]) == 0:  # 0是COCO数据集中人类的类别ID
                        # 获取置信度
                        confidence = float(box.conf[0].cpu().numpy())
                        # 只保留置信度大于0.5的检测
                        if confidence > 0.6:
                            person_boxes.append(box)
                
                if len(person_boxes) > 0:
                    # 根据置信度大小排序，选择置信度最高的
                    confidences = [float(box.conf[0].cpu().numpy()) for box in person_boxes]
                    highest_conf_idx = np.argmax(confidences)
                    box = person_boxes[highest_conf_idx]
                    
                    # 获取边界框坐标并转换为float类型
                    x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                    
                    # 获取置信度
                    confidence = float(box.conf[0].cpu().numpy())
                    
                    # 创建Detection2D消息
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # 设置边界框中心和大小
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    detection.bbox.center.theta = confidence  # 注意：这里使用theta存储置信度值
                    
                    # 添加类别ID
                    if hasattr(box, 'cls'):
                        detection.id = str(int(box.cls[0]))
                    
                    self.get_logger().info(f'检测到人：置信度={confidence:.4f}, 位置=({x1:.1f},{y1:.1f})-({x2:.1f},{y2:.1f})')
                    
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