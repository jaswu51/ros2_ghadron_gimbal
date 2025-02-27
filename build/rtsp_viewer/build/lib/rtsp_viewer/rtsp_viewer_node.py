#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RTSPViewerNode(Node):
    def __init__(self):
        super().__init__('rtsp_viewer_node')
        
        # 创建CV bridge
        self.bridge = CvBridge()
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 订阅image_publisher发布的话题
            self.image_callback,
            10
        )
        
        self.get_logger().info('RTSP viewer node initialized')

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 显示图像
            cv2.imshow("RTSP Stream", cv_image)
            cv2.waitKey(1)  # 1ms延迟
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = RTSPViewerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
