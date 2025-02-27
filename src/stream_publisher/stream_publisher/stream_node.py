#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class StreamPublisher(Node):
    def __init__(self):
        super().__init__('stream_publisher')
        
        # 声明参数
        self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8554/ghadron')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        
        # 获取参数
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.WIDTH = self.get_parameter('width').value
        self.HEIGHT = self.get_parameter('height').value
        
        # 创建QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Image, 'image_raw', qos)
        self.bridge = CvBridge()
        
        # RTSP的FFMPEG命令
        self.FFMPEG_CMD = [
            "ffmpeg",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-rtsp_transport", "udp",
            "-i", self.rtsp_url,
            "-vsync", "0",
            "-copyts",
            "-vf", "fps=30",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-"
        ]
        
        try:
            self.process = subprocess.Popen(
                self.FFMPEG_CMD,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            self.get_logger().info(f'Started FFMPEG with RTSP URL: {self.rtsp_url}')
        except Exception as e:
            self.get_logger().error(f'Failed to start FFMPEG: {str(e)}')
            return
            
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30fps
        
    def process_frame(self):
        try:
            raw_frame = self.process.stdout.read(self.WIDTH * self.HEIGHT * 3)
            if not raw_frame:
                if self.process.poll() is not None:
                    self.get_logger().error("FFMPEG process died")
                    return
                return
                
            if len(raw_frame) != self.WIDTH * self.HEIGHT * 3:
                return
                
            frame = np.frombuffer(raw_frame, np.uint8).reshape((self.HEIGHT, self.WIDTH, 3))
            
            # 发布图像
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')
    
    def shutdown(self):
        """Clean shutdown of the node"""
        if hasattr(self, 'process'):
            self.process.terminate()
            self.process.wait()
        self.get_logger().info('Shutting down stream_publisher node')

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = StreamPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
