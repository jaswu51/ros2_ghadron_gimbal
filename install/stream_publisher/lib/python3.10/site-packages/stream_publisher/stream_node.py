#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np
import time
import threading
import signal
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class StreamPublisher(Node):
    def __init__(self):
        super().__init__('stream_publisher')
        
        # 声明参数
        self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8554/ghadron')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('retry_max', 10)
        self.declare_parameter('retry_delay', 2.0)
        
        # 获取参数
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.WIDTH = self.get_parameter('width').value
        self.HEIGHT = self.get_parameter('height').value
        self.retry_max = self.get_parameter('retry_max').value
        self.retry_delay = self.get_parameter('retry_delay').value
        
        # 运行状态控制
        self.is_running = True
        self.retry_count = 0
        self.process = None
        
        # 创建QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Image, 'image_raw', qos)
        self.bridge = CvBridge()
        
        # 启动RTSP流处理线程
        self.stream_thread = threading.Thread(target=self.process_stream)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
        # 启动监控线程
        self.monitor_thread = threading.Thread(target=self.monitor_process)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def get_ffmpeg_cmd(self):
        """获取FFMPEG命令行配置"""
        return [
            "ffmpeg",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-rtsp_transport", "tcp",  # 使用TCP而非UDP，更稳定
            "-stimeout", "5000000",    # 5秒连接超时
            "-i", self.rtsp_url,
            "-vsync", "0",
            "-copyts",
            "-vf", f"fps=30,scale={self.WIDTH}:{self.HEIGHT}",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-"
        ]
    
    def start_ffmpeg(self):
        """启动FFMPEG进程"""
        try:
            cmd = self.get_ffmpeg_cmd()
            self.get_logger().info(f'启动FFMPEG命令: {" ".join(cmd)}')
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            
            if self.process.poll() is None:
                self.get_logger().info(f'成功启动FFMPEG，URL: {self.rtsp_url}')
                return True
            else:
                self.get_logger().error('FFMPEG进程启动后立即退出')
                return False
                
        except Exception as e:
            self.get_logger().error(f'启动FFMPEG失败: {str(e)}')
            return False
    
    def monitor_process(self):
        """监控FFMPEG进程状态"""
        while self.is_running:
            # 检查进程是否存在且运行
            if self.process and self.process.poll() is not None:
                self.get_logger().warn('FFMPEG进程已终止，将重新启动...')
                if self.retry_count < self.retry_max:
                    # 通知流处理线程重启FFMPEG
                    with threading.Lock():
                        self.process = None
                else:
                    self.get_logger().error(f'达到最大重试次数 ({self.retry_max})，停止重试')
            time.sleep(1.0)
    
    def process_stream(self):
        """处理RTSP流并发布图像"""
        frame_size = self.WIDTH * self.HEIGHT * 3
        
        while self.is_running:
            # 如果进程不存在或已终止，尝试启动
            if not self.process or self.process.poll() is not None:
                # 计算重试延迟（使用指数退避）
                retry_wait = self.retry_delay * (1.5 ** min(self.retry_count, 10))
                self.retry_count += 1
                
                self.get_logger().info(f'尝试启动FFMPEG (尝试 {self.retry_count}/{self.retry_max})...')
                if self.retry_count > 1:
                    self.get_logger().info(f'等待 {retry_wait:.1f} 秒后重试...')
                    time.sleep(retry_wait)
                
                if not self.start_ffmpeg():
                    if self.retry_count >= self.retry_max:
                        self.get_logger().error(f'达到最大重试次数 ({self.retry_max})，停止尝试')
                        break
                    continue
                else:
                    # 成功启动，重置重试计数
                    self.retry_count = 0
            
            # 处理视频帧
            try:
                raw_frame = self.process.stdout.read(frame_size)
                
                if not raw_frame:
                    self.get_logger().warn('读取到空帧，检查FFMPEG状态...')
                    time.sleep(0.1)
                    continue
                
                # 检查帧大小是否正确
                if len(raw_frame) != frame_size:
                    self.get_logger().warn(f'帧大小不匹配: 预期 {frame_size}，实际 {len(raw_frame)}')
                    continue
                    
                # 转换并发布帧
                frame = np.frombuffer(raw_frame, np.uint8).reshape((self.HEIGHT, self.WIDTH, 3))
                
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                
                self.publisher_.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'处理帧时出错: {str(e)}')
                time.sleep(0.1)
    
    def shutdown(self):
        """干净地关闭节点"""
        self.get_logger().info('关闭stream_publisher节点...')
        self.is_running = False
        
        # 等待线程结束
        if hasattr(self, 'stream_thread') and self.stream_thread.is_alive():
            self.stream_thread.join(timeout=2.0)
            
        if hasattr(self, 'monitor_thread') and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        
        # 终止FFMPEG进程
        if self.process and self.process.poll() is None:
            self.get_logger().info('终止FFMPEG进程...')
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('FFMPEG进程未响应，强制终止')
                self.process.kill()
        
        self.get_logger().info('stream_publisher节点已安全关闭')

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = StreamPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"错误: {str(e)}")
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