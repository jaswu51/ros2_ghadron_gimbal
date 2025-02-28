#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool
import time

class HumanTrackingNode(Node):
    def __init__(self):
        super().__init__('human_tracking_node')
        
        # 订阅检测框和云台姿态
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'detection_box',
            self.detection_callback,
            10
        )
        
        self.attitude_sub = self.create_subscription(
            Point,  # 注意：gimbal_attitude使用Point消息类型
            'gimbal_attitude',
            self.attitude_callback,
            10
        )
        
        # 订阅图像以获取分辨率
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        
        # 订阅航点等待状态
        self.waypoint_sub = self.create_subscription(
            Bool,
            'spin_survey',
            self.waypoint_callback,
            10
        )
        
        # 发布云台控制命令
        self.gimbal_pub = self.create_publisher(
            Vector3,
            'gimbal_angles',
            10
        )
        
        # 图像参数（将在收到第一帧图像时更新）
        self.image_width = None
        self.image_height = None
        self.last_image_timestamp = None
        
        # 云台控制参数
        self.angle_step =3.0  # 每次调整5度
        
        self.error_scale = 10  # 偏移阈值（图像尺寸的5%）
        # 当前云台角度（从attitude_callback更新）
        self.pitch = None  # 从x获取
        self.roll = None   # 从y获取
        self.yaw = None    # 从z获取
        
        # 扫描相关参数
        self.spin_survey = True  # 是否在等待航点
        self.scanning = False  # 是否正在扫描
        self.scan_direction = 1  # 扫描方向：1为向右，-1为向左
        self.scan_step = 10.0  # 扫描时每次移动的角度
        self.last_scan_time = 0.0  # 上次扫描的时间
        self.scan_interval = 2  # 扫描间隔时间（秒）
        
        self.get_logger().info('Human tracking node initialized with scanning capability')

    def image_callback(self, msg):
        """更新图像分辨率和时间戳"""
        self.image_width = msg.width
        self.image_height = msg.height
        self.last_image_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def attitude_callback(self, msg):
        """更新当前云台姿态"""
        self.pitch = msg.x  # pitch from x
        self.roll = msg.y   # roll from y
        self.yaw = msg.z    # yaw from z
        self.get_logger().debug(f'Current attitude - Pitch: {self.pitch:.2f}, Roll: {self.roll:.2f}, Yaw: {self.yaw:.2f}')

    def waypoint_callback(self, msg):
        """更新航点等待状态"""
        self.spin_survey = msg.data
        if self.spin_survey:
            self.get_logger().info('开始等待航点，启动人员扫描模式')
        else:
            self.get_logger().info('航点等待结束，停止扫描')
            self.scanning = False

    def scan_for_people(self):
        """在无人时扫描区域寻找人员"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 检查是否到达扫描间隔
        if current_time - self.last_scan_time < self.scan_interval:
            return
            
        self.last_scan_time = current_time
        
        # 定义扫描序列: (pitch, roll, yaw)
        scan_sequence = [
            (-45.0, 0.0, -120.0), # 向左扫描
            (-45.0, 0.0, 0.0),    # 起始位置
            (-45.0, 0.0, 120.0),  # 向右扫描
            (-90.0, 0.0, 0.0),    # 向下看
        ]
        
        # 获取当前扫描点索引
        if not hasattr(self, 'scan_seq_index'):
            self.scan_seq_index = 0
        
        # 获取目标位置
        target_pitch, target_roll, target_yaw = scan_sequence[self.scan_seq_index]
        
        # 发送云台命令
        gimbal_cmd = Vector3()
        gimbal_cmd.x = float(target_pitch)  
        gimbal_cmd.y = float(target_roll)   
        gimbal_cmd.z = float(target_yaw)    
        
        self.gimbal_pub.publish(gimbal_cmd)
        self.get_logger().info(f'扫描位置 {self.scan_seq_index+1}/{len(scan_sequence)}: '
                               f'pitch={target_pitch:.1f}°, roll={target_roll:.1f}°, yaw={target_yaw:.1f}°')
        
        # 更新索引到下一个扫描点
        self.scan_seq_index = (self.scan_seq_index + 1) % len(scan_sequence)

    def detection_callback(self, msg):
        try:
            # 确保我们已经获得图像分辨率和云台姿态
            if self.image_width is None or self.image_height is None or self.pitch is None:
                self.get_logger().warn('Waiting for image resolution or gimbal attitude...')
                return
            self.scanning = True
            self.scan_for_people()
            # 检查是否有检测结果
            if not msg.detections:
                self.get_logger().debug('No detections received')
                
                # 如果在等待航点且没有检测到人，则开始扫描
                if self.spin_survey:
                    self.scanning = True
                    self.scan_for_people()
                return
            
            # 如果检测到人，停止扫描
            self.scanning = False
            
            # 获取当前时间和检测消息时间戳
            current_time = self.get_clock().now().nanoseconds / 1e9
            detection_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 检查检测消息是否在2秒内
            if current_time - detection_timestamp > 2.0:
                self.get_logger().warn(f'Detection too old: {current_time - detection_timestamp:.2f}s')
                return
                
            # 检查图像和检测消息时间戳是否匹配（相差不超过1秒）
            if self.last_image_timestamp is None or abs(detection_timestamp - self.last_image_timestamp) > 2.0:
                self.get_logger().warn(f'Image and detection timestamps mismatch: {abs(detection_timestamp - self.last_image_timestamp):.2f}s')
                return
                
            # 获取第一个检测框的中心点位置
            detection = msg.detections[0]
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            
            # 如果中心点是默认值(0,0)，不进行处理
            if center_x == 0.0 and center_y == 0.0:
                return
                
            # 计算偏差百分比（相对于整个图像尺寸）
            error_x_percent = (center_x / self.image_width) * 100
            error_y_percent = (center_y / self.image_height) * 100
            
            # 计算新的目标角度
            target_pitch = self.pitch
            target_yaw = self.yaw
            
            self.get_logger().debug(f"error_x_percent: {error_x_percent}, error_y_percent: {error_y_percent}")
            
            # 如果偏差超过error_scale则调整角度
            if abs(error_y_percent - 50) > self.error_scale:  # 相对于图像中心的50%位置
                if error_y_percent < 50 - self.error_scale:  # 目标在上方
                    target_pitch += self.angle_step
                elif error_y_percent > 50 + self.error_scale:  # 目标在下方
                    target_pitch -= self.angle_step
            
            if abs(error_x_percent - 50) > self.error_scale:  # 相对于图像中心的50%位置
                if error_x_percent < 50 - self.error_scale:  # 目标在左边
                    target_yaw -= self.angle_step
                elif error_x_percent > 50 + self.error_scale:  # 目标在右边
                    target_yaw += self.angle_step
            
            self.get_logger().debug(f"target_pitch: {target_pitch}, target_yaw: {target_yaw}")
            
            # 限制云台角度范围之前记录超出限制的情况
            if target_pitch > 90.0 or target_pitch < -90.0:
                self.get_logger().warn(f'目标Pitch角度 {target_pitch:.1f}° 超出限制范围 [-90°, 90°]')
                    
            if target_yaw > 120.0 or target_yaw < -120.0:
                self.get_logger().warn(f'目标Yaw角度 {target_yaw:.1f}° 超出限制范围 [-120°, 120°]')

            # 限制云台角度范围
            target_pitch = max(min(target_pitch, 90.0), -90.0)   # -90° (向下) 到 90° (向上)
            target_yaw = max(min(target_yaw, 120.0), -120.0)    # -120° (向左) 到 120° (向右)
            
            # 只有当角度变化超过0.1度时才发送命令
            if (abs(target_pitch - self.pitch) > 0.1 or 
                abs(target_yaw - self.yaw) > 0.1):
                
                # 创建并发布云台控制命令
                gimbal_cmd = Vector3()
                gimbal_cmd.x = float(target_pitch)
                gimbal_cmd.y = float(self.roll)
                gimbal_cmd.z = float(target_yaw)
                
                self.gimbal_pub.publish(gimbal_cmd)
                
                # 输出日志，显示时间戳信息
                self.get_logger().info(
                    f'已锁定目标: x={error_x_percent:.1f}%, y={error_y_percent:.1f}%, '
                    f'Angles: pitch={target_pitch:.1f}°, yaw={target_yaw:.1f}°, '
                    f'Delay: {current_time - detection_timestamp:.2f}s'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in tracking: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackingNode()
    try:
        rclpy.spin(node)  # 这会让节点持续运行并处理回调
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()