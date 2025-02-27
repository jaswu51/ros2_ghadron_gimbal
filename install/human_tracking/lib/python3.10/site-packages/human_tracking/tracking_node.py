#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image

class HumanTrackingNode(Node):
    def __init__(self):
        super().__init__('human_tracking_node')
        
        # 订阅检测中心点和云台姿态
        self.center_sub = self.create_subscription(
            Point,
            'detection_box_center',
            self.center_callback,
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
        
        # 发布云台控制命令
        self.gimbal_pub = self.create_publisher(
            Vector3,
            'gimbal_angles',
            10
        )
        
        # 图像参数（将在收到第一帧图像时更新）
        self.image_width = None
        self.image_height = None
        
        # 云台控制参数
        self.angle_step = 2.0  # 每次调整5度
        
        self.error_scale=5 # 偏移阈值（图像尺寸的5%）
        # 当前云台角度（从attitude_callback更新）
        self.pitch = None  # 从x获取
        self.roll = None   # 从y获取
        self.yaw = None    # 从z获取
        
        self.get_logger().info('Human tracking node initialized')

    def image_callback(self, msg):
        """更新图像分辨率"""
        self.image_width = msg.width
        self.image_height = msg.height

    def attitude_callback(self, msg):
        """更新当前云台姿态"""
        self.pitch = msg.x  # pitch from x
        self.roll = msg.y   # roll from y
        self.yaw = msg.z    # yaw from z
        self.get_logger().debug(f'Current attitude - Pitch: {self.pitch:.2f}, Roll: {self.roll:.2f}, Yaw: {self.yaw:.2f}')

    def center_callback(self, msg):
        try:
            # 确保我们已经获得图像分辨率
            if self.image_width is None or self.image_height is None:
                self.get_logger().warn('Waiting for image resolution...')
                return
            
            # 如果收到默认值(0,0)，不进行处理
            if msg.x == 0.0 and msg.y == 0.0:
                return
                
            # 计算偏差百分比（相对于整个图像尺寸）
            error_x_percent = (msg.x / self.image_width) * 100
            error_y_percent = (msg.y / self.image_height) * 100
            
            # 计算新的目标角度
            target_pitch = self.pitch
            target_yaw = self.yaw
            print(f"error_x_percent: {error_x_percent}, error_y_percent: {error_y_percent}")
            print(f"previus_target_pitch: {target_pitch}, previus_target_yaw: {target_yaw}")
            # 如果偏差超过error_scale则调整角度
            if abs(error_y_percent - 50) > self.error_scale:  # 相对于图像中心的50%位置
                if error_y_percent < self.error_scale:  # 目标在上方
                    target_pitch += self.angle_step
                elif error_y_percent > self.error_scale+50:  # 目标在下方
                    target_pitch -= self.angle_step
            
            if abs(error_x_percent - 50) > self.error_scale:  # 相对于图像中心的50%位置
                if error_x_percent < self.error_scale:  # 目标在左边
                    target_yaw -= self.angle_step
                elif error_x_percent > self.error_scale+50:  # 目标在右边
                    target_yaw += self.angle_step
            print(f"target_pitch: {target_pitch}, target_yaw: {target_yaw}")
            # 限制云台角度范围
            target_pitch = max(min(target_pitch, 90.0), -90.0)   # -90° (向下) 到 90° (向上)
            target_yaw = max(min(target_yaw, 120.0), -120.0)    # -120° (向左) 到 120° (向右)
            
            # 只有当角度变化超过0.1度时才发送命令
            if (abs(target_pitch - self.pitch) > 0.1 or 
                abs(target_yaw - self.yaw) > 0.1):
                
                # 创建并发布云台控制命令
                gimbal_cmd = Vector3()
                print(f"target_pitch: {target_pitch}, target_yaw: {target_yaw}")
                gimbal_cmd.x = float(target_pitch)
                gimbal_cmd.y = float(self.roll)
                gimbal_cmd.z = float(target_yaw)
                
                self.gimbal_pub.publish(gimbal_cmd)
                
                self.get_logger().info(
                    f'Position: x={error_x_percent:.1f}%, y={error_y_percent:.1f}%, '
                    f'Angles: pitch={target_pitch:.1f}°, yaw={target_yaw:.1f}°'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in tracking: {str(e)}')

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