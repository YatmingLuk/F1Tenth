#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class DriveToVescBridge(Node):
    def __init__(self):
        super().__init__('drive_to_vesc_bridge')

        # ===== 舵機校正參數 =====
        self.servo_left = 0.15
        self.servo_center = 0.50
        self.servo_right = 0.85

        # 要和 Pure Pursuit 裡的 max_steering 一致
        self.max_steering = 0.42

        # 如果發現左右相反，改成 True
        self.invert_steering = False

        # ===== 速度轉換參數 =====
        # /drive.speed 是 simulator 中的速度命令
        # /commands/motor/speed 對 VESC driver 通常是 ERPM
        self.speed_to_erpm_gain = 4000.0   
        self.max_erpm = 6000.0            

        # 如果馬達方向相反，改成 -1.0
        self.speed_direction = 1.0

        self.servo_pub = self.create_publisher(
            Float64,
            '/commands/servo/position',
            10
        )

        self.motor_pub = self.create_publisher(
            Float64,
            '/commands/motor/speed',
            10
        )

        self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )

        self.get_logger().info('drive_to_vesc_bridge started')

    def steering_to_servo(self, steering):
        if self.invert_steering:
            steering = -steering

        # 限制 steering 在可用範圍內
        steering = max(-self.max_steering, min(self.max_steering, steering))

        if steering >= 0:
            ratio = steering / self.max_steering
            servo = self.servo_center + ratio * (self.servo_right - self.servo_center)
        else:
            ratio = steering / self.max_steering
            servo = self.servo_center + ratio * (self.servo_center - self.servo_left)

        servo = max(self.servo_left, min(self.servo_right, servo))
        return servo

    def speed_to_erpm(self, speed):
        erpm = self.speed_direction * speed * self.speed_to_erpm_gain
        erpm = max(-self.max_erpm, min(self.max_erpm, erpm))
        return erpm

    def drive_callback(self, msg):
        steering = msg.drive.steering_angle
        speed = msg.drive.speed

        servo_cmd = self.steering_to_servo(steering)
        erpm_cmd = self.speed_to_erpm(speed)

        servo_msg = Float64()
        servo_msg.data = servo_cmd
        self.servo_pub.publish(servo_msg)

        motor_msg = Float64()
        motor_msg.data = erpm_cmd
        self.motor_pub.publish(motor_msg)

        self.get_logger().info(
            f'/drive: speed={speed:.2f}, steering={steering:.3f} '
            f'-> servo={servo_cmd:.3f}, erpm={erpm_cmd:.1f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriveToVescBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
