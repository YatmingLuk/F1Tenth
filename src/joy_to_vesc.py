#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import pygame
import sys

class JoyToVesc(Node):
    def __init__(self):
        super().__init__('joy_to_vesc')
        pygame.init()
        pygame.joystick.init()
        
        count = pygame.joystick.get_count()
        self.get_logger().info(f"偵測到 {count} 個搖桿裝置")
        
        if count == 0:
            self.get_logger().error("❌ 完全找不到搖桿，請檢查 USB 連線！")
            return
            
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(f"✅ 已鎖定裝置: {self.joy.get_name()}")

        # 建立 Publisher
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        
        # 設定更新頻率 (20Hz)
        self.create_timer(0.05, self.timer_cb) 

        # --- 設定區域 ---
        self.max_rpm = 3000.0  # <--- 如果馬達太慢，可以調高到 5000 或更高
        self.steer_axis = 0    # 左類比左右
        self.speed_axis = 3    # 右類比上下 (如果沒反應，試試看 1 或 2)

    def timer_cb(self):
        pygame.event.pump()
        
        # 讀取所有軸數值以供診斷
        num_axes = self.joy.get_numaxes()
        axes_values = [round(self.joy.get_axis(i), 2) for i in range(num_axes)]
        
        # 取得搖桿原始數值 (-1.0 ~ 1.0)
        steer_raw = self.joy.get_axis(self.steer_axis)
        throttle_raw = self.joy.get_axis(self.speed_axis)
        
        # 1. 計算舵機位置 (範圍 0.0 ~ 1.0，0.5 為中立)
        servo_val = (steer_raw + 1.0) / 2.0
        
        # 2. 計算馬達速度 (放大到 ERPM 單位)
        # 注意：如果往前推變後退，就把 throttle_raw 前面的負號拿掉
        speed_val = -throttle_raw * self.max_rpm 
        
        # 發布指令
        self.servo_pub.publish(Float64(data=servo_val))
        self.speed_pub.publish(Float64(data=speed_val))

        # 診斷資訊：當有動作時印出數值
        if abs(throttle_raw) > 0.1 or abs(steer_raw) > 0.1:
            self.get_logger().info(
                f"控制中 -> 舵機: {servo_val:.2f} | 馬達 RPM: {int(speed_val)} | 原始 Axis 數值: {axes_values}"
            )

def main():
    rclpy.init()
    node = JoyToVesc()
    
    try:
        if pygame.joystick.get_count() > 0:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
