import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
# 修正後的匯入方式
from pyvesc.VESC import VESC
from pyvesc.VESC.messages import GetImuData

class VescFullBridge(Node):
    def __init__(self):
        super().__init__('vesc_full_bridge')
        self.port = '/dev/ttyACM1' 
        try:
            # 使用 pyvesc 的 VESC 介面
            self.v = VESC(serial_port=self.port)
            self.get_logger().info(f"✅ 已連接 VESC 於 {self.port} (使用 pyvesc)")
        except Exception as e:
            self.get_logger().error(f"❌ 無法連接 VESC: {e}")
            return

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_timer(0.02, self.update_imu) # 50Hz

    def update_imu(self):
        try:
            # 使用 pyvesc 發送請求並獲取數據
            response = self.v.get_imu_data()
            
            if response:
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "imu"

                # pyvesc 會直接解析好數值
                msg.linear_acceleration.x = response.accel_x * 9.81
                msg.linear_acceleration.y = response.accel_y * 9.81
                msg.linear_acceleration.z = response.accel_z * 9.81

                msg.angular_velocity.x = response.gyro_x * (3.14159 / 180.0)
                msg.angular_velocity.y = response.gyro_y * (3.14159 / 180.0)
                msg.angular_velocity.z = response.gyro_z * (3.14159 / 180.0)

                msg.orientation.w = 1.0
                self.imu_pub.publish(msg)
            else:
                # 如果一直沒回應，可能是 VESC Tool 裡沒開 IMU
                pass
        except Exception as e:
            self.get_logger().warn(f"數據讀取失敗: {e}")

def main():
    rclpy.init()
    node = VescFullBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
