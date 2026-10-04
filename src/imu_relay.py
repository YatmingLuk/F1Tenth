import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')
        # 訂閱 vesc 沒名字的數據
        self.subscription = self.create_subscription(
            Imu, '/sensors/imu/raw', self.listener_callback, 10)
        # 發布修正後的數據
        self.publisher_ = self.create_publisher(Imu, '/imu/data_fixed', 10)
        self.get_logger().info('✅ IMU 修正節點已啟動：/sensors/imu/raw -> /imu/data_fixed')

    def listener_callback(self, msg):
        msg.header.frame_id = 'imu' # 強制補上名字
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
