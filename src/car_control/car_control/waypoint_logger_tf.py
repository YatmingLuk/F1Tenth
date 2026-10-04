#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WaypointLoggerTF(Node):
    def __init__(self):
        super().__init__('waypoint_logger_tf')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_csv', '~/newf1_ws/src/car_control/car_control/waypoints/waypoints.csv')
        self.declare_parameter('record_distance', 0.10)  # meters
        self.declare_parameter('record_period', 0.05)    # seconds
        self.declare_parameter('marker_topic', '/waypoint_logger/markers')

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.output_csv = os.path.expanduser(self.get_parameter('output_csv').value)
        self.record_distance = float(self.get_parameter('record_distance').value)
        self.record_period = float(self.get_parameter('record_period').value)
        self.marker_topic = self.get_parameter('marker_topic').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_x = None
        self.last_y = None
        self.count = 0
        self.points = []

        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)

        self.file = open(self.output_csv, 'w', newline='')
        self.writer = csv.writer(self.file)
        # 你的 pure_pursuit 目前只讀前兩欄 x,y；多存 yaw 也沒關係
        self.writer.writerow(['x', 'y', 'yaw'])

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.timer = self.create_timer(self.record_period, self.timer_callback)

        self.get_logger().info(f'Waypoint logger started.')
        self.get_logger().info(f'TF: {self.map_frame} -> {self.base_frame}')
        self.get_logger().info(f'Output CSV: {self.output_csv}')
        self.get_logger().info(f'Record every {self.record_distance:.2f} m. Press Ctrl+C to stop and close file.')

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF not available: {e}', throttle_duration_sec=1.0)
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        yaw = yaw_from_quaternion(trans.transform.rotation)

        if self.last_x is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.record_distance:
                return

        self.writer.writerow([f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])
        self.file.flush()

        self.last_x = x
        self.last_y = y
        self.count += 1
        self.points.append((x, y))

        self.publish_marker()

        self.get_logger().info(
            f'#{self.count} x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}',
            throttle_duration_sec=0.2
        )

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'recorded_waypoints'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for px, py in self.points:
            p = Point()
            p.x = px
            p.y = py
            p.z = 0.08
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def destroy_node(self):
        try:
            self.file.flush()
            self.file.close()
            self.get_logger().info(f'Saved {self.count} waypoints to {self.output_csv}')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoggerTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
