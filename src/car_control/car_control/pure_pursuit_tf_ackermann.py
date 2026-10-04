#!/usr/bin/env python3
import math
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from tf2_ros import Buffer, TransformListener, TransformException


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # ===== Parameters =====
        self.declare_parameter('waypoint_csv', os.path.expanduser(
            '~/newf1_ws/src/car_control/car_control/waypoints/waypoints.csv'
        ))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lookahead', 0.5)
        self.declare_parameter('speed', 0.7)          # real car first test: keep slow
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steering', 0.45)
        self.declare_parameter('control_period', 0.05)

        self.waypoint_csv = self.get_parameter('waypoint_csv').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.speed = float(self.get_parameter('speed').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.control_period = float(self.get_parameter('control_period').value)

        # ===== TF listener: read map -> base_link pose from Cartographer =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x = None
        self.y = None
        self.yaw = None

        self.waypoints = self.load_waypoints(self.waypoint_csv)

        # Publish to vesc_ackermann's input topic
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_cmd',
            10
        )

        self.target_marker_pub = self.create_publisher(
            Marker,
            '/pure_pursuit/target_marker',
            10
        )

        self.waypoints_marker_pub = self.create_publisher(
            Marker,
            '/pure_pursuit/waypoints_marker',
            10
        )

        self.waypoints_text_pub = self.create_publisher(
            MarkerArray,
            '/pure_pursuit/waypoint_labels',
            10
        )

        self.create_timer(self.control_period, self.control)
        self.create_timer(1.0, self.publish_waypoints_markers)

        self.get_logger().info(
            f'Pure Pursuit started. waypoint_csv={self.waypoint_csv}, '
            f'pose=tf {self.map_frame}->{self.base_frame}, output=/ackermann_cmd'
        )

    def update_pose_from_tf(self):
        """Update self.x, self.y, self.yaw from TF: map -> base_link."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Cannot get TF {self.map_frame}->{self.base_frame}: {ex}',
                throttle_duration_sec=1.0
            )
            return False

        self.x = trans.transform.translation.x
        self.y = trans.transform.translation.y

        q = trans.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        return True

    def load_waypoints(self, filepath):
        waypoints = []

        if not os.path.exists(filepath):
            self.get_logger().error(f'CSV not found: {filepath}')
            return waypoints

        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            next(reader, None)  # skip header if it exists

            for row in reader:
                if len(row) < 2:
                    continue

                try:
                    x = float(row[0])
                    y = float(row[1])
                    waypoints.append((x, y))
                except ValueError:
                    continue

        self.get_logger().info(f'Loaded {len(waypoints)} waypoints')
        return waypoints

    def find_target_point(self):
        if self.x is None or self.y is None or not self.waypoints:
            return None, None

        nearest_idx = 0
        min_dist = float('inf')

        for i, wp in enumerate(self.waypoints):
            d = math.hypot(wp[0] - self.x, wp[1] - self.y)
            if d < min_dist:
                min_dist = d
                nearest_idx = i

        n = len(self.waypoints)
        for offset in range(n):
            idx = (nearest_idx + offset) % n
            wp = self.waypoints[idx]
            d = math.hypot(wp[0] - self.x, wp[1] - self.y)
            if d >= self.lookahead:
                return wp, idx

        return self.waypoints[nearest_idx], nearest_idx

    def publish_target_marker(self, tx, ty):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'pure_pursuit_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = tx
        marker.pose.position.y = ty
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.target_marker_pub.publish(marker)

    def publish_waypoints_markers(self):
        if not self.waypoints:
            return

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'pure_pursuit_waypoints'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.3
        marker.color.b = 1.0

        for wx, wy in self.waypoints:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.05
            marker.points.append(p)

        self.waypoints_marker_pub.publish(marker)

        label_array = MarkerArray()
        for i, (wx, wy) in enumerate(self.waypoints):
            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame
            text_marker.header.stamp = self.get_clock().now().to_msg()

            text_marker.ns = 'pure_pursuit_waypoint_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = wx
            text_marker.pose.position.y = wy
            text_marker.pose.position.z = 0.25
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.15

            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0

            text_marker.text = str(i)
            label_array.markers.append(text_marker)

        self.waypoints_text_pub.publish(label_array)

    def control(self):
        if not self.update_pose_from_tf():
            return

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded.', throttle_duration_sec=2.0)
            return

        target, target_idx = self.find_target_point()
        if target is None:
            return

        tx, ty = target
        self.publish_target_marker(tx, ty)

        dx = tx - self.x
        dy = ty - self.y

        # Transform target point from map frame into vehicle local frame.
        # local_x: forward direction, local_y: left direction.
        local_x = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        local_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        if local_x <= 0.0:
            return

        alpha = math.atan2(local_y, local_x)

        steering = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            self.lookahead
        )
        steering = max(min(steering, self.max_steering), -self.max_steering)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.drive.speed = self.speed
        msg.drive.steering_angle = -steering

        self.ackermann_pub.publish(msg)

        self.get_logger().info(
            f'pos=({self.x:.2f}, {self.y:.2f}) '
            f'yaw={self.yaw:.2f} '
            f'target_idx={target_idx} '
            f'target=({tx:.2f}, {ty:.2f}) '
            f'steering={steering:.3f} '
            f'speed={self.speed:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
