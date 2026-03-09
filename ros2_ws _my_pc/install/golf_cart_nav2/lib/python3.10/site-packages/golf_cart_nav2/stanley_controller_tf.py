#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

import math
import csv

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class StanleyController(Node):

    def __init__(self):
        super().__init__('stanley_controller_tf')

        # Parameters
        self.declare_parameter('path_file', '/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/recorded_path.csv')
        self.declare_parameter('k_gain', 0.8)
        self.declare_parameter('wheelbase', 2.3)
        self.declare_parameter('max_steering_angle', 0.5236)
        self.declare_parameter('constant_speed', 0.2)
        self.declare_parameter('global_frame', 'map')

        self.k = self.get_parameter('k_gain').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.speed = self.get_parameter('constant_speed').value
        self.global_frame = self.get_parameter('global_frame').value

        path_file = self.get_parameter('path_file').value
        self.path = self.load_path(path_file)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.steer_pub = self.create_publisher(Float32, '/cmd_steering_angle', 10)
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)
        self.path_pub = self.create_publisher(Path, '/viz_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)

        self.viz_path_msg = self.create_path_msg()

        # Timer instead of odom subscription
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Stanley Controller (TF based) Started')

    # -----------------------------------------------------
    def load_path(self, file_path):
        path = []
        try:
            with open(file_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)
                for row in reader:
                    path.append((float(row[0]), float(row[1])))
            self.get_logger().info(f'Loaded {len(path)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {e}')
        return path

    # -----------------------------------------------------
    def create_path_msg(self):
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        for pt in self.path:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        return path_msg

    # -----------------------------------------------------
    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                'd455_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            return x, y, yaw

        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    # -----------------------------------------------------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # -----------------------------------------------------
    def control_loop(self):

        if len(self.path) == 0:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose

        # ===== Find closest point =====
        min_dist = float("inf")
        closest_index = 0

        for i, pt in enumerate(self.path):
            dx = pt[0] - x
            dy = pt[1] - y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        target = self.path[closest_index]

        # ===== Heading error =====
        path_yaw = math.atan2(target[1] - y, target[0] - x)
        heading_error = self.normalize_angle(path_yaw - yaw)

        # ===== Cross-track error (signed) =====
        dx = target[0] - x
        dy = target[1] - y

        cross_track_error = math.sin(yaw) * dx - math.cos(yaw) * dy

        # ===== Stanley Control Law =====
        steering = heading_error + math.atan2(
            self.k * cross_track_error,
            self.speed
        )

        steering = self.normalize_angle(steering)

        steering = max(min(steering, self.max_steer), -self.max_steer)

        # ===== Publish Commands =====
        self.publish_cmd(steering)

        # ===== RViz Visualization =====
        self.viz_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.viz_path_msg)
        self.publish_target_marker(target)

    # -----------------------------------------------------
    def publish_target_marker(self, target):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = 0.1
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.g = 1.0  # Green
        self.marker_pub.publish(marker)

    # -----------------------------------------------------
    def publish_cmd(self, steering):

        steer_msg = Float32()
        steer_msg.data = steering
        self.steer_pub.publish(steer_msg)

        speed_msg = Float32()
        speed_msg.data = self.speed
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
