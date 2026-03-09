#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

import math
import csv


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameter('path_file', '/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/recorded_path.csv')
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('wheelbase', 2.3)
        self.declare_parameter('max_steering_angle', 0.35)
        self.declare_parameter('constant_speed', 0.4)

        self.lookahead = self.get_parameter('lookahead_distance').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.speed = self.get_parameter('constant_speed').value

        path_file = self.get_parameter('path_file').value
        self.path = self.load_path(path_file)
        self.target_index = 0

        # Publishers
        self.steer_pub = self.create_publisher(
            Float32, '/cmd_steering_angle', 10)
        self.speed_pub = self.create_publisher(
            Float32, '/cmd_speed', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            # '/rtabmap/odom',
            '/odom',

            self.odom_callback,
            10)

        self.get_logger().info('Pure Pursuit Controller Started')

    # ------------------------------
    # Load Path
    # ------------------------------
    def load_path(self, file_path):
        path = []
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # skip header
            for row in reader:
                x = float(row[0])
                y = float(row[1])
                path.append((x, y))

        self.get_logger().info(f'Loaded {len(path)} waypoints')
        return path

    # ------------------------------
    # Odometry Callback
    # ------------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw(msg)

        target = self.find_lookahead_point(x, y)
        steering = self.compute_steering(x, y, yaw, target)

        self.publish_cmd(steering)

    # ------------------------------
    # Get yaw from quaternion
    # ------------------------------
    def get_yaw(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------
    # Find lookahead waypoint
    # ------------------------------
    def find_lookahead_point(self, x, y):
        for i in range(self.target_index, len(self.path)):
            dx = self.path[i][0] - x
            dy = self.path[i][1] - y
            dist = math.hypot(dx, dy)

            if dist >= self.lookahead:
                self.target_index = i
                return self.path[i]

        return self.path[-1]

    # ------------------------------
    # Pure Pursuit Steering Law
    # ------------------------------
    def compute_steering(self, x, y, yaw, target):
        dx = target[0] - x
        dy = target[1] - y

        target_angle = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_angle - yaw)

        steering = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            self.lookahead)

        return max(min(steering, self.max_steer), -self.max_steer)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ------------------------------
    # Publish commands
    # ------------------------------
    def publish_cmd(self, steering):
        steer_msg = Float32()
        steer_msg.data = steering
        self.steer_pub.publish(steer_msg)

        speed_msg = Float32()
        speed_msg.data = self.speed
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
