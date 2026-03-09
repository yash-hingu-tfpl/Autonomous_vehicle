#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

import math
import csv

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameter('path_file', '/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/recorded_color_path.csv')
        self.declare_parameter('lookahead_distance', 4.0)
        self.declare_parameter('wheelbase', 2.3)
        self.declare_parameter('max_steering_angle', 0.5236)
        self.declare_parameter('constant_speed', 0.2)
        self.declare_parameter('global_frame', 'map') # Frame for RViz

        self.lookahead = self.get_parameter('lookahead_distance').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.speed = self.get_parameter('constant_speed').value
        self.global_frame = self.get_parameter('global_frame').value

        path_file = self.get_parameter('path_file').value
        self.path = self.load_path(path_file)
        self.target_index = 0

        # Publishers
        self.steer_pub = self.create_publisher(Float32, '/cmd_steering_angle', 10)
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)
        
        # RViz Visualization Publishers
        self.path_pub = self.create_publisher(Path, '/viz_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/lookahead_marker', 10)

        # Pre-process the Path message for efficiency
        self.viz_path_msg = self.create_path_msg()

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.get_logger().info('Pure Pursuit Controller with RViz Viz Started')

    def load_path(self, file_path):
        path = []
        try:
            with open(file_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)  # skip header
                for row in reader:
                    path.append((float(row[0]), float(row[1])))
            self.get_logger().info(f'Loaded {len(path)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {e}')
        return path

    def create_path_msg(self):
        """Converts CSV list to a nav_msgs/Path for RViz."""
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        for pt in self.path:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        return path_msg

    def publish_lookahead_marker(self, target):
        """Creates a small sphere in RViz at the current lookahead target."""
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = 0.1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0 # Red color
        self.marker_pub.publish(marker)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw(msg)

        target = self.find_lookahead_point(x, y)
        steering = self.compute_steering(x, y, yaw, target)

        # Publish Control Commands
        self.publish_cmd(steering)

        # Publish Visualization
        self.viz_path_msg.header.stamp = msg.header.stamp
        self.path_pub.publish(self.viz_path_msg)
        self.publish_lookahead_marker(target)

    def get_yaw(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_lookahead_point(self, x, y):
        for i in range(self.target_index, len(self.path)):
            dx = self.path[i][0] - x
            dy = self.path[i][1] - y
            dist = math.hypot(dx, dy)
            if dist >= self.lookahead:
                self.target_index = i
                return self.path[i]
        return self.path[-1]

    def compute_steering(self, x, y, yaw, target):
        dx = target[0] - x
        dy = target[1] - y
        target_angle = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_angle - yaw)
        
        # The Pure Pursuit law
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead)
        return max(min(steering, self.max_steer), -self.max_steer)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def publish_cmd(self, steering):
        steer_msg = Float32()
        steer_msg.data = steering
        #steer_msg.data = -steering

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
