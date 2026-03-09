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


class PurePursuitTF(Node):

    def __init__(self):
        super().__init__('pure_pursuit_tf_controller')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('path_file', '/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/recorded_color_path_25feb.csv')
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('wheelbase', 2.3)
        self.declare_parameter('max_steering_angle', 0.36)
        
        self.declare_parameter('max_speed', 1.2)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('global_frame', 'map')  # KEEP map for RTAB

        self.lookahead = self.get_parameter('lookahead_distance').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.get_logger().info(f"max_steering_angle : {self.max_steer}")
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.global_frame = self.get_parameter('global_frame').value

        path_file = self.get_parameter('path_file').value
        self.path = self.load_path(path_file)
        self.target_index = 0

        # TF (same as old code)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.steer_pub = self.create_publisher(Float32, '/cmd_steering_angle', 10)
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)

        self.path_pub = self.create_publisher(Path, '/viz_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/lookahead_marker', 10)

        self.viz_path_msg = self.create_path_msg()

        # Speed memory
        self.prev_x = None
        self.prev_y = None
        self.prev_time = None
        self.current_speed = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Pure Pursuit Controller (RTAB Safe) Started")
        self.get_logger().info("Hello")

    # ---------------------------------------------------
    def load_path(self, file_path):
        path = []
        try:
            with open(file_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader, None)

                for row in reader:
                    path.append((float(row[0]), float(row[1])))

            self.get_logger().info(f"Loaded {len(path)} waypoints")

        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")

        return path

    # ---------------------------------------------------
    def create_path_msg(self):
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame

        for pt in self.path:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            path_msg.poses.append(pose)

        return path_msg

    # ---------------------------------------------------
    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                'd455_link',
                rclpy.time.Time()   # SAME AS OLD CODE
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation

            # SAME yaw extraction as your old stable code
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            # -------- SAFE SPEED CALCULATION ----------
            now = self.get_clock().now().nanoseconds * 1e-9

            if self.prev_x is not None:
                dt = now - self.prev_time
                if dt > 0 and dt < 1.0:   # prevent RTAB jump spikes
                    dist = math.hypot(x - self.prev_x, y - self.prev_y)
                    self.current_speed = dist / dt

            self.prev_x = x
            self.prev_y = y
            self.prev_time = now

            return x, y, yaw

        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    # ---------------------------------------------------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ---------------------------------------------------
    def find_lookahead_point(self, x, y):
        for i in range(self.target_index, len(self.path)):
            dx = self.path[i][0] - x
            dy = self.path[i][1] - y
            dist = math.hypot(dx, dy)

            if dist >= self.lookahead:
                self.target_index = i
                return self.path[i]

        return self.path[-1]

    # ---------------------------------------------------
    def compute_control(self, x, y, yaw, target):

        dx = target[0] - x
        dy = target[1] - y

        target_angle = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_angle - yaw)

        curvature = (2.0 * math.sin(alpha)) / self.lookahead

        steering = math.atan(self.wheelbase * curvature)
        steering = max(min(steering, self.max_steer), -self.max_steer)

        # Dynamic speed based on curvature
        speed = self.max_speed / (1 + 5 * abs(curvature))
        speed = max(min(speed, self.max_speed), self.min_speed)

        return steering, speed

    # ---------------------------------------------------
    def control_loop(self):

        if len(self.path) == 0:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose

        target = self.find_lookahead_point(x, y)
        steering, speed = self.compute_control(x, y, yaw, target)

        self.publish_cmd(steering, speed)

        # Visualization
        self.viz_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.viz_path_msg)
        self.publish_lookahead_marker(target)

    # ---------------------------------------------------
    def publish_lookahead_marker(self, target):
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
        marker.color.r = 1.0

        self.marker_pub.publish(marker)

    # ---------------------------------------------------
    def publish_cmd(self, steering, speed):

        steer_msg = Float32()
        steer_msg.data = steering
        self.steer_pub.publish(steer_msg)

        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
