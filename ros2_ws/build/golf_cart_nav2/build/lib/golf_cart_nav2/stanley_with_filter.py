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

        # --- Configuration Parameters ---
        self.declare_parameter('path_file', '/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/recorded_color_path.csv')
        self.declare_parameter('k_gain', 0.5)           # Control gain: lower = smoother
        self.declare_parameter('k_soft', 1.0)           # Softening: prevents steering snap at low speed
        self.declare_parameter('max_steering_angle', 0.5236) 
        self.declare_parameter('constant_speed', 0.2)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('camera_link', 'd455_link') 

        # Load values
        self.k = self.get_parameter('k_gain').value
        self.k_soft = self.get_parameter('k_soft').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.speed = self.get_parameter('constant_speed').value
        self.global_frame = self.get_parameter('global_frame').value
        self.camera_link = self.get_parameter('camera_link').value

        self.path = self.load_path(self.get_parameter('path_file').value)

        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- ROS Pubs ---
        self.steer_pub = self.create_publisher(Float32, '/cmd_steering_angle', 10)
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)
        self.path_pub = self.create_publisher(Path, '/viz_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)

        self.viz_path_msg = self.create_path_msg()
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Stanley Controller Running. Tracking from: {self.camera_link}')

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

    def create_path_msg(self):
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        for pt in self.path:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = pt[0], pt[1]
            path_msg.poses.append(pose)
        return path_msg

    def get_robot_pose(self):
        try:
            # We track the front axle center directly (d455_link)
            t = self.tf_buffer.lookup_transform(self.global_frame, self.camera_link, rclpy.time.Time())
            
            q = t.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            x = t.transform.translation.x
            y = t.transform.translation.y

            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        if not self.path: return

        pose = self.get_robot_pose()
        if pose is None: return
        x, y, yaw = pose

        # 1. Find the Closest Waypoint
        min_dist = float("inf")
        idx = 0
        for i, pt in enumerate(self.path):
            d = math.hypot(pt[0] - x, pt[1] - y)
            if d < min_dist:
                min_dist = d
                idx = i
        
        target = self.path[idx]

        # 2. Path Heading (Calculate tangent of the path)
        if idx < len(self.path) - 1:
            p1 = self.path[idx]
            p2 = self.path[idx + 1]
            path_yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        else:
            p1 = self.path[idx - 1]
            p2 = self.path[idx]
            path_yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

        # 3. Heading Error (Robot angle vs Path angle)
        heading_error = self.normalize_angle(path_yaw - yaw)

        # 4. Cross-track Error (The distance from the path center-line)
        dx = x - target[0]
        dy = y - target[1]
        
        # Signed cross-track error: project error vector onto the path normal
        cross_track_error = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        # 5. Stanley Law with Softening
        # This prevents the 'full turn' when speed is low or error is small
        crosstrack_term = math.atan2(self.k * cross_track_error, self.speed + self.k_soft)
        
        raw_steering = heading_error + crosstrack_term

        # Normalize and Limit
        steering = self.normalize_angle(raw_steering)
        steering = max(min(steering, self.max_steer), -self.max_steer)
        
        # Publish
        self.publish_cmd(steering)
        
        # Viz
        self.viz_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.viz_path_msg)
        self.publish_target_marker(target)

    def publish_cmd(self, steering):
        s_msg = Float32(); s_msg.data = float(steering)
        v_msg = Float32(); v_msg.data = float(self.speed)
        self.steer_pub.publish(s_msg)
        self.speed_pub.publish(v_msg)

    def publish_target_marker(self, target):
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = target[0], target[1], 0.1
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.a, m.color.g = 1.0, 1.0 
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()