#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

import math
import csv
import time

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class AdaptivePurePursuit(Node):

    def __init__(self):
        super().__init__('adaptive_pure_pursuit')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('path_file', '/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/recorded_path.csv')
        self.declare_parameter('wheelbase', 2.3)

        self.declare_parameter('min_lookahead', 2.0)
        self.declare_parameter('lookahead_gain', 1.5)

        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('max_lat_acc', 1.8)   # m/s² safe golf cart

        self.declare_parameter('max_accel', 1.0)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('max_steering_rate', 0.5)

        self.declare_parameter('global_frame', 'map')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.min_ld = self.get_parameter('min_lookahead').value
        self.ld_gain = self.get_parameter('lookahead_gain').value

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_lat_acc = self.get_parameter('max_lat_acc').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.max_steer_rate = self.get_parameter('max_steering_rate').value
        self.global_frame = self.get_parameter('global_frame').value

        path_file = self.get_parameter('path_file').value
        self.path = self.load_path(path_file)
        self.target_index = 0

        # ---- State ----
        self.prev_x = None
        self.prev_y = None
        self.prev_time = None
        self.current_speed = 0.0
        self.prev_speed_cmd = 0.0
        self.prev_steer = 0.0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.steer_pub = self.create_publisher(Float32, '/cmd_steering_angle', 10)
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("🔥 Adaptive Pure Pursuit Started")

    # ---------------------------------------------------
    def load_path(self, file_path):
        path = []
        if file_path == '':
            return path
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                path.append((float(row[0]), float(row[1])))
        return path

    # ---------------------------------------------------
    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                'camera_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y*q.y + q.z*q.z)
            )

            return x, y, yaw

        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    # ---------------------------------------------------
    def estimate_speed(self, x, y):
        now = time.time()

        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_time = now
            return 0.0

        dt = now - self.prev_time
        if dt <= 0.0:
            return self.current_speed

        dx = x - self.prev_x
        dy = y - self.prev_y
        speed = math.hypot(dx, dy) / dt

        alpha = 0.4
        self.current_speed = alpha * speed + (1 - alpha) * self.current_speed

        self.prev_x = x
        self.prev_y = y
        self.prev_time = now

        return self.current_speed

    # ---------------------------------------------------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ---------------------------------------------------
    def find_lookahead_point(self, x, y, lookahead):
        for i in range(self.target_index, len(self.path)):
            dx = self.path[i][0] - x
            dy = self.path[i][1] - y
            if math.hypot(dx, dy) >= lookahead:
                self.target_index = i
                return self.path[i]
        return self.path[-1]

    # ---------------------------------------------------
    def control_loop(self):

        if len(self.path) == 0:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose
        speed_est = self.estimate_speed(x, y)

        # 1️⃣ Dynamic Lookahead
        lookahead = self.min_ld + self.ld_gain * speed_est

        target = self.find_lookahead_point(x, y, lookahead)

        dx = target[0] - x
        dy = target[1] - y
        target_angle = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_angle - yaw)

        # 2️⃣ Curvature
        curvature = 2.0 * math.sin(alpha) / lookahead

        # 3️⃣ Steering
        steering = math.atan(curvature * self.wheelbase)
        steering = max(min(steering, self.max_steer), -self.max_steer)

        # Steering rate limit
        steer_diff = steering - self.prev_steer
        max_delta = self.max_steer_rate * 0.05
        steer_diff = max(min(steer_diff, max_delta), -max_delta)
        steering = self.prev_steer + steer_diff
        self.prev_steer = steering

        # 4️⃣ Curve-based Speed Limit
        if abs(curvature) > 0.001:
            max_curve_speed = math.sqrt(self.max_lat_acc / abs(curvature))
        else:
            max_curve_speed = self.max_speed

        speed_cmd = min(self.max_speed, max_curve_speed)
        speed_cmd = max(speed_cmd, self.min_speed)

        # Acceleration limit
        dv = speed_cmd - self.prev_speed_cmd
        max_dv = self.max_accel * 0.05
        dv = max(min(dv, max_dv), -max_dv)
        speed_cmd = self.prev_speed_cmd + dv
        self.prev_speed_cmd = speed_cmd

        self.publish_cmd(steering, speed_cmd)

        self.get_logger().info(
            f"SpeedEst:{speed_est:.2f}  Cmd:{speed_cmd:.2f}  Curv:{curvature:.3f}"
        )

    # ---------------------------------------------------
    def publish_cmd(self, steering, speed):
        s = Float32()
        s.data = steering
        self.steer_pub.publish(s)

        v = Float32()
        v.data = speed
        self.speed_pub.publish(v)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()