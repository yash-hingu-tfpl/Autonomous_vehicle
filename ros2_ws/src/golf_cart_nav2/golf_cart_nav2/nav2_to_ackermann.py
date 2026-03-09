#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class Nav2ToAckermann(Node):
    def __init__(self):
        super().__init__('nav2_to_ackermann_bridge')
        
        # Parameters
        self.declare_parameter('wheelbase', 2.3)
        self.wheelbase = self.get_parameter('wheelbase').value

        # Subscribers
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers (Matching your hardware interface)
        self.steer_pub = self.create_publisher(Float32, '/cmd_steering_angle', 10)
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        # Ackermann Steering Formula: delta = atan(L * omega / v)
        # We handle the zero-velocity case to avoid division by zero
        if abs(v) > 0.01:
            steering_angle = math.atan2(self.wheelbase * omega, v)
        else:
            steering_angle = 0.0

        # Publish to your hardware
        steer_msg = Float32()
        steer_msg.data = steering_angle
        self.steer_pub.publish(steer_msg)

        speed_msg = Float32()
        speed_msg.data = v
        self.speed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()