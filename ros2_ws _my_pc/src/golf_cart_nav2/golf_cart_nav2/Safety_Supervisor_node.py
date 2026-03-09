#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rtabmap_msgs.msg import OdomInfo

class SafetySupervisor(Node):

    def __init__(self):
        super().__init__('safety_supervisor')

        # Parameters
        self.timeout_sec = 0.5

        # State
        self.last_msg_time = self.get_clock().now()
        self.tracking_lost = True  # Start safe

        # Subscriber
        self.odom_sub = self.create_subscription(
            OdomInfo,
            '/odom_info',
            self.odom_callback,
            10)

        # Publisher
        self.estop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10)

        # Timer to check watchdog
        self.timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info("Safety Supervisor Started")

    def odom_callback(self, msg):

        self.last_msg_time = self.get_clock().now()

        # LOST FIELD
        if msg.lost:
            self.tracking_lost = True
            self.get_logger().warn("Tracking LOST!")
        else:
            self.tracking_lost = False

        self.publish_estop()

    def watchdog_check(self):

        now = self.get_clock().now()
        duration = (now - self.last_msg_time).nanoseconds * 1e-9

        if duration > self.timeout_sec:
            self.tracking_lost = True
            self.get_logger().error("Odom timeout!")

        self.publish_estop()

    def publish_estop(self):

        msg = Bool()
        msg.data = self.tracking_lost
        self.estop_pub.publish(msg)


def main():
    rclpy.init()
    node = SafetySupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()