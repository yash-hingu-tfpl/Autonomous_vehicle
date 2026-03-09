# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Bool
# import numpy as np


# class ObstacleDetector(Node):

#     def __init__(self):
#         super().__init__('obstacle_detector')

#         self.stop_distance = 0.8      # soft stop (meters)
#         self.hard_stop_distance = 0.4 # relay cut

#         self.scan_sub = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.scan_callback,
#             10)

#         self.estop_pub = self.create_publisher(
#             Bool,
#             '/emergency_stop',
#             10)

#         self.get_logger().info("Obstacle detector started")

#     def scan_callback(self, msg):

#         ranges = np.array(msg.ranges)

#         # Remove invalid values
#         valid = ranges[np.isfinite(ranges)]
#         valid = valid[valid > 0.05]

#         if len(valid) == 0:
#             return

#         min_dist = np.min(valid)

#         estop_msg = Bool()

#         if min_dist < self.hard_stop_distance:
#             self.get_logger().error("HARD STOP! Obstacle very close!")
#             estop_msg.data = True

#         elif min_dist < self.stop_distance:
#             self.get_logger().warn("Obstacle detected. Stopping.")
#             estop_msg.data = True

#         else:
#             estop_msg.data = False

#         self.estop_pub.publish(estop_msg)


# def main():
#     rclpy.init()
#     node = ObstacleDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import math


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        # ---- Cart Parameters ----
        self.cart_width = 1.14  # meters
        self.stop_distance = 0.8
        self.hard_stop_distance = 0.4
        self.safety_distance = 2.0  # detection length

        # ---- Calculate Front Angle from Width ----
        half_width = self.cart_width / 2.0
        self.front_angle = math.atan(half_width / self.safety_distance)

        self.get_logger().info(
            f"Front detection angle: ±{math.degrees(self.front_angle):.2f} degrees"
        )

        # ---- ROS Interfaces ----
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.estop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10)

        self.get_logger().info("Obstacle detector started")

    def scan_callback(self, msg):

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Select only front cone
        mask = np.abs(angles) <= self.front_angle
        front_ranges = ranges[mask]

        # Remove invalid values
        valid = front_ranges[np.isfinite(front_ranges)]
        valid = valid[valid > 0.05]

        if len(valid) == 0:
            return

        min_dist = np.min(valid)

        estop_msg = Bool()

        # ---- Hard Stop ----
        if min_dist < self.hard_stop_distance:
            self.get_logger().error(
                f"HARD STOP! Obstacle at {min_dist:.2f} m"
            )
            estop_msg.data = True

        # ---- Soft Stop ----
        elif min_dist < self.stop_distance:
            self.get_logger().warn(
                f"Obstacle detected at {min_dist:.2f} m"
            )
            estop_msg.data = True

        else:
            estop_msg.data = False

        self.estop_pub.publish(estop_msg)


def main():
    rclpy.init()
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()