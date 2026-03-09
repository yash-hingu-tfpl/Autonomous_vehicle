import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        self.publisher = self.create_publisher(Path, '/recorded_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        self.load_path()

    def load_path(self):
        file_path = '/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/poses.txt'   # 🔥 CHANGE THIS

        with open(file_path, 'r') as f:
            lines = f.readlines()

        for line in lines:
            data = line.strip().split()

            if len(data) != 8:
                continue

            _, x, y, z, qx, qy, qz, qw = map(float, data)

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # ignore z

            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.path_msg.poses.append(pose)

        self.get_logger().info(f'Loaded {len(self.path_msg.poses)} poses.')

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
