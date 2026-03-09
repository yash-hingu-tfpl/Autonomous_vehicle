import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient

class FollowPathClient(Node):

    def __init__(self):
        super().__init__('follow_path_client')

        self._action_client = ActionClient(self, FollowPath, 'follow_path')

        self.subscription = self.create_subscription(
            Path,
            '/recorded_path',
            self.path_callback,
            10)

    def path_callback(self, msg):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowPath server not available')
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg

        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info('Path sent to Nav2')


def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()
    rclpy.spin(node)
    rclpy.shutdown()
