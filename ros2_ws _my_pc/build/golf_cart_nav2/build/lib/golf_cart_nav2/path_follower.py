import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import os

class TxtPathFollower(Node):
    def __init__(self):
        super().__init__('txt_path_follower')
        self.navigator = BasicNavigator()

        # Update this to your actual file path
        file_path = '/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/poses.txt'
        
        # 1. Load Path from TXT
        self.path_msg = self.load_path_from_txt(file_path)

        # 2. Wait for Nav2
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        # 3. Start Following
        if len(self.path_msg.poses) > 0:
            self.get_logger().info(f'Following path with {len(self.path_msg.poses)} waypoints...')
            self.navigator.followPath(self.path_msg)
        else:
            self.get_logger().error('Path is empty! Check your file path.')
            return

        # 4. Monitor
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # Logging every ~2 meters to avoid spamming the console
                print(f'Distance to goal: {feedback.distance_to_goal:.2f} m')

        # 5. Result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Destination reached!')
        else:
            self.get_logger().error('Navigation failed or was canceled.')

    def load_path_from_txt(self, file_path):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        if not os.path.exists(file_path):
            self.get_logger().error(f'File not found: {file_path}')
            return path

        with open(file_path, 'r') as f:
            for line in f:
                # Skip the header line starting with #
                if line.startswith('#') or not line.strip():
                    continue
                
                parts = line.split()
                if len(parts) < 8:
                    continue
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                
                # Coordinates
                pose.pose.position.x = float(parts[1])
                pose.pose.position.y = float(parts[2])
                pose.pose.position.z = float(parts[3])
                
                # Quaternions (Heading)
                pose.pose.orientation.x = float(parts[4])
                pose.pose.orientation.y = float(parts[5])
                pose.pose.orientation.z = float(parts[6])
                pose.pose.orientation.w = float(parts[7])
                
                path.poses.append(pose)
        
        return path

def main():
    rclpy.init()
    node = TxtPathFollower()
    # No rclpy.spin() needed because Navigator handles its own loop
    rclpy.shutdown()

if __name__ == '__main__':
    main()