from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",   # opens keyboard in new terminal
        remappings=[
            ('/cmd_vel', '/ackermann_steering_controller/reference_unstamped')
        ]
    )

    return LaunchDescription([
        teleop
    ])
