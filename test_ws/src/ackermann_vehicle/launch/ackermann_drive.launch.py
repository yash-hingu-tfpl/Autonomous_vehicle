import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Controller Spawner for Ackermann with CORRECT Remapping syntax
    # We use standard ROS args to rename the topics so teleop works
    ackermann_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_cont",
            "--ros-args",
            "-r", "/ackermann_cont/reference_unstamped:=/cmd_vel",
            "-r", "/ackermann_cont/odometry:=/odom",
            "-r", "/ackermann_cont/tf_odometry:=/tf"
        ],
    )

    # 2. Joint State Broadcaster Spawner
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    return LaunchDescription([
        joint_broad_spawner,
        ackermann_cont_spawner
    ])