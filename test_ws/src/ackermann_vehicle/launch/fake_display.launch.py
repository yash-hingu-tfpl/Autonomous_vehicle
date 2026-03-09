from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("ackermann_vehicle")

    xacro_file = os.path.join(
        pkg_share,
        "description",
        "fake_cart.xacro"   # <-- use new full xacro
    )

    controllers_file = os.path.join(
        pkg_share,
        "config",
        "fake_controller.yaml"
    )

    robot_description = Command(["xacro ", xacro_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_file
        ],
        output="screen",
        remappings=[
            ('/ackermann_steering_controller/reference_unstamped', '/cmd_vel'),
            # ('/ackermann_steering_controller/odometry', '/odom'),
            # ('/ackermann_steering_controller/tf_odometry', '/tf')
        ]
    )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    # )
    # ackermann_steering_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['ackermann_steering_controller'],
    # )
    
    # depth_to_laser_file = os.path.join(
    #     get_package_share_directory("depthimage_to_laserscan"),
    #     'launch',
    #     'depthimage_to_laserscan-launch.py'
    # )

    # depth_to_laser = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([depth_to_laser_file])
    # )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        # joint_state_broadcaster_spawner,
        rviz,
        # ackermann_steering_controller,
        # depth_to_laser
    ])


# Daily Update:-
#     taken front wheels steer angle data at various steering angle 
#     got relation with front wheels to steering 
#     changed front wheel joints for robot model in ros
    
