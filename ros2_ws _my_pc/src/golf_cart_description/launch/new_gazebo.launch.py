from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg_share = FindPackageShare("golf_cart_description").find("golf_cart_description")

    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "golfcar_new.xacro"
    )
    print("this is the path of th xacro_file : ----->",xacro_file)
    controllers_file = os.path.join(
        pkg_share,
        "config",
        "ackermann2.yaml"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("gazebo_ros").find("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", xacro_file]) #while use realsense in gazebo add  ' use_gazebo:=true'
        }],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "golf_cart"
        ],
        output="screen",
    )

    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {"robot_description": Command(["xacro ", xacro_file])},
    #         controllers_file
    #     ],
    #     output="screen",
    # )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        # ros2_control_node
    ])
