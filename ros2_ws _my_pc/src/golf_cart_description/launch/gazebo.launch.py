from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess


def generate_launch_description():

    pkg_share = FindPackageShare("golf_cart_description")

    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", "golf_cart_ackermann.xacro"]
    )

    controller_yaml = PathJoinSubstitution(
        [pkg_share, "config", "ackermann_controller.yaml"]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Gazebo GUI"
        ),

        # Gazebo
ExecuteProcess(
    cmd=[
        "gazebo",
        "--verbose",
        "-s", "libgazebo_ros_factory.so"
    ],
    output="screen"
),

        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", xacro_file])
            }],
            output="screen",
        ),

        # spawn robot
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", "robot_description",
                "-entity", "golf_cart"
            ],
            output="screen",
        ),

        # joint state broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
                "--param-file", controller_yaml,
            ],
            output="screen",
        ),

        # ackermann controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "ackermann_steering_controller",
                "--controller-manager", "/controller_manager",
                "--param-file", controller_yaml,
            ],
            output="screen",
        ),
    ])
