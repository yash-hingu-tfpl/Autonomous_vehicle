import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup paths and file names
    pkg_name = 'ackermann_vehicle' # <--- CHANGE THIS to your actual package name
    xacro_file = 'ackermann_robot.urdf.xacro'
    controller_config = 'ackermann_controller.yaml'

    pkg_path = get_package_share_directory(pkg_name)
    xacro_path = os.path.join(pkg_path, 'description', xacro_file)
    controller_path = os.path.join(pkg_path, 'config', controller_config)

    # 2. Process XACRO to URDF
    robot_description_raw = Command(['xacro ', xacro_path])

    # 3. Node: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 4. Node: ROS 2 Control Node (The Manager)
    # This replaces Gazebo's control plugin for "real" or "mock" hardware
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[{'robot_description': robot_description_raw}, controller_path],
    #     output='screen',
    # )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_raw}, controller_path],
        output='screen',
        # Adding the remappings here ensures the internal nodes inherit them
        remappings=[
            ('/ackermann_cont/reference_unstamped', '/cmd_vel'),
            ('/ackermann_cont/odometry', '/odom'),
            ('/ackermann_cont/tf_odometry', '/tf')
        ]
    )

    # 5. Spawner: Joint State Broadcaster
    # This publishes the /joint_states topic so RViz can move the parts
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )


    # 6. Spawner: Ackermann Steering Controller
    ackermann_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_cont'],
    )
    # ackermann_cont_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["ackermann_cont"],
    #     remappings=[
    #         ("ackermann_cont/reference_unstamped", "/cmd_vel"),
    #         ("ackermann_cont/odometry", "/odom"),
    #         ("ackermann_cont/tf_odometry", "/tf"),
    #     ],
    # )



    # 7. Node: RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_path] # Optional: add your rviz config path
    )

    # Launch everything
    return LaunchDescription([
        node_robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        ackermann_cont_spawner,
        node_rviz
    ])