# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():

#     pkg_name = 'golf_cart_description'
#     pkg_path = get_package_share_directory(pkg_name)

#     xacro_file = os.path.join(pkg_path, 'urdf', 'golf_cart3.urdf.xacro')

#     robot_description = ParameterValue(
#         Command(['xacro', xacro_file]),
#         value_type=str
#     )

#     # ---------------- Gazebo (gz sim, CORRECT) ----------------
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory('ros_gz_sim'),
#                 'launch',
#                 'gz_sim.launch.py'
#             )
#         ),
#         launch_arguments={
#             'gz_args': '-r empty.sdf'
#         }.items()
#     )

#     # ---------------- Robot State Publisher ----------------
#     rsp = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[
#             {'robot_description': robot_description},
#             {'use_sim_time': True}
#         ],
#         output='screen'
#     )

#     # ---------------- Spawn Robot ----------------
#     spawn = Node(
#         package='ros_gz_sim',
#         executable='create',
#         arguments=[
#             '-name', 'golf_cart',
#             '-topic', 'robot_description',
#             '-z', '0.2'
#         ],
#         output='screen'
#     )

#     # ---------------- Controllers (DELAYED!) ----------------
#     joint_state_spawner = TimerAction(
#         period=5.0,   # wait for gz_ros2_control to start
#         actions=[
#             Node(
#                 package='controller_manager',
#                 executable='spawner',
#                 arguments=['joint_state_broadcaster'],
#                 output='screen'
#             )
#         ]
#     )

#     ackermann_spawner = TimerAction(
#         period=6.0,
#         actions=[
#             Node(
#                 package='controller_manager',
#                 executable='spawner',
#                 arguments=['ackermann_steering_controller'],
#                 output='screen'
#             )
#         ]
#     )

#     return LaunchDescription([
#         gazebo,
#         rsp,
#         spawn,
#         joint_state_spawner,
#         ackermann_spawner,
#     ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_name = 'golf_cart_description'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'golf_cart3.urdf.xacro'
    )

    robot_description = Command([
        'xacro ',
        xacro_file
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'golf_cart',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ackermann_steering_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        ackermann_controller
    ])
