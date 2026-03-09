from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    common_params = {
        # Frames
        'frame_id': 'd455_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',

        # Subscriptions
        'subscribe_depth': True,
        'subscribe_imu': True,
        'subscribe_odom_info': False,

        # Sync
        'approx_sync': True,
        'wait_imu_to_init': True,

        # 2D vehicle
        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',

        # MAPPING MODE
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',

        # Database
        'database_path': '/home/developer/ros2_ws/src/golf_cart_description/rtabmap_new.db',
    }

    remappings = [
        ('rgb/image', '/camera/d455/infra1/image_rect_raw'),
        ('rgb/camera_info', '/camera/d455/infra1/camera_info'),
        ('depth/image', '/camera/d455/depth/image_rect_raw'),
        ('imu', '/imu/data'),
    ]

    return LaunchDescription([

        # -------------------------
        # IMU FILTER
        # -------------------------
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu'
            }],
            remappings=[
                ('imu/data_raw', '/camera/d455/imu'),
                ('imu/data', '/imu/data'),
            ],
        ),

        # -------------------------
        # RGB-D ODOMETRY (IMU FUSED)
        # -------------------------
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[common_params],
            remappings=remappings,
        ),

        # -------------------------
        # RTAB-MAP SLAM (MAPPING)
        # -------------------------
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[common_params],
            remappings=remappings,
            arguments=['--delete_db_on_start'],
        ),

        # -------------------------
        # RTABMAP VISUALIZATION
        # -------------------------
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[common_params],
            remappings=remappings,
        ),
    ])
