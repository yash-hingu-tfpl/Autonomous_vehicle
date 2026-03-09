from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    # db_path = os.path.expanduser('/home/developer/ros2_ws/src/golf_cart_description/map_test/office_test/rtabmap.db')

    common_params = {
    'camera_name': 'd455',
    'frame_id': 'd455_link',
    'odom_frame_id': 'odom',
    'map_frame_id': 'map',

    'subscribe_depth': True,
    'subscribe_imu': True,
    'subscribe_odom_info': True,

    'approx_sync': True,
    'wait_imu_to_init': True,

    'database_path': os.path.expanduser('~/.ros/rtabmap.db'),

    # 🔥 LOCALIZATION MODE (as STRINGS)
    'Mem/IncrementalMemory': 'false',
    'Mem/InitWMWithAllNodes': 'true',
    'Reg/Force3DoF': 'true',
    'Optimizer/Slam2D': 'true',
}
    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/d455/color/image_raw'),
          ('rgb/camera_info', '/camera/d455/color/camera_info'),
          ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw')] 

    return LaunchDescription([

        # RGBD ODOMETRY
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[common_params],
            remappings=remappings,
        ),

        # IMU Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[('imu/data_raw', '/camera/d455/imu')]
        ),

        # RTAB-Map LOCALIZATION
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[common_params],
            remappings=remappings,
        ),

        # Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[common_params],
        ),
    ])