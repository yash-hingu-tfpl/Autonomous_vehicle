# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     # 1. RealSense D455 Driver
#     # realsense_launch = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([
#     #         os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
#     #     ]),
#     #     launch_arguments={
#     #         'align_depth.enable': 'true',
#     #         'enable_gyro': 'true',
#     #         'enable_accel': 'true',
#     #         'unite_imu_method': '2', # Merges gyro/accel into /camera/imu
#     #         'pointcloud.enable': 'true',
#     #         'rgb_camera.profile': '640x480x30',
#     #         'depth_module.profile': '640x480x30'
#     #     }.items()
#     # )


# # 1. RealSense D455 Driver
#     realsense_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
#         ]),
#         launch_arguments={
#             'align_depth.enable': 'true',
#             'enable_gyro': 'true',
#             'enable_accel': 'true',
#             'unite_imu_method': '2',
#             'pointcloud.enable': 'true',
#             # These three lines fix the "No stream match" error:
#             'pointcloud.stream_filter': 'RS2_STREAM_COLOR',
#             'pointcloud.stream_index': '0',
#             'enable_color': 'true', 
#             # Ensure profiles match
#             'rgb_camera.profile': '640x480x30',
#             'depth_module.profile': '640x480x30'
#         }.items()
#     )
#     # 2. IMU Filter (Madgwick) - Essential for orientation
#     imu_filter = Node(
#         package='imu_filter_madgwick',
#         executable='imu_filter_madgwick_node',
#         name='imu_filter',
#         parameters=[{
#             'use_mag': False,
#             'world_frame': 'enu',
#             'publish_tf': False
#         }],
#         remappings=[
#             ('/imu/data_raw', '/camera/camera/imu'),
#             ('/imu/data', '/rtabmap/imu')
#         ]
#     )

#     # 3. RTAB-Map SLAM Node
#     # rtabmap_slam = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([
#     #         os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
#     #     ]),
#     #     launch_arguments={
#     #         'rtabmap_args': '--delete_db_on_start',
#     #         'rgb_topic': '/camera/color/image_raw',
#     #         'depth_topic': '/camera/aligned_depth_to_color/image_raw',
#     #         'camera_info_topic': '/camera/color/camera_info',
#     #         'frame_id': 'camera_link',
#     #         'approx_sync': 'true',
#     #         'wait_imu_to_init': 'true',
#     #         'imu_topic': '/rtabmap/imu',
#     #         'qos': '1'
#     #     }.items()
#     # )
#     # Update only the rtabmap_slam section
#     rtabmap_slam = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource([
#         os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
#     ]),
#     launch_arguments={

#         'subscribe_scan_cloud':'true',
#         'scan_cloud_topic':'/camera/camera/depth/color/points',
        
#         # 'rtabmap_args': '--delete_db_on_start',
#         'rtabmap_args': '--delete_db_on_start --RGBD/Enabled false --SubscribeScanCloud true',
#         'rgb_topic': '/camera/camera/color/image_raw',
        
#         'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
#         'camera_info_topic': '/camera/camera/color/camera_info',
#         'frame_id': 'camera_link',
#         'approx_sync': 'true',         # Ensure this is true
#         'wait_imu_to_init': 'true',
#         'imu_topic': '/rtabmap/imu',
#         'qos': '1',                    # 2 often handles Realsense 'Best Effort' better
#         'rviz': 'true'                 # Add this to see the debug window
#     }.items()
# )

#     return LaunchDescription([
#         realsense_launch,
#         imu_filter,
#         rtabmap_slam
#     ])


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. RealSense D455 Driver Fixes
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'pointcloud.enable': 'true',
            'pointcloud.stream_filter': 'RS2_STREAM_COLOR',
            'pointcloud.stream_index': '0',
            'enable_color': 'true',
            'rgb_camera.profile': '640x480x15', # Lowered to 15fps to reduce CPU load
            'depth_module.profile': '640x480x15'
        }.items()
    )

    # 2. IMU Filter
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
        remappings=[('/imu/data_raw', '/camera/camera/imu'), ('/imu/data', '/rtabmap/imu')]
    )

    # 3. RTAB-Map SLAM Node with Timing Fixes
    rtabmap_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --RGBD/NeighborLinkRefining true --RGBD/ProximityBySpace true',
             'subscribe_scan_cloud':'true',
            'scan_cloud_topic':'/camera/camera/depth/color/points',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'camera_link',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.01', # Fixes the high diff warning
            'wait_imu_to_init': 'true',
            'imu_topic': '/rtabmap/imu',
            'qos': '2',
            'wait_for_transform': '0.2' # Fixes the extrapolation error
        }.items()
    )

    return LaunchDescription([realsense_launch, imu_filter, rtabmap_slam])