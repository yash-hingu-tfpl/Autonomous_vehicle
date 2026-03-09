# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, Command
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time')

#     declare_use_sim_time = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false'
#     )

#     # -------------------------
#     # Robot description
#     # -------------------------
#     xacro_file = os.path.join(
#         get_package_share_directory('ackermann_vehicle'),
#         "description",
#         "fake_cart.xacro"   # <-- use new full xacro
#     )

#     # robot_description = Command(['xacro ', xacro_file])

#     # robot_state_publisher = Node(
#     #     package='robot_state_publisher',
#     #     executable='robot_state_publisher',
#     #     parameters=[{
#     #         'robot_description': robot_description,
#     #         'use_sim_time': use_sim_time
#     #     }],
#     #     output='screen'
#     # )

#     # -------------------------
#     # RTAB-Map Visual Odometry
#     # -------------------------
#     rtabmap_odom = Node(
#         package='rtabmap_odom',
#         executable='rgbd_odometry',
#         name='rtabmap_odom',
#         output='screen',
#         parameters=[{
#             'frame_id': 'base_footprint',
#             'odom_frame_id': 'odom',
#             'publish_tf': True,
#             'approx_sync': True,
#             'subscribe_depth': True,
#             'subscribe_imu': True,
#             'use_sim_time': use_sim_time
#         }],
#         remappings=[
#             ('rgb/image', '/camera/d455/color/image_raw'),
#             ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw'),
#             ('rgb/camera_info', '/camera/d455/color/camera_info'),
#             ('imu', '/camera/d455/imu')
#         ]
#     )

#     # -------------------------
#     # RTAB-Map SLAM (mapping)
#     # -------------------------
#     rtabmap_slam = Node(
#         package='rtabmap_slam',
#         executable='rtabmap',
#         name='rtabmap',
#         output='screen',
#         parameters=[{
#             'frame_id': 'base_footprint',
#             'map_frame_id': 'map',
#             'odom_frame_id': 'odom',
#             'publish_tf': True,
#             'subscribe_depth': True,
#             'subscribe_imu': True,
#             'approx_sync': True,
#             'use_sim_time': use_sim_time,

#             # Mapping parameters
#             'Mem/IncrementalMemory': "true",
#             'RGBD/OptimizeMaxError': 2.0,
#         }],
#         remappings=[
#             ('rgb/image', '/camera/d455/color/image_raw'),
#             ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw'),
#             ('rgb/camera_info', '/camera/d455/color/camera_info'),
#             ('imu', '/camera/d455/imu')
#         ]
#     )

#     # -------------------------
#     # RViz (optional)
#     # -------------------------
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen'
#     )

#     return LaunchDescription([
#         declare_use_sim_time,
#         # robot_state_publisher,
#         rtabmap_odom,
#         rtabmap_slam,
#         rviz
#     ])


# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_infra.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    parameters=[{
          'frame_id':'d455_link',
        #   'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True,
          


        #   #-------------------------------------
          'Reg/Force3DoF': 'true',      # Forces 2D movement (no Z, Roll, or Pitch)
          'Optimizer/Slam2D': 'true',   # Ensures the map stays flat
        #   'RGBD/OptimizeMaxError': '2.0',
        #   'Vis/MinInliers': '12',       # Helps stabilize visual odom in outdoor areas
        #   # ------------------------------------
          
          }]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/d455/infra1/image_rect_raw'),
          ('rgb/camera_info', '/camera/d455/infra1/camera_info'),
        #   ('rgb/image', '/camera/d455/rgb/image_rect_raw'),
        #   ('rgb/camera_info', '/camera/d455/rgb/camera_info'),
          ('depth/image', '/camera/d455/depth/image_rect_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        #Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=2),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={
                    # 'camera_namespace': 'd455',
                    'camera_name': 'd455',
                    'enable_gyro': 'true',
                    'enable_accel': 'true',
                    'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                    'enable_infra1': 'true',
                    'enable_infra2': 'true',
                    'enable_depth':'true',
                    'pointcloud.enable': 'true',
                    'emitter_enabled':'2',
                    'align_depth.enable':'true',
                    'enable_sync': 'true'}.items(),
        ),
                                  
        #                           'enable_gyro': 'true',
        #                         #   'enable_color':'true',
        #                           'enable_accel': 'true',
        #                           'unite_imu_method': LaunchConfiguration('unite_imu_method'),
        #                           'enable_infra1': 'true',
        #                           'enable_infra2': 'true',
        #                           'enable_sync': 'true',
        #                           'enable_depth':'true',
        #                           'pointcloud.enable': 'true',
        #                           'emitter_enabled':'2',
        #                           'align_depth.enable':'true'
        #                           }.items(),
        # ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/d455/imu')]),
    ])
