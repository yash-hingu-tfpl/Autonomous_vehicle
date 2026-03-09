# # Requirements:
# #   Download rosbag:
# #    * demo_mapping.db3: https://drive.google.com/file/d/1v9qJ2U7GlYhqBJr7OQHWbDSCfgiVaLWb/view?usp=drive_link
# #
# # Example:
# #
# #   SLAM:
# #     $ ros2 launch rtabmap_demos robot_mapping_demo.launch.py rviz:=true rtabmap_viz:=true
# #
# #   Rosbag:
# #     $ ros2 bag play demo_mapping.db3 --clock
# #

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition, UnlessCondition
# from launch_ros.actions import Node
# from launch_ros.actions import SetParameter
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():

#     localization = LaunchConfiguration('localization')

#     parameters={
#           'frame_id':'base_footprint',
#           'odom_frame_id':'odom',
#           'odom_tf_linear_variance':0.001,
#           'odom_tf_angular_variance':0.001,
#           'subscribe_rgbd':True,
#           'subscribe_scan':True,
#           'approx_sync':True,
#           'sync_queue_size': 10,
#           # RTAB-Map's internal parameters should be strings
#           'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
#           'RGBD/ProximityBySpace':     'true',    # Local loop closure detection (using estimated position) with locations in WM
#           'RGBD/ProximityByTime':      'false',   # Local loop closure detection with locations in STM
#           'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
#           'Reg/Strategy':              '1',       # 0=Visual, 1=ICP, 2=Visual+ICP
#           'Vis/MinInliers':            '12',      # 3D visual words minimum inliers to accept loop closure
#           'RGBD/OptimizeFromGraphEnd': 'false',   # Optimize graph from initial node so /map -> /odom transform will be generated
#           'RGBD/OptimizeMaxError':     '4',       # Reject any loop closure causing large errors (>3x link's covariance) in the map
#           'Reg/Force3DoF':             'true',    # 2D SLAM
#           'Grid/FromDepth':            'false',   # Create 2D occupancy grid from laser scan
#           'Mem/STMSize':               '30',      # increased to 30 to avoid adding too many loop closures on just seen locations
#           'RGBD/LocalRadius':          '5',       # limit length of proximity detections
#           'Icp/CorrespondenceRatio':   '0.2',     # minimum scan overlap to accept loop closure
#           'Icp/PM':                    'false',
#           'Icp/PointToPlane':          'false',
#           'Icp/MaxCorrespondenceDistance': '0.15',
#           'Icp/VoxelSize':             '0.05'
#     }
    
#     remappings=[
#          ('rgb/image',       '/data_throttled_image'),
#          ('depth/image',     '/data_throttled_image_depth'),
#          ('rgb/camera_info', '/data_throttled_camera_info'),
#          ('scan',            '/jn0/base_scan')]
    
#     config_rviz = os.path.join(
#         get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
#     )

#     return LaunchDescription([

#         # Launch arguments
#         DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
#         DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
#         DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
#         DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

#         SetParameter(name='use_sim_time', value=True),

#         # Nodes to launch
#         Node(
#             package='rtabmap_sync', executable='rgbd_sync', output='screen',
#             parameters=[parameters,
#               {'rgb_image_transport':'compressed',
#                'depth_image_transport':'compressedDepth',
#                'approx_sync_max_interval': 0.02}],
#             remappings=remappings),
        
#         # SLAM mode:
#         Node(
#             condition=UnlessCondition(localization),
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[parameters],
#             remappings=remappings,
#             arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
#         # Localization mode:
#         Node(
#             condition=IfCondition(localization),
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[parameters,
#               {'Mem/IncrementalMemory':'False',
#                'Mem/InitWMWithAllNodes':'True'}],
#             remappings=remappings),

#         # Visualization:
#         Node(
#             package='rtabmap_viz', executable='rtabmap_viz', output='screen',
#             condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
#             parameters=[parameters],
#             remappings=remappings),
#         Node(
#             package='rviz2', executable='rviz2', name="rviz2", output='screen',
#             condition=IfCondition(LaunchConfiguration("rviz")),
#             arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
#     ])



from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    # Common parameters for D455
    parameters={
          'frame_id':'camera_link', # Change to 'base_link' or 'base_footprint' if using a robot base
          'odom_frame_id':'odom',
          'subscribe_rgbd':True,
          'subscribe_scan':False,
          'approx_sync':True,
          'use_action_for_goal':True,
          'RGBD/NeighborLinkRefining':'true',
          'RGBD/ProximityBySpace':'true',
          'Reg/Strategy':'0',      # 0=Visual, 1=ICP, 2=Visual+ICP
          'Reg/Force3DoF':'false', # Set to true if moving on a flat floor only
          'Grid/FromDepth':'true', # Create occupancy grid from camera depth
          'Mem/STMSize':'30',
          'Icp/VoxelSize':'0.05',
    }
    
    # D455 Specific Remappings
    remappings=[
         ('rgb/image',       '/camera/camera/color/image_raw'),
         ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
         ('rgb/camera_info', '/camera/camera/color/camera_info'),
         ('rgbd_image',      '/rgbd_image'),
         ('imu',             '/imu/data')]

    return LaunchDescription([

        DeclareLaunchArgument('rtabmap_viz',  default_value='true'),
        DeclareLaunchArgument('rviz',         default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),

        SetParameter(name='use_sim_time', value=False),

        # 1. Sync Node: Combines RGB + Depth into a single RGBD message
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'use_mag':False}],
            remappings=remappings),

        # 2. Odometry Node: Generates 'odom' transform from visual data
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings),
        
        # 3. SLAM Node: Main Mapping algorithm
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            # arguments=['-d']
            ), # Deletes old database to start fresh
            
        # 4. Localization Node (Optional)
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters, {'Mem/IncrementalMemory':'False'}],
            remappings=remappings),

        # 5. Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings),

        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                        'world_frame':'enu', 
                        'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/camera/imu')]),
    ])