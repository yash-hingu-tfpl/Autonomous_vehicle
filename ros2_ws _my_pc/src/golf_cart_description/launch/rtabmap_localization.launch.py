# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():

#     parameters = [{
#         'frame_id': 'base_link',
#         'odom_frame_id': 'odom',
#         'map_frame_id': 'map',

#         'subscribe_depth': True,
#         'subscribe_imu': True,
#         'subscribe_odom_info': True,

#         'approx_sync': False,
#         'wait_imu_to_init': True,

#         # 2D vehicle
#         'Reg/Force3DoF': True,
#         'Optimizer/Slam2D': True,

#         # LOCALIZATION MODE
#         # 'Mem/IncrementalMemory': False,
#         # 'Mem/InitWMWithAllNodes': True,

#         'database_path': '/home/developer/ros2_ws/src/golf_cart_description/rtabmap.db',
#     }]

#     remappings = [
#         ('imu', '/imu/data'),
#         ('rgb/image', '/camera/d455/infra1/image_rect_raw'),
#         ('rgb/camera_info', '/camera/d455/infra1/camera_info'),
#         ('depth/image', '/camera/d455/depth/image_rect_raw'),
#     ]

#     return LaunchDescription([

#         # Visual Odometry
#         Node(
#             package='rtabmap_odom',
#             executable='rgbd_odometry',
#             output='screen',
#             parameters=parameters,
#             remappings=remappings,
#         ),

#         # RTAB-Map Localization
#         Node(
#             package='rtabmap_slam',
#             executable='rtabmap',
#             name='rtabmap',
#             output='screen',
#             parameters=parameters,
#             remappings=remappings,
#             arguments=[
#                 '--Mem/IncrementalMemory', 'false',
#                 '--Mem/InitWMWithAllNodes', 'true',
#                 '--Optimizer/Slam2D', 'true'
#             ],
#         ),
#                 Node(
#             package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
#             parameters=[{'use_mag': False, 
#                          'world_frame':'enu', 
#                          'publish_tf':False}],
#             remappings=[('imu/data_raw', '/camera/d455/imu')]),
    
#     ])



from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    

    common_params = {
        # 'frame_id': 'base_footprint',
        'frame_id': 'd455_link',
        # 'frame_id': 'base_link',


        'odom_frame_id': 'odom',
        # 'odom_frame_id': 'odom',

        'map_frame_id': 'map',

        'subscribe_depth': True,
        'subscribe_imu': True,
        'subscribe_odom_info': True,

        'approx_sync': False,
        'wait_imu_to_init': True,

        # 'visual_odometry':'true', ###########


        # 'database_path': '/home/developer/ros2_ws/src/golf_cart_description/rtabmap.db',
        'database_path': '~/.ros/rtabmap.db',
        # 'database_path': '/home/developer/ros2_ws/src/golf_cart_description/map_test/office_test/rtabmap.db',

    }

    remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/d455/infra1/image_rect_raw'),
        ('rgb/camera_info', '/camera/d455/infra1/camera_info'),
        ('depth/image', '/camera/d455/depth/image_rect_raw'),
    ]
    # remappings=[
    #       ('imu', '/imu/data'),
    #       ('rgb/image', '/camera/d455/color/image_raw'),
    #       ('rgb/camera_info', '/camera/d455/color/camera_info'),
    #       ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw')] 

    return LaunchDescription([

        # -------------------------
        # RGBD VISUAL ODOMETRY
        # -------------------------
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[common_params],
            remappings=remappings,
        ),

            Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 
                        'world_frame':'enu', 
                        'publish_tf':False}],
        remappings=[('imu/data_raw', '/camera/d455/imu')]),

Node(
    package='rtabmap_viz',
    executable='rtabmap_viz',
    output='screen',
    parameters=[common_params]
        ),
        # -------------------------
        # RTAB-MAP LOCALIZATION
        # -------------------------
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[common_params],
            remappings=remappings,
            arguments=[
                '--Reg/Force3DoF', 'true',
                '--Optimizer/Slam2D', 'true',
                '--Mem/IncrementalMemory', 'false',
                '--Mem/InitWMWithAllNodes', 'true'
            ],
        ),
    ])
