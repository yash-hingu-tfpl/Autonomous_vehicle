import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Path Setup ---
    pkg_share = get_package_share_directory('golf_cart_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Path to your tuned params file
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    # Default map (can be empty if just using RTAB-Map for SLAM)
    # map_yaml_file = os.path.join(pkg_share, 'maps', '')

    # --- 2. Camera Driver (Realsense D455) ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
        )]),
        launch_arguments={
            'camera_name':'d455',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_color':'true',
            'enable_depth':'true', 
            'enable_sync':'true',   
            'align_depth.enable':'true',   
            'pointcloud.enable':'true',   
            'unite_imu_method':'2', 
            'enable_infra1':'true', 
            'enable_infra2':'true',

        }.items()
    )
    # imu_kalman = Node(
    #     package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
    #     parameters=[{'use_mag': False, 
    #                     'world_frame':'enu', 
    #                     'publish_tf':False}],

    #     remappings=[('imu/data_raw', '/camera/camera/imu')])
    # # --- 3. RTAB-Map (Visual Odometry & Mapping) ---
    # rtabmap_node = Node(
    #     package='rtabmap_odom',
    #     executable='rgbd_odometry',
    #     name='rgbd_odometry',
    #     output='screen',
    #     parameters=[{
    #         'frame_id': 'base_link',
    #         'odom_frame_id': 'odom',
    #         'publish_tf': True,
    #         'wait_imu_to_init': True,
    #         'approx_sync': False,
    #     }],
    #     remappings=[
    #         # ('rgb/image', '/camera/color/image_raw'),
    #         # ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
    #         # ('rgb/camera_info', '/camera/color/camera_info'),
    #         # ('imu', '/camera/imu')

    #     ('imu', '/imu/data'),
    #     ('rgb/image', '/camera/camera/infra1/image_rect_raw'),
    #     ('rgb/camera_info', '/camera/camera/infra1/camera_info'),
    #     ('depth/image', '/camera/camera/depth/image_rect_raw'),
    #     ]
    # )

    # --- 4. Nav2 Stack Bringup ---
# --- 4. Nav2 Stack Bringup (Modified for your setup) ---
    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    )),
    launch_arguments={
        'use_sim_time': 'false',
        'params_file': params_file,
        'autostart': 'true'
    }.items()
)

    # --- 5. Custom Bridge Node (Nav2 Twist -> Ackermann Float32) ---
    bridge_node = Node(
        package='golf_cart_nav2',
        executable='nav2_to_ackermann',
        name='nav2_to_ackermann_bridge',
        output='screen',
        # parameters=[{'wheelbase': 2.3}]
    )

    # --- 6. Static TF (Base Link to Camera) ---
    # Camera mounted 0.5m forward, 1.2m up
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # arguments=['2.65', '0', '0.31', '0', '0', '0', 'base_link', 'd455_link']
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'd455_link']

    )

    # --- 7. CSV Path Visualization Node ---
    csv_viz_node = Node(
        package='golf_cart_nav2',
        executable='path_publisher',
        name='path_publisher'
    )

    return LaunchDescription([
        realsense_launch,
        # imu_kalman,
        # rtabmap_node,
        nav2_launch,
        bridge_node,
        static_tf,
        csv_viz_node
    ])