from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'output_frame': 'base_link',
                'scan_time': 0.1,
                'range_min': 0.5,
                'range_max': 10.0,
                'use_sensor_qos': True,
                'camera_info_reliability': 'best_effort'
            }],
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan')
            ]
        )
    ])

