from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='lanelet2_rviz2',
            executable='lanelet2_rviz2',
            output='screen',
            parameters=[
                {"frame_id": "map"},
                {"osm_file_path": "/home/developer/Downloads/lanelet2_maps_color.osm"},
                {"speed_color_max": 90.0},
                {"refresh_freq": 2},
            ],
        ),
    ])
