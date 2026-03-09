# from setuptools import find_packages, setup

# package_name = 'golf_cart_nav2'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name, 'launch')
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='developer',
#     maintainer_email='developer@todo.todo',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
from setuptools import setup
import os
from glob import glob

package_name = 'golf_cart_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Required
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ⬇⬇⬇ THIS IS THE KEY PART ⬇⬇⬇
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Optional (maps, params)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@todo.todo',
    description='Nav2 bringup for golf cart',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'pure_pursuit = golf_cart_nav2.pure_pursuit_node:main',
            'pure_pursuit_viz = golf_cart_nav2.pure_pursuit_node_viz:main',
            'nav2_to_ackermann = golf_cart_nav2.nav2_to_ackermann:main',
            'pure_pursuit_local = golf_cart_nav2.pure_pursuit_local:main',
            'path_publisher = golf_cart_nav2.path_publisher:main',
            'follow_path = golf_cart_nav2.follow_path:main',
            'path_follower = golf_cart_nav2.path_follower:main',
            'stanley_controller_tf = golf_cart_nav2.stanley_controller_tf:main',
            'pure_pursuit_tf = golf_cart_nav2.pure_pursuit_tf:main',
            'pure_pursuit_speed = golf_cart_nav2.pure_pursuit_speed:main',
            'Safety_Supervisor_node = golf_cart_nav2.Safety_Supervisor_node:main',
            'object_detection_node = golf_cart_nav2.object_detection_node:main',
            'esp_parameter_load = golf_cart_nav2.esp_parameter_load:main',

        ],
    },
)
