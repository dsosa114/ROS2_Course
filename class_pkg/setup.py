from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'class_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='dasc14@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_talker = class_pkg.custom_talker:main',
            'turtlesim_controller = class_pkg.turtlesim_controller:main',
            'undock_tb4 = class_pkg.turtlebot4_undock:main',
            'dock_tb4 = class_pkg.turtlebot4_dock:main',
            'patrol_tutorial = class_pkg.turtlebot4_nav_tutorial:main',
            'point_listener = class_pkg.turtlebot4_point_selector:main',
            'navegacion = class_pkg.NavigationDaniel:main',
            'navegacion2 = class_pkg.Patrol:main',
            'cam_publisher = class_pkg.camera_publisher:main',
            'img_processor = class_pkg.image_processor:main',
            'img_converter = class_pkg.image_converter:main',
            'compressed_image_viewer = class_pkg.compressed_image_viewer:main',
            'nav2_test = class_pkg.nav2_test:main',
            'odom_path_recorder = class_pkg.path_recorder:main'
        ],
    },
)
