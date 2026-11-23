from glob import glob
from setuptools import setup

package_name = 'slam_drone_m3'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kishan Ballal',
    maintainer_email='kishanrajballal22@gmail.com',
    description='M3: height-coloured 3D map and LiDAR victim detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'height_map_node = slam_drone_m3.height_map_node:main',
            'victim_detector_node = slam_drone_m3.victim_detector_node:main',
        ],
    },
)
