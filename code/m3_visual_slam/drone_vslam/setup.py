import os
from glob import glob
from setuptools import setup

package_name = 'drone_vslam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kishan Ballal',
    maintainer_email='kishanrajballal22@gmail.com',
    description='M3v: RTAB-Map RGB-D visual SLAM with a memory-guarded Gazebo camera',
    license='MIT',
    entry_points={'console_scripts': ['camera_guard = drone_vslam.camera_guard:main']},
)
