import os
from glob import glob
from setuptools import setup

package_name = 'drone_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'models', 'walker'), glob('models/walker/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kishan Ballal',
    maintainer_email='kishanrajballal22@gmail.com',
    description='M2c: cost-aware planner (distance, risk, battery, complexity) + dynamic obstacles',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cost_planner = drone_planning.cost_planner:main',
            'obstacle_mover = drone_planning.obstacle_mover:main',
            'dynamic_tracker = drone_planning.dynamic_tracker:main',
        ],
    },
)
