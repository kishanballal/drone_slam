from glob import glob
from setuptools import setup

package_name = 'drone_search'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kishan Ballal',
    maintainer_email='kishanrajballal22@gmail.com',
    description='M4: self-planned search route (room split, coverage stops, shortest order)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'search_route = drone_search.search_route_node:main',
        ],
    },
)
