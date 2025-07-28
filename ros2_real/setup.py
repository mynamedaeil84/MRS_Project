from setuptools import setup
from glob import glob
import os

package_name = 'my_tb3_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # my_tb3_sim/__init__.py 반드시 필요
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/resource', ['resource/my_tb3_sim']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel',
    maintainer_email='daniel@example.com',
    description='Warehouse scenario for multi-robot system with ROS2 and Gazebo Classic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_info_publisher.py = my_tb3_sim.obstacle_info_publisher:main',
            'map_update_publisher.py = my_tb3_sim.map_update_publisher:main',
            'hazard_zone_publisher.py = my_tb3_sim.hazard_zone_publisher:main',
            'task_allocation_publisher.py = my_tb3_sim.task_allocation_publisher:main',
            'obstacle_info_subscriber.py = my_tb3_sim.obstacle_info_subscriber:main',
            'evaluation_logger.py = my_tb3_sim.evaluation_logger:main',
            'auctioneer_node.py = my_tb3_sim.auctioneer_node:main',
            'bidder_node.py = my_tb3_sim.bidder_node:main',
            'frontier_explorer.py = my_tb3_sim.frontier_explorer:main'
        ],
    },
)
