from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_tb3_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(), 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='TB3 minimal experiment simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'log_results = my_tb3_sim.log_results:main',
            'obstacle_info_publisher = my_tb3_sim.obstacle_info_publisher:main',
            'obstacle_info_marker = my_tb3_sim.obstacle_info_marker:main',
            'reaction_logger = my_tb3_sim.reaction_logger:main',
        ],
    },
)

