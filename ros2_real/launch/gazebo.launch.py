from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    world_file_name = 'warehouse.world'
    world_path = os.path.join(
        FindPackageShare('my_tb3_sim').find('my_tb3_sim'),
        'worlds',
        world_file_name
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world_path,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])
