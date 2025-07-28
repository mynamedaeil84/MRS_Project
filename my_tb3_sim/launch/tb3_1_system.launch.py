from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            namespace='tb3_1',
            output='screen'
        ),
        Node(
            package='my_tb3_sim',
            executable='obstacle_info_publisher',
            namespace='tb3_1',
            output='screen'
        )
    ])
