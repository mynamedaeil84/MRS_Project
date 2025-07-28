from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace='tb3_2',
            output='screen'
        ),
        Node(
            package='my_tb3_sim',
            executable='reaction_logger',
            namespace='tb3_2',
            output='screen'
        )
    ])
