from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_sim',
            executable='obstacle_info_publisher.py',
            name='obstacle_info_publisher',
            namespace='robot1',
            parameters=['config/qos_profile_A.yaml'],
            output='screen'),
        Node(
            package='ros2_sim',
            executable='map_update_publisher.py',
            name='map_update_publisher',
            namespace='robot1',
            parameters=['config/qos_profile_A.yaml'],
            output='screen'),
        Node(
            package='ros2_sim',
            executable='goal_pose_publisher.py',
            name='goal_pose_publisher',
            namespace='robot1',
            parameters=['config/qos_profile_A.yaml'],
            output='screen'),
    ])
