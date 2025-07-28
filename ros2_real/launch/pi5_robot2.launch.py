from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_sim',
            executable='obstacle_info_subscriber.py',
            name='obstacle_info_subscriber',
            namespace='robot2',
            parameters=['config/qos_profile_A.yaml'],
            output='screen'),
        Node(
            package='ros2_sim',
            executable='map_update_subscriber.py',
            name='map_update_subscriber',
            namespace='robot2',
            parameters=['config/qos_profile_A.yaml'],
            output='screen'),
        Node(
            package='ros2_sim',
            executable='goal_pose_subscriber.py',
            name='goal_pose_subscriber',
            namespace='robot2',
            parameters=['config/qos_profile_A.yaml'],
            output='screen'),
        Node(
            package='ros2_sim',
            executable='evaluation_logger.py',
            name='evaluation_logger',
            output='screen'),
    ])
