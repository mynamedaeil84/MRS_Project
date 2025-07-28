from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('merging_rate', default_value='0.3'),
        DeclareLaunchArgument('world_frame', default_value='map'),

        Node(
            package='multirobot_map_merge',
            executable='map_merge',
            name='map_merge',
            output='screen',
            parameters=[{
                'known_init_poses': False,
                'merging_rate': LaunchConfiguration('merging_rate'),
                'robot_map_topic': 'map_update',
                'robot_namespace': '',
                'world_frame': LaunchConfiguration('world_frame'),
                'qos_reliability': 'reliable',
                'qos_durability': 'transient_local',
                'qos_history': 'keep_last',
                'qos_depth': 50
            }]
        )
    ])
