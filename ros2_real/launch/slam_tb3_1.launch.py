from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_sim = get_package_share_directory('my_tb3_sim')
    slam_params_file = os.path.join(pkg_sim, 'config', 'slam_params.yaml')
    qos_profile_file = os.path.join(pkg_sim, 'config', 'qos_profiles.yaml')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Path to SLAM params'
    )

    return LaunchDescription([
        declare_slam_params,
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            namespace='tb3_1',
            parameters=[
                {'use_sim_time': True},
                {'slam_params_file': LaunchConfiguration('slam_params_file')},
                {'transform_timeout': 0.2, 'tf_buffer_duration': 30.0, 'message_filter_queue_size': 100},
                qos_profile_file if os.path.exists(qos_profile_file) else {}
            ],
            output='screen'
        )
    ])
