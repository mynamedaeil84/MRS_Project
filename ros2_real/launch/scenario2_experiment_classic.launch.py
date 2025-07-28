from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_sim = get_package_share_directory('my_tb3_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    warehouse = os.path.join(pkg_sim, 'worlds', 'warehouse.world')
    slam_params = os.path.join(pkg_sim, 'config', 'slam_params.yaml')
    qos_profile = os.path.join(pkg_sim, 'config', 'qos_profiles.yaml')

    declare_robot_model = DeclareLaunchArgument(
        'robot_model', default_value='burger',
        description='Type of TurtleBot3 robot'
    )

    return LaunchDescription([
        declare_robot_model,

        # Gazebo 환경 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'empty_wolrd.launch.py')
            ),
            launch_arguments={
                'world': warehouse,
                'verbose': 'true',
                'gui': 'true'
            }.items()
        ),

        # TurtleBot3 두 대 스폰
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_sim, 'launch', 'spawn_tb3s.launch.py')
                    )
                )
            ]
        ),

        # TB3-1 SLAM
        TimerAction(
            period=4.0,
            actions=[Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                namespace='tb3_1',
                parameters=[
                    {'use_sim_time': True},
                    {'slam_params_file': slam_params},
                    {'transform_timeout': 0.2, 'tf_buffer_duration': 30.0, 'message_filter_queue_size': 100},
                    qos_profile if os.path.exists(qos_profile) else {}
                ],
                output='screen'
            )]
        ),

        # 사용자 노드들 순차 실행
        TimerAction(period=6.0, actions=[
            Node(package='my_tb3_sim', executable='obstacle_info_publisher', output='screen'),
            Node(package='my_tb3_sim', executable='map_update_publisher', output='screen'),
            Node(package='my_tb3_sim', executable='hazard_zone_publisher', output='screen'),
            Node(package='my_tb3_sim', executable='task_allocation_publisher', output='screen')
        ]),

        TimerAction(period=7.0, actions=[
            Node(package='my_tb3_sim', executable='obstacle_info_subscriber', output='screen')
        ]),

        TimerAction(period=8.0, actions=[
            Node(package='multirobot_map_merge', executable='map_merge', output='screen',
                 parameters=[{
                     'known_init_poses': False,
                     'merging_rate': 0.3,
                     'robot_map_topic': 'map_update',
                     'robot_namespace': '',
                     'world_frame': 'map',
                     'qos_reliability': 'reliable',
                     'qos_durability': 'transient_local',
                     'qos_history': 'keep_last',
                     'qos_depth': 50
                 }])
        ]),

        TimerAction(period=9.0, actions=[
            Node(package='my_tb3_sim', executable='evaluation_logger', output='screen')
        ])
    ])
