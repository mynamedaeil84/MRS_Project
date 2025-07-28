import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ns = 'tb3_2'
    use_sim_time = True

    pkg_dir = get_package_share_directory('my_tb3_sim')

    nav2_yaml_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    slam_yaml_path = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    ctrl_yaml_path = os.path.join(pkg_dir, 'config', 'tb3_2_controllers.yaml')

    urdf_path = os.path.join(pkg_dir, 'urdf', 'turtlebot3_waffle_pi_tb3_2.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Gazebo에 엔티티 스폰
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', ns,
                '-file', urdf_path,
                '-x', '0.0', '-y', '0.0', '-z', '0.01',
                '-robot_namespace', ns
            ],
            output='screen'
        ),

        # robot_state_publisher: robot_description 토픽 publish
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description_content}
            ],
            remappings=[
                ('/robot_description', f'/{ns}/robot_description'),
            ]
        ),

        # ros2_control_node: 토픽 기반 robot_description 사용 (param X)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=ns,
            parameters=[os.path.join(pkg_dir, 'config', 'tb3_2_controllers.yaml')],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            namespace=ns,
            arguments=['joint_state_broadcaster', '--controller-manager', f'/{ns}/controller_manager'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            namespace=ns,
            arguments=['diff_drive_base', '--controller-manager', f'/{ns}/controller_manager'],
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            namespace=ns,
            parameters=[slam_yaml_path, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=ns,
            parameters=[nav2_yaml_path, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            namespace=ns,
            parameters=[nav2_yaml_path, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace=ns,
            parameters=[nav2_yaml_path, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=ns,
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['controller_server', 'planner_server', 'bt_navigator']
            }],
            output='screen'
        ),
   ])
