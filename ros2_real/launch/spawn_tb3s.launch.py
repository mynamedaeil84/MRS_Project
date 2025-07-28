from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import os
os.environ['RCUTILS_LOGGING_BUFFERED_STREAM'] = '1'
os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '1'
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

def generate_launch_description():
    pkg_my_sim = get_package_share_directory('my_tb3_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(pkg_my_sim, 'worlds', 'warehouse.world')
    urdf_file_tb3_1 = os.path.join(pkg_my_sim, 'urdf', 'turtlebot3_waffle_pi_tb3_1.urdf')
    urdf_file_tb3_2 = os.path.join(pkg_my_sim, 'urdf', 'turtlebot3_waffle_pi_tb3_2.urdf')
    
    # 1️⃣ Gazebo headless
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': 'false',
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )

    # 2️⃣ TB3-1 system launch
    tb3_1_system = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_my_sim, 'launch', 'tb3_1_system.launch.py')
                ),
                launch_arguments={
                    'urdf_file': urdf_file_tb3_1
                }.items()
            )
        ]
    )

    # 3️⃣ TB3-2 system launch
    tb3_2_system = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_my_sim, 'launch', 'tb3_2_system.launch.py')
                ),
                launch_arguments={
                    'urdf_file': urdf_file_tb3_2
                }.items()
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        tb3_1_system,
        tb3_2_system
    ])
