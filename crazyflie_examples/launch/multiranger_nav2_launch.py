import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    map_name = 'map'

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('crazyflie'), 'launch'
                        ),
                        '/launch.py',
                    ]
                ),
                launch_arguments={
                    'backend': 'cflib',
                    'gui': 'false',
                    'teleop': 'false',
                    'mocap': 'false',
                }.items(),
            ),
            Node(
                package='crazyflie',
                executable='vel_mux.py',
                name='vel_mux',
                output='screen',
                parameters=[
                    {'hover_height': 0.3},
                    {'incoming_twist_topic': '/cmd_vel'},
                    {'robot_prefix': '/cf231'},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch/online_async_launch.py',
                    )
                ),
                launch_arguments={
                    'slam_params_file': os.path.join(
                        get_package_share_directory('crazyflie_examples'),
                        'config/slam_params.yaml',
                    ),
                    'use_sim_time': 'False',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch/bringup_launch.py',
                    )
                ),
                launch_arguments={
                    'slam': 'False',
                    'use_sim_time': 'False',
                    'map': get_package_share_directory('crazyflie_examples')
                    + '/data/'
                    + map_name
                    + '.yaml',
                    'params_file': os.path.join(
                        get_package_share_directory('crazyflie_examples'),
                        'config/nav2_params.yaml',
                    ),
                    'autostart': 'True',
                    'use_composition': 'True',
                    'transform_publish_period': '0.02',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch/rviz_launch.py',
                    )
                ),
                launch_arguments={
                    'rviz_config': os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'rviz',
                        'nav2_default_view.rviz',
                    )
                }.items(),
            ),
        ]
    )
