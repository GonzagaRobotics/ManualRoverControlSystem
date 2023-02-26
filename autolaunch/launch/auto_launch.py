import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    telop_twist_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'launch'),
            '/teleop-launch.py']),launch_arguments={'joy_config': 'xbox','joy_dev':'dev/inputs/js1'}.items(),
      )
    return LaunchDescription([
        telop_twist_joy,
        Node(
            name='CommandReciever',
            package='jetson_comm',
            executable='CommandReciever',
            output='screen'
        ),
        Node(
            name='XboxBroker',
            package='command_broker',
            executable='XboxBroker',
            output='screen'
        ),
        Node(
            name='CommandExposer',
            package='command_exposer',
            executable='CommandExposer',
            output='screen'
        )

    ])

ld = generate_launch_description()
