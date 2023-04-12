import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare command line arguments for specifying which nodes to launch
    declare_xbox_broker_cmd = DeclareLaunchArgument(
        'xbox_broker',
        default_value='True',
        description='Whether to launch the XboxBroker node')
    declare_command_exposer_cmd = DeclareLaunchArgument(
        'command_exposer',
        default_value='True',
        description='Whether to launch the CommandExposer node')
    declare_command_receiver_cmd = DeclareLaunchArgument(
        'command_receiver',
        default_value='True',
        description='Whether to launch the CommandReceiver node')
    declare_telop_twist_joy_cmd = DeclareLaunchArgument(
        'telop_twist_joy',
        default_value='True',
        description='Whether to launch the Teleop Twist node')
    declare_realsense2_camera_cmd = DeclareLaunchArgument(
        'realsense2_camera',
        default_value='True',
        description='Whether to launch the realsense2_camera node')
    
    declare_camerasub_cmd = DeclareLaunchArgument(
        'camerasub',
        default_value='True',
        description='Whether to launch the realsense_sub node')
    
    camerasub = Node(
            package='camerasub',
            executable='subber',
            name='subber',
            condition=IfCondition(LaunchConfiguration('camerasub'))
        )
    # Include teleop_twist_joy.launch.py with arguments for joy_config and joy_dev
    telop_twist_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'launch'),
            '/teleop-launch.py']),
            launch_arguments={'joy_config': 'xbox','joy_dev':'/dev/input/js1'}.items(),
            condition=IfCondition(LaunchConfiguration('telop_twist_joy'))
      )
    # Include rs_launch.py with arguments
    realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
            #launch_arguments={'depth_module.profile':'640x480x30'}.items(),
            condition=IfCondition(LaunchConfiguration('realsense2_camera'))
            
      )

    # Define XboxBroker node with conditional launching based on command line argument value
    xbox_broker = Node(
        name='XboxBroker',
        package='command_broker',
        executable='XboxBroker',
        output='screen',
        condition=IfCondition(LaunchConfiguration('xbox_broker'))
    )

    # Define CommandExposer node with conditional launching based on command line argument value
    command_exposer = Node(
        name='CommandExposer',
        package='command_exposer',
        executable='CommandExposer',
        output='screen',
         condition=IfCondition(LaunchConfiguration('command_exposer'))
     )

     # Define CommandReceiver node with conditional launching based on command line argument value
    command_receiver = Node(
         name="CommandReceiver",
         package="jetson_comm",
         executable="CommandReceiver",
         output="screen",
         condition=IfCondition(LaunchConfiguration('command_receiver'))
     )
    
    return LaunchDescription([
        declare_xbox_broker_cmd,
        declare_command_exposer_cmd,
        declare_command_receiver_cmd,
        declare_telop_twist_joy_cmd,
        declare_realsense2_camera_cmd,
        declare_camerasub_cmd,
        telop_twist_joy,
        realsense2_camera,
        camerasub,
        xbox_broker,
        command_exposer,
        command_receiver,
     ])
ld = generate_launch_description()
