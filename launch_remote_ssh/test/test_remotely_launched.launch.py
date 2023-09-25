from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='param1',
            default_value='parameter1value',
        ),
        Node(
            package='launch_remote_ssh',
            executable='param_tester.py',
            name='param1_tester',
            parameters=[{'param': LaunchConfiguration('param1')}],
        ),
        DeclareLaunchArgument(
            name='param2',
            default_value='parameter2value',
        ),
        Node(
            package='launch_remote_ssh',
            executable='param_tester.py',
            name='param2_tester',
            parameters=[{'param': LaunchConfiguration('param2')}],
        ),
        DeclareLaunchArgument(
            name='param3',
            default_value='parameter2value',
        ),
        Node(
            package='launch_remote_ssh',
            executable='param_tester.py',
            name='param3_tester',
            parameters=[{'param': LaunchConfiguration('param3')}],
        ),
    ])
