from ament_index_python.packages import get_package_share_directory
import sys
import subprocess

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import SetLaunchConfiguration, DeclareLaunchArgument
from launch_remote_ssh import LaunchRemote
from launch_catch_ros2 import Catch2LaunchDescription, Catch2IntegrationTestNode


def generate_launch_description():
    # Manually get user and machine arguments
    user = ''
    machine = ''
    
    for argv in sys.argv:
        if 'user:=' in argv:
            user = argv.replace('user:=', '')
        elif 'machine:=' in argv:
            machine = argv.replace('machine:=', '')

    if user == '' or machine == '':
        raise Exception("'user' and 'machine' arguments must be provided.")
    
    install_space = '/home/' + user + '/launch_remote_ssh_test/install/launch_remote_ssh/'
    user_machine = user + '@' + machine
    package_install = get_package_share_directory('launch_remote_ssh') + '/../../'

    # Create install space in home directory of user on machine
    subprocess.run(
        ['ssh ' + user_machine + ' "mkdir -p ' + install_space + '"'],
        shell=True,
    )

    # Copy files to install space
    subprocess.run(
        ['rsync -r ' + package_install + ' ' + user_machine + ':' + install_space],
        shell=True,
    )

    # Run launch test
    return Catch2LaunchDescription([
        DeclareLaunchArgument(
            name='test_duration',
            default_value='60.0',
        ),
        DeclareLaunchArgument(
            name='user',
        ),
        DeclareLaunchArgument(
            name='machine',
        ),
        SetLaunchConfiguration(
            name='param1',
            value='-0.567',
        ),
        SetLaunchConfiguration(
            name='param2',
            value='this is a sentence!',
        ),
        SetLaunchConfiguration(
            name='param3',
            value='453',
        ),
        LaunchRemote(
            user=LaunchConfiguration('user'),
            machine=LaunchConfiguration('machine'),
            package='launch_remote_ssh',
            launch_file='test_remotely_launched.launch.py',
            source_paths=[
                install_space + 'share/launch_remote_ssh/local_setup.bash',
            ],
            launch_arguments=[
                ('param1', LaunchConfiguration('param1')),
                ('param2', LaunchConfiguration('param2')),
                ('param3', LaunchConfiguration('param3')),
            ]
        ),
        Catch2IntegrationTestNode(
            package='launch_remote_ssh',
            executable='tester_node',
            parameters=[{
                'test_duration': LaunchConfiguration('test_duration'),
                'param1': LaunchConfiguration('param1'),
                'param2': LaunchConfiguration('param2'),
                'param3': LaunchConfiguration('param3'),
            }],
        )
    ])