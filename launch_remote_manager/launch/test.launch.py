# TODO(ngmor) remove

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_remote_manager import LaunchRemote

def generate_launch_description():
    # Inputs
    machine = 'galagongm'
    package = 'launch_remote_ssh'
    launch_file = 'test_remotely_launched.launch.py'
    install_dirs = ['/home/ngm/classes/final/remote_ws/install']
    launch_arguments=[
        ('param1', '-0.567'),
        ('param2', 'this is a sentence!'),
        ('param3', '453'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_raw_values',
            default_value='true',
            choices=['true','false'],
        ),
        # Test with raw values
        LaunchRemote(
            machine=machine,
            package=package,
            launch_file=launch_file,
            launch_arguments=launch_arguments,
            install_dirs=install_dirs,
            condition=IfCondition(LaunchConfiguration('use_raw_values')),
        ),

        # Test with launch configurations
        SetLaunchConfiguration(
            name='machine',
            value=machine
        ),
        SetLaunchConfiguration(
            name='package',
            value=package
        ),
        SetLaunchConfiguration(
            name='launch_file',
            value=launch_file
        ),
        SetLaunchConfiguration(
            name='install_dir',
            value=install_dirs[0]
        ),
        SetLaunchConfiguration(
            name='argname0',
            value=launch_arguments[0][0]
        ),
        SetLaunchConfiguration(
            name='argval0',
            value='different sentence?'
        ),
        SetLaunchConfiguration(
            name='argname1',
            value=launch_arguments[1][0]
        ),
        SetLaunchConfiguration(
            name='argval1',
            value='-106'
        ),
        SetLaunchConfiguration(
            name='argname2',
            value=launch_arguments[2][0]
        ),
        SetLaunchConfiguration(
            name='argval2',
            value='-0.2341'
        ),
        LaunchRemote(
            machine=LaunchConfiguration('machine'),
            package=LaunchConfiguration('package'),
            launch_file=LaunchConfiguration('launch_file'),
            launch_arguments=[
                (LaunchConfiguration('argname0'), LaunchConfiguration('argval0')),
                (LaunchConfiguration('argname1'), LaunchConfiguration('argval1')),
                (LaunchConfiguration('argname2'), LaunchConfiguration('argval2')),
            ],
            install_dirs=[LaunchConfiguration('install_dir')],
            condition=UnlessCondition(LaunchConfiguration('use_raw_values')),
        ),
    ])