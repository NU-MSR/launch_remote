# TODO(ngmor) convert into launch action

from uuid import uuid4

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    machine = 'galagongm'
    uuid = uuid4()
    package = 'launch_remote_ssh'
    file = 'test_remotely_launched.launch.py'

    return LaunchDescription([
        Node(
            package='launch_remote_manager',
            executable='launch_client',
            output='screen', # TODO(ngmor) allow this to be passed through
            name='launch_client_' + f'{uuid.int:x}',
            namespace='/' + machine,
            parameters=[{
                'package' : package,
                'file' : file,
            }]
        )
    ])