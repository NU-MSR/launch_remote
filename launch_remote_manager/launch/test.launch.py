# TODO(ngmor) convert into launch action

from uuid import uuid4

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Inputs
    machine = 'galagongm'
    package = 'launch_remote_ssh'
    file = 'test_remotely_launched.launch.py'
    # install_dirs = [''] # TODO(ngmor) must pass this in if empty
    install_dirs = ['/home/ngm/classes/final/remote_ws/install']
    # TODO(ngmor) going to have to process these in the same way as LaunchRemote
    launch_arguments=[
        ('param1', '-0.567'),
        ('param2', 'this is a sentence!'),
        ('param3', '453'),
    ]


    # Logic
    uuid = uuid4()

    parameters = {}
    parameters['package'] = package
    parameters['file'] = file
    parameters['install_dirs'] = install_dirs

    for i, argument in enumerate(launch_arguments):
        parameters[f'argname{i}'] = argument[0]
        parameters[f'argval{i}'] = argument[1]

    return LaunchDescription([
        Node(
            package='launch_remote_manager',
            executable='launch_client',
            output='screen', # TODO(ngmor) allow this to be passed through
            name='launch_client_' + f'{uuid.int:x}',
            namespace='/' + machine,
            parameters=[parameters]
        )
    ])