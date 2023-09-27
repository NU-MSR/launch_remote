# TODO(ngmor) remove

from launch import LaunchDescription
from launch_remote_manager import LaunchRemote

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

    return LaunchDescription([
        LaunchRemote(
            machine=machine,
            package=package,
            launch_file=file,
            launch_arguments=launch_arguments,
            install_dirs=install_dirs,
        )
    ])