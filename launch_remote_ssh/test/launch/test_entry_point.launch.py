# BSD 3-Clause License
#
# Copyright (c) 2023, Northwestern University MSR
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Nick Morales

import sys

from launch.substitutions import LaunchConfiguration
from launch.actions import SetLaunchConfiguration, DeclareLaunchArgument
from launch_remote_ssh import LaunchRemote, copy_single_package_install
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

    remote_install_space = '/home/' + user + '/launch_remote_ssh_test/install'

    # Copy files to remote install space
    copy_single_package_install(user, machine, 'launch_remote_ssh', remote_install_space)

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
                remote_install_space + '/launch_remote_ssh/share/launch_remote_ssh/local_setup.bash',
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