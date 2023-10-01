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

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import SetLaunchConfiguration, DeclareLaunchArgument
from launch_ros.substitutions import FindPackagePrefix
from launch_remote_ssh import LaunchRemoteSSH, CopySinglePackageInstall, FindPackageShareRemote
from launch_catch_ros2 import Catch2LaunchDescription, Catch2IntegrationTestNode


def generate_launch_description():
    return Catch2LaunchDescription([
        DeclareLaunchArgument(
            name='user',
        ),
        DeclareLaunchArgument(
            name='machine',
        ),
        SetLaunchConfiguration(
            name='remote_install_space',
            value='/tmp/launch_remote_ssh_test/install'
        ),
        CopySinglePackageInstall(
            user=LaunchConfiguration('user'),
            machine=LaunchConfiguration('machine'),
            package='launch_remote_ssh',
            remote_install_space=LaunchConfiguration('remote_install_space'),
            remove_preexisting='true'
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
        LaunchRemoteSSH(
            user=LaunchConfiguration('user'),
            machine=LaunchConfiguration('machine'),
            package='launch_remote_ssh',
            file='test_launch_remotely_launched.launch.py',
            source_paths=[
                PathJoinSubstitution([
                    FindPackageShareRemote(
                        LaunchConfiguration('remote_install_space'),
                        'launch_remote_ssh'
                    ),
                    'local_setup.bash'
                ])
            ],
            launch_arguments=[
                ('param1', LaunchConfiguration('param1')),
                ('param2', LaunchConfiguration('param2')),
                ('param3', LaunchConfiguration('param3')),
            ]
        ),
        DeclareLaunchArgument(
            name='test_duration',
            default_value='20.0',
        ),
        Catch2IntegrationTestNode(
            package='launch_remote_ssh',
            executable='test_launch_tester_node',
            parameters=[{
                'test_duration': LaunchConfiguration('test_duration'),
                'param1': LaunchConfiguration('param1'),
                'param2': LaunchConfiguration('param2'),
                'param3': LaunchConfiguration('param3'),
            }],
        )
    ])