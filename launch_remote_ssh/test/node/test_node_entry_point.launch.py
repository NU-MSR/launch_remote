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

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import SetLaunchConfiguration, DeclareLaunchArgument
from launch_remote_ssh import NodeRemoteSSH, copy_single_package_install, FindPackageShareRemote
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

    remote_install_space = '/tmp/launch_remote_ssh_test/install'

    # Copy files to remote install space
    copy_single_package_install(user, machine, 'launch_remote_ssh', remote_install_space, True)

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
            name='param10_name',
            value='param10'
        ),
        SetLaunchConfiguration(
            name='param10_value',
            value='hello world'
        ),
        SetLaunchConfiguration(
            name='param11_name',
            value='param11'
        ),
        SetLaunchConfiguration(
            name='param11_value',
            value='true'
        ),
        SetLaunchConfiguration(
            name='param12_name',
            value='param12'
        ),
        SetLaunchConfiguration(
            name='param12_value',
            value='45'
        ),
        SetLaunchConfiguration(
            name='param13_value',
            value='False'
        ),
        SetLaunchConfiguration(
            name='param14_value',
            value='3'
        ),
        SetLaunchConfiguration(
            name='param15_value',
            value='8.0'
        ),
        NodeRemoteSSH(
            user=LaunchConfiguration('user'),
            machine=LaunchConfiguration('machine'),
            package='launch_remote_ssh',
            executable='param_node.py',
            source_paths=[
                PathJoinSubstitution([
                    remote_install_space,
                    'launch_remote_ssh',
                    'share',
                    'launch_remote_ssh',
                    'local_setup.bash'
                ])
            ],
            name='param_node1',
            namespace='param_nodes',
            parameters=[
                {'num_params': 24},
                {
                    'param0': False,
                    'param1': 'happy!',
                    'param2': -256,
                    'param3': 42.42,
                    'param4': [False, True, False, True, True],
                    'param5': ['I', 'like', 'when', 'things', 'work'],
                    'param6': ['But ', TextSubstitution(text='this is'), ' a ', 'single', ' string'],
                    'param7': [9,8,7,6,5,4,3,2,1],
                    'param8': [9.0,8,7,6,5,4,3,2,1],
                    'param9': ['This', [TextSubstitution(text='is'), 'not'], 'a', 'single', 'string'],
                    LaunchConfiguration('param10_name'): LaunchConfiguration('param10_value'),
                    LaunchConfiguration('param11_name'): LaunchConfiguration('param11_value'),
                    LaunchConfiguration('param12_name'): LaunchConfiguration('param12_value'),
                    'param13': [False, True, False, True, LaunchConfiguration('param13_value')],
                    'param14': [1,2,LaunchConfiguration('param14_value'),4,5,6,7,8,9],
                    'param15': [1.0,2.0,3.0,4.0,5.0,6.0,7.0,LaunchConfiguration('param15_value'),9.0],
                },
                PathJoinSubstitution([
                    FindPackageShareRemote(remote_install_space, 'launch_remote_ssh'),
                    'test',
                    'test_node_params1.yaml'
                ])
            ]
        ),
        
        # Catch2IntegrationTestNode(
        #     package='launch_remote_ssh',
        #     executable='test_launch_tester_node',
        #     parameters=[{
        #         'test_duration': LaunchConfiguration('test_duration'),
        #         'param1': LaunchConfiguration('param1'),
        #         'param2': LaunchConfiguration('param2'),
        #         'param3': LaunchConfiguration('param3'),
        #     }],
        # )
    ])