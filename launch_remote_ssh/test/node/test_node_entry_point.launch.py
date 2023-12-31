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

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, \
    Command
from launch.actions import SetLaunchConfiguration, DeclareLaunchArgument, LogInfo
from launch_ros.parameter_descriptions import Parameter, ParameterFile
from launch_ros.substitutions import FindPackagePrefix
from launch_remote_ssh import NodeRemoteSSH, FindPackageShareRemote
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
        # Typically, copying of the install space should not be performed in a launch file.
        # Best practice would be to copy the install space during the build process, for example with a
        # colcon extension.

        # This copy command workaround is only here to simplify running this test.
        SetLaunchConfiguration(
            name='copy_command',
            value=[
                'ros2 run launch_remote_ssh copy_install_space.py ',
                LaunchConfiguration('user'),
                ' ',
                LaunchConfiguration('machine'),
                ' ',
                FindPackagePrefix('launch_remote_ssh'),
                '/ ',
                LaunchConfiguration('remote_install_space'),
                '/launch_remote_ssh/ -d'
            ]
        ),
        LogInfo(
            msg=Command(command=LaunchConfiguration('copy_command'))
        ),
        SetLaunchConfiguration(
            name='num_params1',
            value='25'
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
                    FindPackageShareRemote(
                        LaunchConfiguration('remote_install_space'),
                        'launch_remote_ssh'
                    ),
                    'local_setup.bash'
                ])
            ],
            name='param_node1',
            namespace='param_nodes',
            parameters=[
                {'num_params': LaunchConfiguration('num_params1')},
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
                    FindPackageShareRemote(
                        LaunchConfiguration('remote_install_space'),
                        'launch_remote_ssh'
                    ),
                    'test',
                    'test_node_params1.yaml'
                ]),
                ParameterFile(
                    PathJoinSubstitution([
                        FindPackageShareRemote(
                            LaunchConfiguration('remote_install_space'),
                            'launch_remote_ssh'
                        ),
                        'test',
                        'test_node_params2.yaml'
                    ]),
                ),
                Parameter('param24', 1337),
            ]
        ),
        SetLaunchConfiguration(
            name='num_params2',
            value='1'
        ),
        NodeRemoteSSH(
            user=LaunchConfiguration('user'),
            machine=LaunchConfiguration('machine'),
            package='launch_remote_ssh',
            executable='param_node.py',
            source_paths=[
                PathJoinSubstitution([
                    FindPackageShareRemote(
                        LaunchConfiguration('remote_install_space'),
                        'launch_remote_ssh'
                    ),
                    'local_setup.bash'
                ])
            ],
            name='param_node2',
            namespace='param_nodes',
            arguments=[
                ' --ros-args ',
                ' -p ',
                [' num_params:=', LaunchConfiguration('num_params2')]
            ],
            parameters=[{'param0': 'Success'}],
        ),
        SetLaunchConfiguration(
            name='num_services1',
            value='5'
        ),
        SetLaunchConfiguration(
            name='srv3_name',
            value='srv3'
        ),
        SetLaunchConfiguration(
            name='srv4_remap',
            value='~/fifth_service'
        ),
        NodeRemoteSSH(
            user=LaunchConfiguration('user'),
            machine=LaunchConfiguration('machine'),
            package='launch_remote_ssh',
            executable='service_node.py',
            source_paths=[
                PathJoinSubstitution([
                    FindPackageShareRemote(
                        LaunchConfiguration('remote_install_space'),
                        'launch_remote_ssh'
                    ),
                    'local_setup.bash'
                ])
            ],
            name='service_node1',
            namespace='service_nodes',
            ros_arguments=[
                ' --log-level ',
                ' WARN ',
                ' -p ',
                [' num_services:=',LaunchConfiguration('num_services1')],
            ],
            remappings=[
                ('srv0', 'first_service'),
                ('srv1', TextSubstitution(text='second_service')),
                (TextSubstitution(text='srv2'), TextSubstitution(text='third_service')),
                (LaunchConfiguration('srv3_name'), '~/fourth_service'),
                ('srv4', LaunchConfiguration('srv4_remap'))
            ]
        ),
        DeclareLaunchArgument(
            name='test_duration',
            default_value='10.0',
        ),
        Catch2IntegrationTestNode(
            package='launch_remote_ssh',
            executable='test_node_tester_node',
            parameters=[{
                'test_duration': LaunchConfiguration('test_duration'),
                'num_params1': LaunchConfiguration('num_params1'),
                'num_params2': LaunchConfiguration('num_params2'),
                'num_services1': LaunchConfiguration('num_services1'),
            }],
        )
    ])