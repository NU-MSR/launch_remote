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

from typing import Optional
from typing import Iterable
from typing import Text
from typing import List

from uuid import uuid4

from launch.some_substitutions_type import SomeSubstitutionsType
from launch import LaunchDescription
from launch.frontend import expose_action
from launch.frontend import Entity
from launch.frontend import Parser
from launch.action import Action
from launch.condition import Condition
from launch.actions import ExecuteProcess
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions import Node

from .replace_text_substitution import ReplaceTextSubstitution

# https://answers.ros.org/question/364152/remotely-launch-nodes-in-ros2/

@expose_action('execute_process_remote_ssh')
class ExecuteProcessRemoteSSH(LaunchDescription):
    def __init__(
        self, *,
        user: SomeSubstitutionsType,
        machine: SomeSubstitutionsType,
        command: Iterable[SomeSubstitutionsType],
        port: Optional[SomeSubstitutionsType] = None,
        source_paths: Optional[Iterable[SomeSubstitutionsType]] = None,
        condition: Optional[Condition] = None
    ):
        # Store arguments
        self.__user = normalize_to_list_of_substitutions(user)
        self.__machine = normalize_to_list_of_substitutions(machine)
        self.__command = command
        self.__port = None if port is None else normalize_to_list_of_substitutions(port)
        self.__source_paths = [] if source_paths is None else source_paths
        self.__condition = condition
        self.__uuid = uuid4()

        # Use a custom port if specified
        port_list = []
        if self.__port is not None:
            port_list.append(' -p ')
            port_list += self.__port

        # Compile process name into list with shortened UUID at the end
        # TODO(anyone) - this has a max of 80 characters. Can this be enforced still using
        # substitutions?
        process_name_list = []
        process_name_list += self.__machine
        process_name_list += [
            '_',
            self.uuid_short
        ]

        # Replace any dots with underscores
        process_name_list = [
            ReplaceTextSubstitution(
                normalize_to_list_of_substitutions(process_name_list),
                '.',
                '_',
            )
        ]

        # Compile source paths
        source_path_commands = []

        for path in self.__source_paths:
            source_path_commands.append('source '),
            source_path_commands.extend(normalize_to_list_of_substitutions(path)),
            source_path_commands.append(' && ')

        # Compile command into list
        command_list = []

        for cmd in self.__command:
            command_list += normalize_to_list_of_substitutions(cmd)

        # Build full command
        self.__full_command = [
            '{ outer_stdout=$(readlink -f /proc/self/fd/3); } 3>&1 && screen -DmS ',
        ]
        self.__full_command += process_name_list
        self.__full_command += [
            ' bash -i -c "ssh ',
        ]
        self.__full_command += port_list
        self.__full_command += [
            ' -t ',
        ]
        self.__full_command += self.__user
        self.__full_command += [
            '@',
        ]
        self.__full_command += self.__machine
        self.__full_command += [
            ' \'bash -i -c \\"'
        ]
        self.__full_command += source_path_commands
        self.__full_command += command_list
        self.__full_command += [
            '\\"\' > $outer_stdout"',
        ]

        super().__init__(
            initial_entities = [
                ExecuteProcess(
                    name=process_name_list,
                    cmd=[self.__full_command],
                    output="screen",
                    shell=True,
                    emulate_tty=True,
                    condition=self.__condition,
                ),
                Node(
                    package='launch_remote_ssh',
                    executable='remote_process_handler',
                    name='remote_process_handler_' + self.uuid_short,
                    namespace=ReplaceTextSubstitution(self.__machine, '.', '_'),
                    output='screen',
                    parameters=[{'screen_process_name': process_name_list}],
                    condition=self.__condition,
                ),
            ]
        )

    @property
    def uuid_full(self) -> Text:
        """Getter for full uuid."""
        return f'{self.__uuid.int:x}'

    @property
    def uuid_short(self) -> Text:
        """Getter for short uuid."""
        return f'{self.uuid_full:.12}'

    @classmethod
    def parse(
        self,
        entity: Entity,
        parser: Parser,
        ignore: Optional[List[str]] = None
    ):
        # Adapted from parse method here:
        # https://github.com/ros2/launch/blob/rolling/launch/launch/actions/execute_process.py

        # Even though this is not derived from the Action class,
        # we're treating it like an action, so use the Action class
        # parsing method to get the condition kwarg
        _, kwargs = Action().parse(entity, parser)

        if ignore is None:
            ignore = []

        if 'user' not in ignore:
            kwargs['user'] = parser.parse_substitution(entity.get_attr('user'))

        if 'machine' not in ignore:
            kwargs['machine'] = parser.parse_substitution(entity.get_attr('machine'))

        if 'command' not in ignore:
            kwargs['command'] = self._parse_cmdline(entity.get_attr('command'), parser)

        if 'port' not in ignore:
            port = entity.get_attr('port', optional=True)
            if port is not None:
                kwargs['port'] = parser.parse_substitution(port)
        
        if 'source_paths' not in ignore:
            source_paths = entity.get_attr('source_path', data_type=List[Entity], optional=True)
            if source_paths is not None:
                kwargs['source_paths'] = [
                    parser.parse_substitution(e.get_attr('path'))
                    for e in source_paths
                ]
                for e in source_paths:
                    e.assert_entity_completely_parsed()

        return self, kwargs
    
    @classmethod
    def _parse_cmdline(
        self,
        cmd: Text,
        parser: Parser
    ) -> List[SomeSubstitutionsType]:
        # TODO(anyone) this is not yet well tested

        result = []
        for sub in parser.parse_substitution(cmd):
            result.append(sub)

        return result