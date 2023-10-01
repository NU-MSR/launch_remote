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
from typing import Tuple
from typing import List

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.frontend import expose_action
from launch.frontend import Entity
from launch.frontend import Parser
from launch.condition import Condition
from launch.utilities import normalize_to_list_of_substitutions

from .replace_text_substitution import ReplaceTextSubstitution
from .execute_process_remote_ssh import ExecuteProcessRemoteSSH

@expose_action('launch_remote_ssh')
class LaunchRemoteSSH(ExecuteProcessRemoteSSH):
    def __init__(
        self, *,
        user : SomeSubstitutionsType,
        machine : SomeSubstitutionsType,
        package : SomeSubstitutionsType,
        file : SomeSubstitutionsType,
        launch_arguments: Optional[
            Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]
        ] = None,
        port : Optional[SomeSubstitutionsType] = None,
        source_paths : Optional[Iterable[SomeSubstitutionsType]] = None,
        condition : Optional[Condition] = None
    ):
        self.__package = normalize_to_list_of_substitutions(package)
        self.__file = normalize_to_list_of_substitutions(file)

        # Generate run node command
        command = ['ros2 launch ']
        command += self.__package
        command.append(' ')
        command += self.__file

        if launch_arguments is not None:
            for argument in launch_arguments:
                command += [
                    ' ',
                    ReplaceTextSubstitution(  # escape spaces
                        normalize_to_list_of_substitutions(argument[0]),
                        ' ',
                        '\ '
                    ),
                    ':=',
                    ReplaceTextSubstitution(  # escape spaces
                        normalize_to_list_of_substitutions(argument[1]),
                        ' ',
                        '\ '
                    ),
                ]

        # ExecuteRemoteProcess
        super().__init__(
            user=user,
            machine=machine,
            command=command,
            port=port,
            source_paths=source_paths,
            condition=condition,
        )

    @classmethod
    def parse(
        self,
        entity : Entity,
        parser : Parser
    ):
        # Adapted from parse method here:
        # https://github.com/ros2/launch/blob/rolling/launch/launch/actions/include_launch_description.py
        _, kwargs = super().parse(entity, parser, ignore=['command'])

        kwargs['package'] = parser.parse_substitution(entity.get_attr('package'))
        kwargs['file'] = parser.parse_substitution(entity.get_attr('file'))

        args = entity.get_attr('arg', data_type=List[Entity], optional=True)
        if args is not None:
            kwargs['launch_arguments'] = [
                (
                    parser.parse_substitution(e.get_attr('name')),
                    parser.parse_substitution(e.get_attr('value')),
                )
                for e in args
            ]
            for e in args:
                e.assert_entity_completely_parsed()

        return self, kwargs