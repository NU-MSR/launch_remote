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

from uuid import uuid4

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.condition import Condition
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions import Node

class LaunchRemote(Node):
    def __init__(
        self, *,
        machine : SomeSubstitutionsType,
        package : SomeSubstitutionsType,
        launch_file : SomeSubstitutionsType,
        launch_arguments : Optional[
            Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]
        ] = None,
        install_dirs : Optional[Iterable[SomeSubstitutionsType]] = None,
        condition : Optional[Condition] = None,
        output: SomeSubstitutionsType = 'screen',
    ):
        # Store arguments
        self.__machine = machine
        self.__package = package
        self.__launch_file = launch_file
        self.__uuid = uuid4()
        if (install_dirs is None) or (len(install_dirs) == 0):
            self.__install_dirs = ['']
        else:
            self.__install_dirs = install_dirs

        # Parameters for the launch argument node
        parameters = {}
        parameters['package'] = self.__package
        parameters['file'] = self.__launch_file
        parameters['install_dirs'] = self.__install_dirs

        # Turn launch arguments into parameters to provide to the launch client node
        for i, argument in enumerate(launch_arguments):
            parameters[f'argname{i}'] = argument[0]
            parameters[f'argval{i}'] = argument[1]

        super().__init__(
            package='launch_remote_manager',
            executable='launch_client',
            name='launch_client_' + f'{self.__uuid.int:x}',
            namespace=normalize_to_list_of_substitutions(['/', self.__machine]),
            parameters=[parameters],
            condition=condition,
            output=output
        )

