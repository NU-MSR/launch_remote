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

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.condition import Condition
from launch.utilities import ensure_argument_type
from launch_ros.parameters_type import SomeParameters, ParametersDict
from launch_ros.remap_rule_type import SomeRemapRules
from launch_ros.utilities import normalize_remap_rules, normalize_parameters
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import Parameter as ParameterDescription

from .execute_process_remote_ssh import ExecuteProcessRemoteSSH

class NodeRemoteSSH(ExecuteProcessRemoteSSH):
    def __init__(
        self, *,
        user : SomeSubstitutionsType,
        machine : SomeSubstitutionsType,
        package : SomeSubstitutionsType,
        executable : SomeSubstitutionsType,
        # TODO(anyone) anything between these lines is not well tested
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        # TODO(anyone) anything between these lines is not well tested
        port : Optional[SomeSubstitutionsType] = None,
        source_paths : Optional[Iterable[SomeSubstitutionsType]] = None,
        condition : Optional[Condition] = None
    ):
        self.__package = package
        self.__node_executable = executable
        self.__node_name = name
        self.__node_namespace = namespace
        self.__arguments = arguments
        self.__ros_arguments = ros_arguments
        self.__remappings = [] if remappings is None else list(normalize_remap_rules(remappings))
        if parameters is not None:
            ensure_argument_type(parameters, (list), 'parameters', 'Node')
            normalized_params = normalize_parameters(parameters)
        self.__parameters = [] if parameters is None else normalized_params

        # Build argument list
        argument_list = []
        for argument in self.__arguments:
            argument_list.append(' ')
            argument_list.append(argument)

        # Build ros argument list
        ros_argument_list = []
        for ros_argument in self.__ros_arguments:
            ros_argument_list.append(' ')
            ros_argument_list.append(ros_argument)

        # Remap node name
        if self.__node_name is not None:
            ros_argument_list.append(' -r __node:=')
            ros_argument_list.append(self.__node_name)

        # Set namespace
        if self.__node_namespace is not None:
            ros_argument_list.append(' -r __ns:=/')
            ros_argument_list.append(self.__node_namespace)

        # Remappings
        for remapping in self.__remappings:
            ros_argument_list.append(' -r ')
            ros_argument_list.append(remapping[0])
            ros_argument_list.append(':=')
            ros_argument_list.append(remapping[1])

        # Parameters
        for parameter in self.__parameters:
            if isinstance(parameter, ParameterFile):
                ros_argument_list.append(' --params-file ')
                ros_argument_list.append(parameter.param_file)
            elif isinstance(parameter, ParametersDict):
                for param_name, param_value in parameter.items():
                    ros_argument_list.append(' -p ')
                    ros_argument_list.append(param_name)
                    ros_argument_list.append(':=')
                    ros_argument_list.append(param_value)
            elif isinstance(parameter, ParameterDescription):
                ros_argument_list.append(' -p ')
                ros_argument_list.append(parameter.name)
                ros_argument_list.append(':=')
                ros_argument_list.append(parameter.value)

        # Generate run node command
        command = [
            'ros2 run ',
            self.__package,
            ' ',
            self.__node_executable,
        ]
        command += argument_list
        command += [
            ' --ros-args '
        ]
        command += ros_argument_list

        # ExecuteRemoteProcess
        super().__init__(
            user=user,
            machine=machine,
            command=command,
            port=port,
            source_paths=source_paths,
            condition=condition,
        )