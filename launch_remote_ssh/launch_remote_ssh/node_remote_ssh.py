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
from typing import Mapping
from typing import List
from typing import Union
from typing import Sequence
from typing import cast

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.frontend import expose_action
from launch.frontend import Entity
from launch.frontend import Parser
from launch.condition import Condition
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities.type_utils import SomeValueType
from launch.utilities.type_utils import ScalarValueType
from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import Parameter as ParameterDescription
from launch_ros.parameter_descriptions import ParameterValue as ParameterValueDescription

from .execute_process_remote_ssh import ExecuteProcessRemoteSSH
from .replace_text_substitution import ReplaceTextSubstitution

@expose_action('node_remote_ssh')
class NodeRemoteSSH(ExecuteProcessRemoteSSH):
    def __init__(
        self, *,
        user: SomeSubstitutionsType,
        machine: SomeSubstitutionsType,
        executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,  # If any of these are yaml files they must be
                                                      # present on the remote system, and this must
                                                      # be the path on the remote system
        remappings: Optional[SomeRemapRules] = None,
        ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        port: Optional[SomeSubstitutionsType] = None,
        source_paths: Optional[Iterable[SomeSubstitutionsType]] = None,
        condition: Optional[Condition] = None
    ):
        self.__package = [] if package is None else normalize_to_list_of_substitutions(package)
        self.__node_executable = normalize_to_list_of_substitutions(executable)
        self.__node_name = name
        self.__node_namespace = namespace
        self.__arguments = [] if arguments is None else arguments
        self.__ros_arguments = [] if ros_arguments is None else ros_arguments
        self.__remappings = [] if remappings is None else remappings
        if parameters is not None:
            ensure_argument_type(parameters, (list), 'parameters', 'Node')
        self.__parameters = [] if parameters is None else parameters

        # Build argument list
        argument_list = []
        for argument in self.__arguments:
            argument_list.append(' ')
            argument_list += normalize_to_list_of_substitutions(argument)

        # Build ros argument list
        ros_argument_list = []
        for ros_argument in self.__ros_arguments:
            ros_argument_list.append(' ')
            ros_argument_list += normalize_to_list_of_substitutions(ros_argument)

        # Remap node name
        if self.__node_name is not None:
            ros_argument_list.append(' -r __node:=')
            ros_argument_list += normalize_to_list_of_substitutions(self.__node_name)

        # Set namespace
        if self.__node_namespace is not None:
            ros_argument_list.append(' -r __ns:=/')
            ros_argument_list += normalize_to_list_of_substitutions(self.__node_namespace)

        # Remappings
        for remapping in self.__remappings:
            ros_argument_list.append(' -r ')
            ros_argument_list += normalize_to_list_of_substitutions(remapping[0])
            ros_argument_list.append(':=')
            ros_argument_list += normalize_to_list_of_substitutions(remapping[1])

        # Parameters
        # Logic modified from launch_ros.utilities.normalize_parameters
        for parameter in self.__parameters:
            if isinstance(parameter, Mapping):
                ros_argument_list += _mapping_to_substitution_list(parameter)
            elif isinstance(parameter, ParameterDescription):
                ros_argument_list += _parameter_description_to_substitution_list(parameter)
            elif isinstance(parameter, ParameterFile):
                ros_argument_list += _parameter_file_to_substitution_list(parameter)
            else:
                # It's a path
                ros_argument_list += _parameter_file_to_substitution_list(ParameterFile(parameter))

        # Generate run node command
        command = [
            'ros2 run ',
        ]
        command += self.__package
        command += [
            ' ',
        ]
        command += self.__node_executable,
        command += argument_list
        command += [
            ' --ros-args ',
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

    @classmethod
    def parse(
        self,
        entity: Entity,
        parser: Parser
    ):
        # Adapted from parse method here:
        # https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/node.py
        _, kwargs = super().parse(entity, parser, ignore=['cmd'])
        args = entity.get_attr('args', optional=True)
        if args is not None:
            kwargs['arguments'] = super()._parse_cmdline(args, parser)
        ros_args = entity.get_attr('ros_args', optional=True)
        if ros_args is not None:
            kwargs['ros_arguments'] = super()._parse_cmdline(ros_args, parser)
        name = entity.get_attr('name', optional=True)
        if name is not None:
            kwargs['name'] = parser.parse_substitution(name)
        package = entity.get_attr('pkg', optional=True)
        if package is not None:
            kwargs['package'] = parser.parse_substitution(package)
        kwargs['executable'] = parser.parse_substitution(entity.get_attr('exec'))
        namespace = entity.get_attr('namespace', optional=True)
        if namespace is not None:
            kwargs['namespace'] = parser.parse_substitution(namespace)
        remappings = entity.get_attr('remap', data_type=List[Entity], optional=True)
        if remappings is not None:
            kwargs['remappings'] = [
                (
                    parser.parse_substitution(remap.get_attr('from')),
                    parser.parse_substitution(remap.get_attr('to'))
                ) for remap in remappings
            ]
            for remap in remappings:
                remap.assert_entity_completely_parsed()
        parameters = entity.get_attr('param', data_type=List[Entity], optional=True)
        if parameters is not None:
            kwargs['parameters'] = Node().parse_nested_parameters(parameters, parser)

        return self, kwargs




def _mapping_to_substitution_list(
    mapping: Mapping,
    _prefix: Optional[Sequence[Substitution]] = None
) -> List[Substitution]:
    out: List[SomeSubstitutionsType] = []

    for name, value in mapping.items():
        name = normalize_to_list_of_substitutions(name)
        if _prefix:
            combined = list(_prefix)
            combined.append(TextSubstitution(text='.'))
            combined.extend(name)
            name = combined
        
        if isinstance(value, Mapping):
            # Flatten recursive dictionaries
            out += _mapping_to_substitution_list(value, _prefix=name)
        elif isinstance(value, ParameterValueDescription):
            out += _name_and_value_to_substitution_list(name, value.value)
        elif isinstance(value, Union[ScalarValueType, Substitution, Sequence]):
            out += _name_and_value_to_substitution_list(name, value)
        elif isinstance(value, bytes):
            out += _name_and_value_to_substitution_list(name, str(value))
        else:
            raise TypeError('Unexpected type for parameter value {}'.format(repr(value)))

    return normalize_to_list_of_substitutions(out)


def _parameter_description_to_substitution_list(param: ParameterDescription) -> List[Substitution]:
    return _name_and_value_to_substitution_list(param.name, param.value)

def _name_and_value_to_substitution_list(
    name: List[Substitution],
    value: SomeValueType
) -> List[Substitution]:
    out: List[SomeSubstitutionsType] = []

    out.append(' -p ')
    out += name
    out.append(':=')
    if isinstance(value, Union[ScalarValueType, Substitution]):
        out += _scalar_value_to_substitution_list(value)
    elif isinstance(value, Sequence):
        out += _sequence_value_to_substitution_list(value)

    return normalize_to_list_of_substitutions(out)

def _scalar_value_to_substitution_list(value: Union[ScalarValueType, Substitution]) -> List[Substitution]:
    out: List[SomeSubstitutionsType] = []
    if isinstance(value, Union[int, float, bool]):
        out.append(str(value))
    elif isinstance(value, Union[str, Substitution]):
        out.append(ReplaceTextSubstitution(value, ' ', '\ '))  # escape spaces
    return normalize_to_list_of_substitutions(out)

def _sequence_value_to_substitution_list(value: Sequence) -> List[Substitution]:
    out: List[SomeSubstitutionsType] = []
   
    has_types = set()
    for subvalue in value:
        allowed_subtypes = (float, int, str, bool) + SomeSubstitutionsType_types_tuple
        ensure_argument_type(subvalue, allowed_subtypes, 'subvalue')

        if isinstance(subvalue, Substitution):
            has_types.add(Substitution)
        elif isinstance(subvalue, str):
            has_types.add(str)
        elif isinstance(subvalue, bool):
            has_types.add(bool)
        elif isinstance(subvalue, int):
            has_types.add(int)
        elif isinstance(subvalue, float):
            has_types.add(float)
        elif isinstance(subvalue, Sequence):
            has_types.add(tuple)
        else:
            raise RuntimeError('Failed to handle type {}'.format(repr(subvalue)))
    
    start_str = '['
    end_str = ']'

    if {int} == has_types:
        # everything is an integer
        make_mypy_happy_int = cast(List[int], value)
        out.append(start_str)
        for val in make_mypy_happy_int:
            out += _scalar_value_to_substitution_list(val)
            out.append(',')
        out[-1] = end_str
    elif has_types in ({float}, {int, float}):
        # all were floats or ints, so return floats
        make_mypy_happy_float = cast(List[Union[int, float]], value)
        out.append(start_str)
        for val in make_mypy_happy_float:
            out += _scalar_value_to_substitution_list(float(val))
            out.append(',')
        out[-1] = end_str
    elif Substitution in has_types and has_types.issubset({str, Substitution}):
        # make a list of substitutions forming a single string
        for val in value:
            out += _scalar_value_to_substitution_list(val)
    elif {bool} == has_types:
        # all were bools
        out.append(start_str)
        for val in value:
            out += _scalar_value_to_substitution_list(val)
            out.append(',')
        out[-1] = end_str
    else:
        # Should evaluate to a list of strings
        # Normalize to a list of lists of substitutions
        out.append(start_str)
        out += _recursive_string_list_to_substitution_list(value)
        out[-1] = end_str

    return normalize_to_list_of_substitutions(out)

def _recursive_string_list_to_substitution_list(value: Sequence) -> List[Substitution]:
    out: List[SomeSubstitutionsType] = []
    for val in value:
        if isinstance(val, str):
            out += _scalar_value_to_substitution_list(val)
            out.append(',')
        elif isinstance(val, Sequence):
            out += _recursive_string_list_to_substitution_list(val)
        else:
            out += _scalar_value_to_substitution_list(val)
            out.append(',')
    return normalize_to_list_of_substitutions(out)

def _parameter_file_to_substitution_list(param_file: ParameterFile) -> List[Substitution]:
    out: List[SomeSubstitutionsType] = []
    out.append(' --params-file ')
    out.append(ReplaceTextSubstitution(  # escape spaces in file path
            param_file.param_file,
            ' ',
            '\ '
        )
    )
    return normalize_to_list_of_substitutions(out)