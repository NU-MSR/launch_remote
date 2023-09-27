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

