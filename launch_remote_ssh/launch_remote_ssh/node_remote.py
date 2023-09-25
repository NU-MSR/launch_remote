from typing import Optional
from typing import Iterable

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.condition import Condition
from launch.utilities import normalize_to_list_of_substitutions

from .execute_process_remote import ExecuteProcessRemote

class NodeRemote(ExecuteProcessRemote):
    def __init__(
        self,
        user : SomeSubstitutionsType,
        machine : SomeSubstitutionsType,
        package : SomeSubstitutionsType,
        executable : SomeSubstitutionsType,
        # TODO - arguments, parameters
        port : SomeSubstitutionsType = None,
        source_paths : Optional[Iterable[SomeSubstitutionsType]] = None,
        condition : Optional[Condition] = None
    ):
        self.__package = package
        self.__executable = executable

        # Generate run node command
        command = [
            'ros2 run ',
            self.__package,
            ' ',
            self.__executable,
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