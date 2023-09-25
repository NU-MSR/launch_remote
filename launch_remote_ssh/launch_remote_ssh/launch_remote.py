from typing import Optional
from typing import Iterable
from typing import Tuple

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.condition import Condition

from .replace_text_substitution import ReplaceTextSubstitution
from .execute_process_remote import ExecuteProcessRemote




class LaunchRemote(ExecuteProcessRemote):
    def __init__(
        self,
        user : SomeSubstitutionsType,
        machine : SomeSubstitutionsType,
        package : SomeSubstitutionsType,
        launch_file : SomeSubstitutionsType,
        launch_arguments: Optional[
            Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]
        ] = None,
        port : SomeSubstitutionsType = None,
        source_paths : Optional[Iterable[SomeSubstitutionsType]] = None,
        condition : Optional[Condition] = None
    ):
        self.__package = package
        self.__launch_file = launch_file

        # Generate run node command
        command = [
            'ros2 launch ',
            self.__package,
            ' ',
            self.__launch_file,
        ]

        if launch_arguments is not None:
            for argument in launch_arguments:
                command += [
                    ' ',
                    ReplaceTextSubstitution([argument[0]], ' ', '\ '), # escape spaces
                    ':=',
                    ReplaceTextSubstitution([argument[1]], ' ', '\ '), # escape spaces
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