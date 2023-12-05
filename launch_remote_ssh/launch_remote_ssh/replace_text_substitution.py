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

from typing import List
from typing import Text
from typing import Iterable
from typing import Sequence

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.frontend import expose_substitution
from launch.frontend import Entity
from launch.frontend import Parser
from launch.substitution import Substitution
from launch.launch_context import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

@expose_substitution('replace-text')
class ReplaceTextSubstitution(Substitution):
    """Substitution that replaces text1 with text2 in an input string."""
    def __init__(
        self,
        input_string: SomeSubstitutionsType,
        text1: SomeSubstitutionsType,
        text2: SomeSubstitutionsType
    ) -> None:
        """Create a ReplaceTextSubstitution."""
        super().__init__()

        self.__input_string = normalize_to_list_of_substitutions(input_string)
        self.__text1 = normalize_to_list_of_substitutions(text1)
        self.__text2 = normalize_to_list_of_substitutions(text2)

    @property
    def input_string(self) -> List[Substitution]:
        """Getter for text1."""
        return self.__input_string

    @property
    def text1(self) -> List[Substitution]:
        """Getter for text1."""
        return self.__text1
    
    @property
    def text2(self) -> List[Substitution]:
        """Getter for text1."""
        return self.__text2

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f'Replace {"".join([sub.describe() for sub in self.text1])}' \
               f' with {"".join([sub.describe() for sub in self.text2])}' \
               f' in {"".join([sub.describe() for sub in self.input_string])}'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by returning the string itself."""

        performed_input_string = perform_substitutions(context, self.input_string)
        performed_text1 = perform_substitutions(context, self.text1)
        performed_text2 = perform_substitutions(context, self.text2)
        return performed_input_string.replace(performed_text1, performed_text2)
    
    @classmethod
    def parse(self, data: Sequence[SomeSubstitutionsType]):
        if len(data) != 3:
            raise ValueError('ReplaceTextSubstitution expects 3 arguments')
        return self, {'input_string': data[0], 'text1': data[1], 'text2': data[2]}