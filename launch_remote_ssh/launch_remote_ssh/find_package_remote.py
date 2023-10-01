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

from typing import Sequence

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.frontend import expose_substitution
from launch.substitutions import PathJoinSubstitution
from launch.utilities import normalize_to_list_of_substitutions

@expose_substitution('find-package-prefix-remote')
class FindPackagePrefixRemote(PathJoinSubstitution):
    def __init__(
        self,
        remote_install_space: SomeSubstitutionsType,
        package: SomeSubstitutionsType,
    ):
        path = []
        path.extend(normalize_to_list_of_substitutions(remote_install_space))
        path.extend(normalize_to_list_of_substitutions(package))
        super().__init__(path)

    @classmethod
    def parse(self, data: Sequence[SomeSubstitutionsType]):
        return self, _parse_find_package_remote(data)

@expose_substitution('find-package-share-remote')
class FindPackageShareRemote(PathJoinSubstitution):
    def __init__(
        self,
        remote_install_space: SomeSubstitutionsType,
        package: SomeSubstitutionsType,
    ):
        path = [FindPackagePrefixRemote(remote_install_space, package), 'share']
        path.extend(normalize_to_list_of_substitutions(package))
        super().__init__(path)

    @classmethod
    def parse(self, data: Sequence[SomeSubstitutionsType]):
        return self, _parse_find_package_remote(data)

def _parse_find_package_remote(data: Sequence[SomeSubstitutionsType]):
    if len(data) != 2:
        raise ValueError('find-package-remote substitutions expect 2 arguments')
    kwargs = {
        'remote_install_space': data[0],
        'package': data[1]
    }

    return kwargs