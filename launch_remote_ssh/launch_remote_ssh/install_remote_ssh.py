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

from ament_index_python.packages import get_package_prefix
import subprocess

# Typically use this to copy an entire install space
def copy_install_space(
    user,
    machine,
    local_install_space,
    remote_install_space,
    remove_preexisting=False
):
    if remove_preexisting:
        _remove_dir(user, machine, remote_install_space)

    _copy_dir(user, machine, local_install_space, remote_install_space)

# Probably don't want to use this unless you intentionally want to only use a single
# package, and that package's rosdeps will be fulfilled on the remote system
def copy_single_package_install(
    user,
    machine,
    package_name,
    remote_install_space,
    remove_preexisting=False
):
    local_package_install = get_package_prefix(package_name) + '/'
    remote_package_install = remote_install_space + '/' + package_name + '/'

    if remove_preexisting:
        _remove_dir(user, machine, remote_package_install)

    # Copy package to remote install directory
    _copy_dir(user, machine, local_package_install, remote_package_install)

def _copy_dir(user, machine, local_dir, remote_dir):
    if user == '' or machine == '':
        raise Exception("'user' and 'machine' arguments must be provided.")
    
    user_machine = user + '@' + machine

    # Create remote directory on machine
    subprocess.run(
        ['ssh ' + user_machine + ' "mkdir -p ' + remote_dir + '"'],
        shell=True,
    )

    # Copy files to remote directory
    subprocess.run(
        ['rsync -r ' + local_dir + ' ' + user_machine + ':' + remote_dir],
        shell=True
    )

def _remove_dir(user, machine, remote_dir):
    if user == '' or machine == '' or remote_dir == '':
        raise Exception("'user', 'machine', and 'remote_dir' arguments must be provided.")
    
    user_machine = user + '@' + machine

    # Remove remote directory from machine
    subprocess.run(
        ['ssh ' + user_machine + ' "rm -rf ' + remote_dir + '"'],
        shell=True
    )
