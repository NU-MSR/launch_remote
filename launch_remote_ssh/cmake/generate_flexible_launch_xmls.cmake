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

# TODO(ngmor) document

function(generate_flexible_launch_xmls)
  cmake_parse_arguments(ARG
    ""
    "DESTINATION;PACKAGE"
    "FILES;DIRECTORIES"
  ${ARGN}
  )
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "generate_flexible_launch_xmls() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}"
    )
  endif()
  if(NOT ARG_DESTINATION)
    message(FATAL_ERROR
      "generate_flexible_launch_xmls() must be invoked with the DESTINATION argument"
    )
  endif()
  if(NOT ARG_PACKAGE)
    message(FATAL_ERROR
      "generate_flexible_launch_xmls() must be invoked with the PACKAGE argument"
    )
  endif()
  if(NOT ARG_FILES AND NOT ARG_DIRECTORIES)
    message(FATAL_ERROR
      "No files or directories provided to generate_flexible_launch_xmls() for generation."
    )
  endif()

  # TODO make sure to install the core launch files

  if(NOT ${PythonInterp_FOUND})
    find_package(PythonInterp REQUIRED)
  endif()
endfunction()