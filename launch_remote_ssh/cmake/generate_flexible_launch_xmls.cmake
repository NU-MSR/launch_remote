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

  # TODO - make sure to install the core launch files

  find_package(PythonInterp REQUIRED)

  # Compile arguments for script
  set(FILES_ARG " -f ")
  foreach(FILE ${ARG_FILES})
    set(FILES_ARG "${FILES_ARG} ${FILE}")
  endforeach()
  set(DIRS_ARG " -d ")
  foreach(DIR ${ARG_DIRECTORIES})
    set(DIRS_ARG "${DIRS_ARG} ${DIR}")
  endforeach()
  
  set(ERROR_FILE 
    ${CMAKE_CURRENT_BINARY_DIR}/generate_flexible_launch_xmls_error.log
  )

  # Run the script to generate the flexible launch files at install time
  install(CODE "
    # Run processing/install script
    execute_process(
      COMMAND
        ${PYTHON_EXECUTABLE}
        ${launch_remote_ssh_DIR}/../scripts/generate_flexible_launch_xmls.py
        ${ARG_PACKAGE}
        ${CMAKE_INSTALL_PREFIX}/${ARG_DESTINATION}
        ${FILES_ARG}
        ${DIRS_ARG}
      RESULT_VARIABLE SCRIPT_RESULT
      ERROR_FILE ${ERROR_FILE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # Read errors from error file if script did not complete successfully
    if (NOT SCRIPT_RESULT EQUAL 0)
      if(EXISTS ${ERROR_FILE})
        file(READ ${ERROR_FILE} ERROR_MSG)
      endif()
      message(FATAL_ERROR \"Generating flexible launch files failed.\n\${ERROR_MSG}\")
    endif()
  ")

endfunction()