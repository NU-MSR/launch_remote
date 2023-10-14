find_package(PythonInterp REQUIRED)

message(${ERROR_FILE})

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} ${SCRIPT_PATH}
  RESULT_VARIBALE EXIT_CODE
  ERROR_FILE ${ERROR_FILE}
)

foreach(FILE ${FILES})
  message(${FILE})
endforeach()

foreach(FILE ${DIRECTORIES})
  message(${FILE})
endforeach()


if (NOT EXIT_CODE EQUAL 0)
  if (EXISTS ${ERROR_FILE})
    file(READ ${ERROR_FILE} ERROR_MESSAGE)
  endif()
  message(FATAL_ERROR ${ERROR_MESSAGE})
endif()