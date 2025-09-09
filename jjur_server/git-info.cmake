# in case Git is not available, we default to "unknown"
set(GIT_HASH "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%h
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_SOURCE_DIR}
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} branch --show-current
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_SOURCE_DIR}
    )
else()
    message(STATUS "Git not found")
endif()
message(STATUS "Git hash is ${GIT_HASH}")
message(STATUS "Git branch is ${GIT_BRANCH}")
# generate file version.hpp based on version.hpp.in
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
  ${TARGET_DIR}/generated/version.hpp
  @ONLY
  )
