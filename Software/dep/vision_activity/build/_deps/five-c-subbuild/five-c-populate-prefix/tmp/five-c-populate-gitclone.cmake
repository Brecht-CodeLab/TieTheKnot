
if(NOT "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-subbuild/five-c-populate-prefix/src/five-c-populate-stamp/five-c-populate-gitinfo.txt" IS_NEWER_THAN "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-subbuild/five-c-populate-prefix/src/five-c-populate-stamp/five-c-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-subbuild/five-c-populate-prefix/src/five-c-populate-stamp/five-c-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout "git@gitlab.kuleuven.be:u0144428/algorithm_and_activity_component_architecture.git" "five-c-src"
    WORKING_DIRECTORY "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'git@gitlab.kuleuven.be:u0144428/algorithm_and_activity_component_architecture.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout repo/organization --
  WORKING_DIRECTORY "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'repo/organization'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-subbuild/five-c-populate-prefix/src/five-c-populate-stamp/five-c-populate-gitinfo.txt"
    "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-subbuild/five-c-populate-prefix/src/five-c-populate-stamp/five-c-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-subbuild/five-c-populate-prefix/src/five-c-populate-stamp/five-c-populate-gitclone-lastrun.txt'")
endif()

