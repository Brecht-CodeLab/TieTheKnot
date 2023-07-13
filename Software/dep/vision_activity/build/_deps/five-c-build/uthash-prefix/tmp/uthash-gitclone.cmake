
if(NOT "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash-stamp/uthash-gitinfo.txt" IS_NEWER_THAN "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash-stamp/uthash-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash-stamp/uthash-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout "https://github.com/troydhanson/uthash.git" "uthash"
    WORKING_DIRECTORY "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/troydhanson/uthash.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout master --
  WORKING_DIRECTORY "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'master'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash-stamp/uthash-gitinfo.txt"
    "/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash-stamp/uthash-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/brecht/arcs2022/Projects/TieTheKnot/Software/dep/vision_activity/build/_deps/five-c-build/uthash-prefix/src/uthash-stamp/uthash-gitclone-lastrun.txt'")
endif()

