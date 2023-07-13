cmake_minimum_required(VERSION 3.10.0)
project(align_activity VERSION 0.0.1 LANGUAGES C CXX)

message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}) 

set(CMAKE_BUILD_TYPE Debug)

############
## Option ##
############
option(STAND_ALONE "Compile this package outside a higher lever repository" ON)

###########
## Build ##
###########
## KUKA SDK specific
set(KUKA_FRI_SDK ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/)
set(FRISDK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/include)
set(FRISDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/lib)
add_custom_target(kuka_fri_sdk COMMAND make clean && make
                   WORKING_DIRECTORY ${KUKA_FRI_SDK}/build/GNUMake
)


# ROB-KUL libraries
include(FetchContent)
FetchContent_Declare(five-c
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/algorithm_and_activity_component_architecture.git
    GIT_TAG repo/organization
    TIMEOUT 5
    INSTALL_COMMAND ""
)
FetchContent_MakeAvailable(five-c)
FetchContent_GetProperties(five-c
    SOURCE_DIR FIVE_C_SOURCE_DIR
    BINARY_DIR FIVE_C_BUILD_DIR
)
set(FIVE_C_INCLUDE_DIR ${FIVE_C_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED
    ../../../../../Software/iiwa-5C/src/iiwa_client.cpp
    ../../../../../Software/iiwa-5C/src/iiwa_interface.cpp
    ../../../../../Software/iiwa-5C/src/iiwa_activity.cpp
)
add_dependencies(${PROJECT_NAME} five_c kuka_fri_sdk)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${FIVE_C_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${FRISDK_INCLUDE}>
)


target_link_libraries(${PROJECT_NAME} PRIVATE
    five_c
    ${FRISDK_LIB}/libFRIClient.so
)

if(STAND_ALONE)
    add_executable(align_activity_exe src/align_activity.cpp)
    target_link_libraries(align_activity_exe PRIVATE
        ${PROJECT_NAME}
        pthread
        five_c
        ${FRISDK_LIB}/libFRIClient.so
        )    
endif()





#############
## Install ##
#############
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
