cmake_minimum_required(VERSION 3.10.0)
project(petrinet_activity VERSION 0.0.1 LANGUAGES C CXX)
set_property(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR} PROPERTY EP_UPDATE_DISCONNECTED 1)

message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}) 

set(CMAKE_BUILD_TYPE Debug)

############
## Option ##
############
option(STAND_ALONE "Compile this package outside a higher lever repository" OFF)
option(BUILD_OFFLINE "Build offline" OFF)

###########
## Build ##
###########
## KUKA SDK specific
# set(KUKA_FRI_SDK ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/)
# set(FRISDK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/include)
# set(FRISDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/lib)
# add_custom_target(kuka_fri_sdk COMMAND make clean && make
                #    WORKING_DIRECTORY ${KUKA_FRI_SDK}/build/GNUMake
# )

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

# Coordination libraries
FetchContent_Declare(coordination-libraries
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/coordination_libraries.git
    GIT_TAG repo/organization
    TIMEOUT 5
    INSTALL_COMMAND ""
)
FetchContent_MakeAvailable(coordination-libraries)

FetchContent_GetProperties(coordination-libraries
    SOURCE_DIR COORDINATON_LIBRARIES_SOURCE_DIR
    BINARY_DIR COORDINATON_LIBRARIES_BUILD_DIR
)
set(COORDINATON_LIBRARIES_INCLUDE_DIR ${COORDINATON_LIBRARIES_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED
    src/petrinet_activity.cpp
    # src/iiwa_client.cpp
    # src/iiwa_interface.cpp
    # src/iiwa_activity.cpp
)
# add_dependencies(${PROJECT_NAME} kuka_fri_sdk)
add_dependencies(${PROJECT_NAME} five_c coordination_libraries)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${FIVE_C_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${COORDINATON_LIBRARIES_INCLUDE_DIR}>
)

target_link_directories(${PROJECT_NAME}
    PUBLIC
        ${COORDINATON_LIBRARIES_BUILD_DIR}
)


target_link_libraries(${PROJECT_NAME} PRIVATE
    five_c
    coordination_libraries
)

if(STAND_ALONE)
    add_executable(petrinet_activity petrinet_activity.cpp)
    target_link_libraries(petrinet_activity PRIVATE
        ${PROJECT_NAME}
        pthread
        five_c
        coordination_libraries
        )    
endif()


#############
## Install ##
#############
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
