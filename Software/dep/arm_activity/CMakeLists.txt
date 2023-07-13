cmake_minimum_required(VERSION 3.10.0)
project(iiwa-aacal VERSION 0.0.1 LANGUAGES C CXX)

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

#KDL and KDL Models
set(BUILD_MODELS ON CACHE BOOL "Build kdl models")
add_subdirectory(external/orocos_kdl)
include_directories(external/orocos_kdl/models)
include_directories(external/orocos_kdl/src)




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
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Orocos_KDL REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})


add_library(${PROJECT_NAME} SHARED
    src/iiwa_client.cpp
    src/iiwa_interface.cpp
    src/iiwa_activity.cpp
    # src/iiwa_controller.cpp
)
add_dependencies(${PROJECT_NAME} five_c kuka_fri_sdk orocos-kdl orocos-kdl-models)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${FIVE_C_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${FRISDK_INCLUDE}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    five_c
    ${FRISDK_LIB}/libFRIClient.so
    Eigen3::Eigen
    ${orocos_kdl_LIBRARIES}
    orocos-kdl 
    orocos-kdl-models
)

if(STAND_ALONE)
    add_executable(iiwa_demo src/Table_perception.cpp)
    target_link_libraries(iiwa_demo PUBLIC
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
