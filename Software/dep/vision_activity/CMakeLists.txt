cmake_minimum_required(VERSION 3.10.0)
project(vision_activity VERSION 0.0.1 LANGUAGES C CXX)

message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}) ##This is to print the status during the cmake

set(CMAKE_BUILD_TYPE Debug)

############
## Option ##
############
option(STAND_ALONE "Compile this package outside a higher lever repository" ON)
option(BUILD_OFFLINE "Build offline" OFF)

if (${BUILD_OFFLINE})
    set_property(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR}
                PROPERTY EP_UPDATE_DISCONNECTED 1)
endif()


###########
## Build ##
###########

# FIVE C
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
    src/vision_activity.cpp
)

add_dependencies(${PROJECT_NAME} five_c)

#Opencv libraries
find_package( OpenCV REQUIRED COMPONENTS highgui imgproc core )
# include_directories( ${OpenCV_INCLUDE_DIRS} )

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${FIVE_C_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${${OpenCV_INCLUDE_DIRS}}>
)

target_link_directories(${PROJECT_NAME}
    PUBLIC
        # ${COORDINATON_LIBRARIES_BUILD_DIR}
        # ${IIWA_AACAL_BUILD_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    five_c    
    ${OpenCV_LIBS}
)

if(STAND_ALONE)
    add_executable(vision_activity_exe src/vision_activity.cpp)
    target_link_libraries(vision_activity_exe PRIVATE
        ${PROJECT_NAME}
        five_c
        ${OpenCV_LIBS}
        )    
endif()


#############
## Install ##
#############
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
