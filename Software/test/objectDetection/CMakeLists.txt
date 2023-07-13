# OpenCV_INCLUDE_DIRS = opencv_highgui opencv_imgproc
cmake_minimum_required(VERSION 3.10.0)
project( ObjectDetection VERSION 0.0.1 LANGUAGES C CXX)
set_property(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR} PROPERTY EP_UPDATE_DISCONNECTED 1)

# include_directories( ${OpenCV_INCLUDE_DIRS} )
# include_directories($<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>)

add_library(${PROJECT_NAME} SHARED
    ropeSegmentation.cpp
)

find_package( OpenCV REQUIRED COMPONENTS highgui imgproc core )

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${${OpenCV_INCLUDE_DIRS}}>
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
)
add_executable( detection ropeSegmentation.cpp )
target_link_libraries( detection PRIVATE
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  # opencv_core 
  # opencv_video
  # opencv_imgproc
)

install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)