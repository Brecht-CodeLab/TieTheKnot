OPTION(BUILD_MODELS "Build models for some well known robots" FALSE)

IF(BUILD_MODELS)
  
  ADD_LIBRARY(orocos-kdl-models SHARED puma560.cpp kukaLWR_DHnew.cpp kukaIIWA14.cpp) 
  INCLUDE_DIRECTORIES(${PROJ_SOURCE_DIR}/src ${PROJ_BINARY_DIR}/src)
  SET_TARGET_PROPERTIES( orocos-kdl-models PROPERTIES
    SOVERSION "${KDL_VERSION_MAJOR}.${KDL_VERSION_MINOR}"
    VERSION "${KDL_VERSION}"
    COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD} ${KDL_CFLAGS}"
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}"
    PUBLIC_HEADER models.hpp)
  TARGET_LINK_LIBRARIES(orocos-kdl-models orocos-kdl)

  export(TARGETS orocos-kdl-models APPEND
  FILE "${PROJECT_BINARY_DIR}/OrocosKDLTargets.cmake")

  INSTALL( TARGETS orocos-kdl-models
      EXPORT OrocosKDLTargets
      ARCHIVE DESTINATION lib${LIB_SUFFIX}
      LIBRARY DESTINATION lib${LIB_SUFFIX}
      PUBLIC_HEADER DESTINATION include/kdl
  )

ENDIF(BUILD_MODELS)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(BUILD_MODELS_DEMO "Build demo for some of the models" OFF "BUILD_MODELS" OFF)  
IF(BUILD_MODELS_DEMO)
  ADD_EXECUTABLE(p560test puma560test.cpp)
  TARGET_LINK_LIBRARIES(p560test orocos-kdl-models)
  SET_TARGET_PROPERTIES( p560test PROPERTIES
    COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD} ${KDL_CFLAGS}")

  ADD_EXECUTABLE(kukaLWRtestDHnew kukaLWRtestDHnew.cpp)
  TARGET_LINK_LIBRARIES(kukaLWRtestDHnew orocos-kdl-models)
  SET_TARGET_PROPERTIES( kukaLWRtestDHnew PROPERTIES
    COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD} ${KDL_CFLAGS}")

  ADD_EXECUTABLE(kukaLWRtestHCG kukaLWRtestHCG.cpp)
  TARGET_LINK_LIBRARIES(kukaLWRtestHCG orocos-kdl-models)
  SET_TARGET_PROPERTIES( kukaLWRtestHCG PROPERTIES
    COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD} ${KDL_CFLAGS}")

ENDIF(BUILD_MODELS_DEMO)