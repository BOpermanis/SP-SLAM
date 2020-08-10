cmake_minimum_required(VERSION 3.5.1)
project(grid_map_core)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -pg")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -pg")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Define Eigen addons.
include(cmake/${PROJECT_NAME}-extras.cmake)


set(EIGEN3_INCLUDE_DIR /eigen-3.3.7)
set(Eigen3_FIND_VERSION "${Eigen3_FIND_VERSION_MAJOR}.${Eigen3_FIND_VERSION_MINOR}.${Eigen3_FIND_VERSION_PATCH}")



#set(EIGEN_FUNCTORS_PLUGIN_PATH "eigen_plugins/FunctorsPlugin.hpp")
#if (EIGEN_FUNCTORS_PLUGIN)
#    if (NOT EIGEN_FUNCTORS_PLUGIN STREQUAL EIGEN_FUNCTORS_PLUGIN_PATH)
#        MESSAGE(FATAL_ERROR "EIGEN_FUNCTORS_PLUGIN already defined!")
#    endif ()
#else (EIGEN_FUNCTORS_PLUGIN)
#    add_definitions(-DEIGEN_FUNCTORS_PLUGIN=\"${EIGEN_FUNCTORS_PLUGIN_PATH}\")
#endif (EIGEN_FUNCTORS_PLUGIN)
#
#set(EIGEN_DENSEBASE_PLUGIN_PATH "eigen_plugins/DenseBasePlugin.hpp")
#if (EIGEN_DENSEBASE_PLUGIN)
#    if (NOT EIGEN_DENSEBASE_PLUGIN STREQUAL EIGEN_DENSEBASE_PLUGIN_PATH)
#        MESSAGE(FATAL_ERROR "EIGEN_DENSEBASE_PLUGIN already defined!")
#    endif ()
#else (EIGEN_DENSEBASE_PLUGIN)
#    add_definitions(-DEIGEN_DENSEBASE_PLUGIN=\"${EIGEN_DENSEBASE_PLUGIN_PATH}\")
#endif (EIGEN_DENSEBASE_PLUGIN)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    #   message(FATAL_ERROR "OpenCV not found.")
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR}
)

## Declare a cpp library
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED
   src/GridMap.cpp
   src/GridMapMath.cpp
   src/SubmapGeometry.cpp
   src/BufferRegion.cpp
   src/Polygon.cpp
   src/CubicInterpolation.cpp
   src/iterators/GridMapIterator.cpp
   src/iterators/SubmapIterator.cpp
   src/iterators/CircleIterator.cpp
   src/iterators/EllipseIterator.cpp
   src/iterators/SpiralIterator.cpp
   src/iterators/PolygonIterator.cpp
   src/iterators/LineIterator.cpp
   src/iterators/SlidingWindowIterator.cpp
#        src/GridMapCvProcessing.cpp
#        src/InpaintFilter.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
