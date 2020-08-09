cmake_minimum_required(VERSION 3.5.1)
project(grid_map_core)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Define Eigen addons.
include(cmake/${PROJECT_NAME}-extras.cmake)

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)
# Solution to find Eigen3 with Saucy.
#find_package(Eigen3 QUIET)
#if(NOT EIGEN3_FOUND)
#  find_package(PkgConfig REQUIRED)
#  pkg_check_modules(EIGEN3 REQUIRED eigen3)
#  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
#endif()
set(EIGEN3_INCLUDE_DIR /eigen-3.3.7)
set(Eigen3_FIND_VERSION "${Eigen3_FIND_VERSION_MAJOR}.${Eigen3_FIND_VERSION_MINOR}.${Eigen3_FIND_VERSION_PATCH}")

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
        src/GridMapCvProcessing.cpp
        src/InpaintFilter.cpp
)

#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
