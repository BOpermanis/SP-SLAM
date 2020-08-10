cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
#   message(FATAL_ERROR "OpenCV not found.")
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
#set(OpenCV_LIBS /slamdoom/libs/opencv3)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
#find_package( G2O REQUIRED )
#find_package( PCL 1.7 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/g2oAddition
${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/CAPE
        ${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/grid_map_core/include
        /librealsense/include
#        ${PCL_INCLUDE_DIRS}
#${G2O_INCLUDE_DIRS}
)
#add_definitions( ${PCL_DEFINITIONS} )
#link_directories( ${PCL_LIBRARY_DIRS} )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
           src/System.cc
           src/Tracking.cc
           src/LocalMapping.cc
           src/LoopClosing.cc
           src/ORBextractor.cc
           src/ORBmatcher.cc
           src/FrameDrawer.cc
           src/Converter.cc
           src/MapPoint.cc
           src/KeyFrame.cc
           src/Map.cc
           src/MapDrawer.cc
           src/Optimizer.cc
           src/PnPsolver.cc
           src/Frame.cc
           src/KeyFrameDatabase.cc
           src/Sim3Solver.cc
           src/Initializer.cc
           src/Viewer.cc
           src/PointCloudMapping.cc
           src/Config.cc
           src/MapPlane.cc
           g2oAddition/VertexPlane.cc
           g2oAddition/EdgePlane.cc
           g2oAddition/EdgeParallelPlane.cc
           g2oAddition/EdgeVerticalPlane.cc
           g2oAddition/EdgeTwoParPlanes.cc
           g2oAddition/EdgeTwoParPlanes.h
           g2oAddition/EdgeTwoVerPlanes.cc
           g2oAddition/EdgeTwoVerPlanes.h
            CAPE/CAPE.cpp
        CAPE/CAPE.h
        CAPE/capewrap.cpp
        CAPE/CylinderSeg.cpp
        CAPE/CylinderSeg.h
        CAPE/Histogram.cpp
        CAPE/Histogram.h
        CAPE/Params.h
        CAPE/PlaneSeg.cpp
        CAPE/PlaneSeg.h
        Examples/slam_zero_run.cpp)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/grid_map_core/lib/libgrid_map_core.so
        #        ${PCL_LIBRARIES}
)

# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(SPSLAM
        Examples/RGB-D/SPSLAM.cc)
target_link_libraries(SPSLAM ${PROJECT_NAME})

add_executable(slam_zero_run
        Examples/slam_zero_run.cpp Examples/gridmap_example.cpp)
target_link_libraries(slam_zero_run ${PROJECT_NAME})

add_executable(cape_offline CAPE/run_cape_offline.cpp
        CAPE/CAPE.cpp
        CAPE/PlaneSeg.cpp
        CAPE/CylinderSeg.cpp
        CAPE/Histogram.cpp)

target_link_libraries(cape_offline ${OpenCV_LIBS} ${Eigen_LIBRARIES})

#add_executable(test_pcl_plane_extraction
#        CAPE/CAPE.cpp
#        CAPE/frame.cpp
#        CAPE/PlaneSeg.cpp
#        CAPE/CylinderSeg.cpp
#        CAPE/Histogram.cpp
#        CAPE/test_pcl_plane_extraction.cpp CAPE/cape_on_realsense.cpp)
#target_link_libraries(test_pcl_plane_extraction ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBRARIES})

#set(realsense2_LIBRARY /librealsense/build/CMakeFiles)
#set(realsense_INCLUDE_DIR /librealsense/include)
add_executable(run_realsense Examples/RGB-D/run_realsense.cpp)
#include_directories(include /librealsense/include)
target_link_libraries(run_realsense ${PROJECT_NAME})

target_link_libraries(run_realsense
        /librealsense/build/librealsense2.so
        /SP-SLAM/lib/libORB_SLAM2.so
        ${OpenCV_LIBS}
        ${Eigen_LIBRARIES}
        ${Pangolin_LIBRARIES})


add_executable(cape_on_realsense CAPE/cape_on_realsense.cpp)
#include_directories(include /librealsense/include)
#target_link_libraries(run_realsense /CAPE)
target_link_libraries(cape_on_realsense
        /SP-SLAM/lib/libORB_SLAM2.so
        /librealsense/build/librealsense2.so
        ${OpenCV_LIBS})

add_executable(gridmap_experiment Examples/gridmap_example.cpp)
target_link_libraries(gridmap_experiment
        /SP-SLAM/grid_map_core/lib/libgrid_map_core.so ${OpenCV_LIBS})
