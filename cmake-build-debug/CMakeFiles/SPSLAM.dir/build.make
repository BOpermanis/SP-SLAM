# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/slam_data/clion-2019.3.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/slam_data/clion-2019.3.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /SP-SLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /SP-SLAM/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/SPSLAM.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SPSLAM.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SPSLAM.dir/flags.make

CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.o: CMakeFiles/SPSLAM.dir/flags.make
CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.o: ../Examples/RGB-D/SPSLAM.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/SP-SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.o -c /SP-SLAM/Examples/RGB-D/SPSLAM.cc

CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /SP-SLAM/Examples/RGB-D/SPSLAM.cc > CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.i

CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /SP-SLAM/Examples/RGB-D/SPSLAM.cc -o CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.s

# Object files for target SPSLAM
SPSLAM_OBJECTS = \
"CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.o"

# External object files for target SPSLAM
SPSLAM_EXTERNAL_OBJECTS =

../Examples/RGB-D/SPSLAM: CMakeFiles/SPSLAM.dir/Examples/RGB-D/SPSLAM.cc.o
../Examples/RGB-D/SPSLAM: CMakeFiles/SPSLAM.dir/build.make
../Examples/RGB-D/SPSLAM: ../lib/libORB_SLAM2.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpangolin.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libdc1394.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/RGB-D/SPSLAM: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/RGB-D/SPSLAM: ../Thirdparty/g2o/lib/libg2o.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_surface.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_recognition.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_registration.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_keypoints.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_stereo.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_outofcore.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_tracking.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_people.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_visualization.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_io.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_segmentation.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_features.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_filters.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_sample_consensus.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_search.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_octree.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_kdtree.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_ml.so
../Examples/RGB-D/SPSLAM: /usr/local/lib/libpcl_common.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libboost_system.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libqhull.so
../Examples/RGB-D/SPSLAM: /usr/lib/libOpenNI.so
../Examples/RGB-D/SPSLAM: /usr/lib/libOpenNI2.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libfreetype.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtksys-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.2.so.6.2.0
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libXt.so
../Examples/RGB-D/SPSLAM: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
../Examples/RGB-D/SPSLAM: CMakeFiles/SPSLAM.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/SP-SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/RGB-D/SPSLAM"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SPSLAM.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SPSLAM.dir/build: ../Examples/RGB-D/SPSLAM

.PHONY : CMakeFiles/SPSLAM.dir/build

CMakeFiles/SPSLAM.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SPSLAM.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SPSLAM.dir/clean

CMakeFiles/SPSLAM.dir/depend:
	cd /SP-SLAM/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /SP-SLAM /SP-SLAM /SP-SLAM/cmake-build-debug /SP-SLAM/cmake-build-debug /SP-SLAM/cmake-build-debug/CMakeFiles/SPSLAM.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SPSLAM.dir/depend

