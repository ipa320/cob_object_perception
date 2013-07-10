cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include
		    ${PROJECT_SOURCE_DIR}/common/include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


rosbuild_add_library(cob_3d_curvatureSegmentation
	common/src/refine_segmentation.cpp)
	
target_link_libraries(cob_3d_curvatureSegmentation cob_3d_segmentation cob_3d_mapping_common)



rosbuild_add_executable(surface_classification ros/src/surface_classification_node.cpp
												common/src/surface_classification.cpp)
rosbuild_add_executable(surface_classification ros/src/surface_classification_node.cpp
												ros/src/scene_recording.cpp)
#target_link_libraries(surface_classification cob_3d_mapping_common)
target_link_libraries(surface_classification cob_3d_curvatureSegmentation)
rosbuild_link_boost(surface_classification system)

#rosbuild_add_library(test common/include/cob_surface_classification/impl/curvatureSegmentation.hpp)
#target_link_libraries(test cob_3d_mapping_common)
#rosbuild_link_boost(test system)