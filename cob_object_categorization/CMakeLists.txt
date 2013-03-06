cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include
		    ${PROJECT_SOURCE_DIR}/common/include)
		    #/usr/include/pcl-1.4/
		    #/usr/include/eigen3/
		    #/usr/include/boost/
		    #/usr/include/flann/
		    #/usr/include/vtk-5.4/)

# external pcl
#find_package(PCL 1.4 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Check for sse support
rosbuild_check_for_sse()
#set(SSE_FLAGS "-msse2 -mfpmath=sse")

rosbuild_add_boost_directories()

rosbuild_add_executable(object_categorization
				common/src/AbstractBlobDetector.cpp
				common/src/BlobFeature.cpp
				common/src/BlobList.cpp
				common/src/DetectorCore.cpp
				common/src/ICP.cpp
				common/src/JBKUtils.cpp
				common/src/Math3d.cpp
				common/src/ObjectClassifier.cpp
				#common/src/ObjectClassifierMain.cpp
				ros/src/object_categorization.cpp
				common/src/OpenCVUtils.cpp
				common/src/SharedImageJBK.cpp
				common/src/SharedImageSequence.cpp
				common/src/ThreeDUtils.cpp
				common/src/timer.cpp)
				#common/src/pcl/gfpfh.cpp)
				
rosbuild_add_executable(object_segmentation ros/src/segmentation_node.cpp)

rosbuild_add_compile_flags(object_categorization -D__LINUX__)
rosbuild_add_compile_flags(object_segmentation -D__LINUX__)

rosbuild_link_boost(object_categorization filesystem system signals)
rosbuild_link_boost(object_segmentation filesystem system signals)

target_link_libraries(object_categorization ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES})
target_link_libraries(object_segmentation ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES})
#target_link_libraries(object_categorization pcl_features pcl_common pcl_kdtree pcl_search pcl_filters pcl_io)
#target_link_libraries(object_segmentation pcl_features pcl_common pcl_kdtree pcl_search pcl_filters pcl_io)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
