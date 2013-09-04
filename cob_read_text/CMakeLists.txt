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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add include search paths
INCLUDE_DIRECTORIES(	${PROJECT_SOURCE_DIR}/ros/include
			${PROJECT_SOURCE_DIR}/common/include)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

# external pcl
#find_package(PCL 1.4 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_boost_directories()

rosbuild_add_library(read_text	common/src/text_detect.cpp)

rosbuild_add_executable(run_detect	common/src/run_detection.cpp common/src/text_detect.cpp)

rosbuild_add_executable(cob_read_text ros/src/cob_read_text.cpp)
target_link_libraries(cob_read_text read_text)
rosbuild_link_boost(cob_read_text thread)

rosbuild_add_executable(read_evaluation ros/src/read_evaluation.cpp)

rosbuild_add_executable(letter_evaluation ros/src/letter_evaluation.cpp)

rosbuild_add_executable(labelBox ros/src/labelBox.cpp)
rosbuild_link_boost(labelBox filesystem system)

rosbuild_add_executable(create_correlation common/src/createCorrelation.cpp)
rosbuild_link_boost(create_correlation filesystem system)

rosbuild_add_executable(record_kinect_prosilica ros/src/recordKinPro.cpp)
rosbuild_link_boost(record_kinect_prosilica filesystem system thread)

rosbuild_add_executable(script_for_imglist common/src/convertXML.cpp)

rosbuild_add_executable(showPCD common/src/showPCD.cpp)
target_link_libraries(showPCD ${PCL_LIBRARIES} vtkCommon vtkFiltering)
rosbuild_link_boost(showPCD thread)

rosbuild_add_roslaunch_check(launch)
