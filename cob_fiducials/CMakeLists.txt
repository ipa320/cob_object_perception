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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/common/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(cob_fiducials common/src/pi/FiducialModelPi.cpp
	common/src/aruco/arucofidmarkers.cpp
	common/src/aruco/board.cpp
	common/src/aruco/boarddetector.cpp
	common/src/aruco/cameraparameters.cpp
	common/src/aruco/cvdrawingutils.cpp
	common/src/aruco/FiducialModelAruco.cpp
	common/src/aruco/marker.cpp
	common/src/aruco/markerdetector.cpp
)

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

rosbuild_add_executable(fiducials ros/src/fiducials.cpp)

# add compile flag
rosbuild_add_compile_flags(cob_fiducials -D__LINUX__ -DBOOST_FILESYSTEM_VERSION=2)
rosbuild_add_compile_flags(fiducials -D__LINUX__ -DBOOST_FILESYSTEM_VERSION=2)

# initialize boost directory search
rosbuild_add_boost_directories()

target_link_libraries(cob_fiducials tinyxml)
rosbuild_link_boost(${PROJECT_NAME} filesystem)

target_link_libraries(fiducials cob_fiducials)
