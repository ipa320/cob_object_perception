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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#include_directories(/usr/local/lib/)
#include_directories(/opt/ocr/tesseract-3.00/api)
#include_directories(/opt/ocr/tesseract-3.00/ccmain)
#link_directories (/usr/local/lib) 

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/text_detect.cpp)
#target_link_libraries(${PROJECT_NAME} /usr/local/lib/libtesseract_api.so)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(read_text ${PORJECT_NAME})
rosbuild_add_library(read_text	src/text_detect.cpp)
rosbuild_add_executable(run_detect	src/run_detection.cpp
									src/text_detect.cpp)
