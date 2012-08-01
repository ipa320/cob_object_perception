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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_download_test_data(http://cob-test-data-server/cob_test_dataset/cob_datamatrix/yaml/test_cases.yaml test/yaml/test_cases.yaml e44f3d6d2afd9e978cd7a16d87c1c335)

rosbuild_download_test_data(http://cob-test-data-server/cob_test_dataset/cob_datamatrix/bagfiles/withMilk.bag test/bagfiles/withMilk.bag 1c40f6a63f95046183487df77ba36b17)
rosbuild_download_test_data(http://cob-test-data-server/cob_test_dataset/cob_datamatrix/bagfiles/withoutMilk.bag test/bagfiles/withoutMilk.bag e5e913fcf724c98bebbd8c090f9070c0)

rosbuild_download_test_data(http://cob-test-data-server/cob_test_dataset/cob_datamatrix/yaml/withMilk.yaml test/yaml/withMilk.yaml 9b544261209abfe2c9cb5a9349644c74)
rosbuild_download_test_data(http://cob-test-data-server/cob_test_dataset/cob_datamatrix/yaml/withoutMilk.yaml test/yaml/withoutMilk.yaml 40721e8843e05ea7cd0a661caf9faf3f)

# Tests
rosbuild_add_rostest(test/component_test.xml)

