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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include
		    ${PROJECT_SOURCE_DIR}/common/include/cob_texture_categorization)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(texture_categorization_node ros/src/texture_categorization.cpp common/src/create_lbp.cpp common/src/get_mapping.cpp common/src/lbp.cpp common/src/splitandmerge.cpp common/src/texture_features.cpp common/src/color_parameter.cpp common/src/amadasun.cpp common/src/compute_textures.cpp common/src/write_xml.cpp common/src/meanshift_3d.cpp common/src/run_meanshift_test.cpp common/src/meanshift.cpp common/src/depth_image.cpp common/src/segment_trans.cpp common/src/perspective_transformation.cpp common/src/create_train_data.cpp common/src/train_svm.cpp common/src/predict_svm.cpp common/src/train_kneighbor.cpp)
