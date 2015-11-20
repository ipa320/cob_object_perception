cmake_minimum_required(VERSION 2.8.3)
project(cob_texture_categorization)

set(catkin_RUN_PACKAGES # all ROS packages from package.xml (libopencv-dev is system dependency --> sudo apt-get install)
  pcl_ros
  roscpp
  roslib
  cv_bridge
  sensor_msgs
  message_filters
  image_transport
  libvlfeat
  cob_surface_classification
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
	cmake_modules
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Creates a bunch of environment variables that may be used later in the script
## e.g. catkin_INCLUDE_DIRS

find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES} # this makes ${catkin_LIBRARIES} include all libraries of ${catkin_BUILD_PACKAGES}
)
find_package(OpenCV REQUIRED)	# name identical to FindOpenCV.cmake in cmake_modules
# find_package(PCL REQUIRED) # is already done by inclusion of pcl_ros above


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need for build and run
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS
	common/include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
	OpenCV
#	PCL
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	common/include
	ros/include
	${catkin_INCLUDE_DIRS} # same name as find_package
	${OpenCV_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(texture_categorization_node
ros/src/texture_categorization.cpp common/src/create_lbp.cpp common/src/get_mapping.cpp common/src/lbp.cpp common/src/splitandmerge.cpp common/src/texture_features.cpp common/src/color_parameter.cpp common/src/amadasun.cpp common/src/compute_textures.cpp common/src/write_xml.cpp common/src/meanshift_3d.cpp common/src/run_meanshift_test.cpp common/src/meanshift.cpp common/src/depth_image.cpp common/src/segment_trans.cpp common/src/perspective_transformation.cpp common/src/create_train_data.cpp common/src/train_svm.cpp common/src/predict_svm.cpp common/src/train_ml.cpp
common/src/attribute_learning.cpp common/src/ifv_features.cpp
)

target_link_libraries(texture_categorization_node
	${catkin_LIBRARIES} # automatically links all catkin_BUILD_PACKAGES
	${OpenCV_LIBRARIES}
)

add_executable(texture_generator common/src/texture_generator.cpp
                                 common/src/color_parameter.cpp
                                 common/src/amadasun.cpp
)
target_link_libraries(texture_generator
	${catkin_LIBRARIES} # automatically links all catkin_BUILD_PACKAGES
	${OpenCV_LIBRARIES}
)

add_dependencies(texture_categorization_node ${catkin_EXPORTED_TARGETS})
add_dependencies(texture_generator ${catkin_EXPORTED_TARGETS})

# set build flags for targets
#set_target_properties(cob_3d_curvatureSegmentation PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DBOOST_FILESYSTEM_VERSION=2")
#set_target_properties(surface_classification PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DBOOST_FILESYSTEM_VERSION=2")


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS texture_categorization_node texture_generator
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)
