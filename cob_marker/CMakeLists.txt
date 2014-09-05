cmake_minimum_required(VERSION 2.4.6)
project(cob_marker)

# To find FindDMTX.cmake for its later use in find_package
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
#message(STATUS "${CMAKE_MODULE_PATH}")

set(catkin_PACKAGES 
	cmake_modules
	roscpp
	pcl_ros # Should automatically pull in all PCL dependecies
	nodelet
	cv_bridge
	message_filters
	tf
	tf_conversions
	cob_object_detection_msgs
	visualization_msgs
	actionlib
	libzxing
	image_transport
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Creates a bunch of environment variables that may be used later in the script
## e.g. catkin_INCLUDE_DIRS

find_package(catkin REQUIRED COMPONENTS
	${catkin_PACKAGES}
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals)
find_package(DMTX REQUIRED)
find_package(ImageMagick COMPONENTS Magick++ REQUIRED)

#find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS
	common/include
LIBRARIES
	${PROJECT_NAME}
CATKIN_DEPENDS
	${catkin_PACKAGES}
DEPENDS
	OpenCV
	Boost
	DMTX
	ImageMagick
#	PCL
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
	common/include
#	/usr/include/ImageMagick
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${DMTX_INCLUDE_DIRS}
	${ImageMagick_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
)


set(project_CPP_FILES
	common/src/MagickBitmapSource.cpp
)

## Declare a cpp executable
add_executable(marker
	ros/src/marker_action.cpp
	${project_CPP_FILES}
)

target_link_libraries(marker
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
	${ImageMagick_LIBRARIES}
	${DMTX_LIBRARIES}
#	${PCL_LIBRARIES}
)

add_dependencies(marker ${catkin_EXPORTED_TARGETS})

# set build flags for targets
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DBOOST_FILESYSTEM_VERSION=2")
#set_target_properties(marker PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DBOOST_FILESYSTEM_VERSION=2")


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS marker
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

