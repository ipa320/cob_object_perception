cmake_minimum_required(VERSION 2.6)
project(cob_object_categorization)

set(catkin_PACKAGES 
	pcl_ros # Should automatically pull in all PCL dependecies
	roscpp
	cv_bridge
	sensor_msgs
	message_filters
	image_transport
	nodelet
	cob_perception_msgs
	cob_object_detection_msgs
	tf
	actionlib
	actionlib_msgs
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
find_package(PCL REQUIRED)
find_package(eigen REQUIRED)
find_package(VTK REQUIRED)

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
CATKIN_DEPENDS
	${catkin_PACKAGES}
DEPENDS
	OpenCV 
	eigen
	PCL
	VTK
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
	common/include
	ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${eigen_INCLUDE_DIRS}
	${VTK_INCLUDE_DIRS}
)


set(project_CPP_FILES
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
	common/src/timer.cpp
	common/src/pcl/rsd.cpp
	#common/src/pcl/gfpfh.cpp)
)

## Declare a cpp executable
add_executable(object_categorization
	${project_CPP_FILES}
)

add_executable(object_segmentation 
	ros/src/segmentation_node.cpp
)

target_link_libraries(object_categorization
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${eigen_LIBRARIES}
	${VTK_LIBRARIES}
)

target_link_libraries(object_segmentation
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${eigen_LIBRARIES}
)

set_target_properties(object_categorization PROPERTIES COMPILE_FLAGS "-D__LINUX__")
set_target_properties(object_segmentation PROPERTIES COMPILE_FLAGS "-D__LINUX__")

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS 
		object_categorization 
		object_segmentation
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
		common/include/${PROJECT_NAME}/
		common/include/pcl/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)


