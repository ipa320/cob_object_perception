cmake_minimum_required(VERSION 2.6)
project(cob_object_recording)

set(catkin_RUN_PACKAGES 
  pcl_ros # Pulls in PCL variabels for CMake automatically
  roscpp
  roslib
  dynamic_reconfigure
  cv_bridge
  std_msgs
  std_srvs
  sensor_msgs
  message_filters
  image_transport
  visualization_msgs
  cob_object_detection_msgs
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
	${catkin_BUILD_PACKAGES}
)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/ObjectRecording.cfg)

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
	ros/include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
	ros/include
	${catkin_INCLUDE_DIRS}
)

##############################
## Declare a cpp executable
##############################

## object_recording 
add_executable(object_recording 
	ros/src/object_recording.cpp
)
target_link_libraries(object_recording 
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(object_recording ${catkin_EXPORTED_TARGETS})

## object_recording_client 
add_executable(object_recording_client 
	ros/src/object_recording_client.cpp
)
target_link_libraries(object_recording_client
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(object_recording_client ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS 
		object_recording 
		object_recording_client 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
		ros/include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

