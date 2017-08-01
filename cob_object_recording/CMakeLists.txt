cmake_minimum_required(VERSION 2.8.3)
project(cob_object_recording)

find_package(catkin REQUIRED COMPONENTS
  cob_object_detection_msgs
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_filters
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

generate_dynamic_reconfigure_options(cfg/ObjectRecording.cfg)

catkin_package()

###########
## Build ##
###########
include_directories(
	ros/include
	${catkin_INCLUDE_DIRS}
)

## object_recording 
add_executable(object_recording 
	ros/src/object_recording.cpp
)
add_dependencies(object_recording ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_recording 
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

## object_recording_client 
add_executable(object_recording_client 
	ros/src/object_recording_client.cpp
)
add_dependencies(object_recording_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_recording_client
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


#############
## Install ##
#############
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
)

install(DIRECTORY ros/launch ros/rviz
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
)
