cmake_minimum_required(VERSION 2.4.6)
project(cob_read_text)

set(catkin_RUN_PACKAGES 
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  pcl_ros 
  geometry_msgs
  sensor_msgs
  cob_read_text_data
  cob_tesseract_data
  image_view
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

find_package(Boost REQUIRED COMPONENTS thread filesystem system)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
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
	${catkin_RUN_PACKAGES}
DEPENDS
	OpenCV 
	Eigen
#	PCL
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
#	${PCL_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${VTK_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


## Declare libraries
add_library(read_text
	common/src/text_detect.cpp
)
target_link_libraries(read_text
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

## Declare a cpp executable
add_executable(run_detect
	common/src/run_detection.cpp common/src/text_detect.cpp
)
target_link_libraries(run_detect 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

add_executable(cob_read_text 
	ros/src/cob_read_text.cpp
)
target_link_libraries(cob_read_text 
	read_text
	${catkin_LIBRARIES}
)

add_executable(read_evaluation 
	ros/src/read_evaluation.cpp
)
target_link_libraries(read_evaluation 
	read_text
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

#add_executable(letter_evaluation
#	ros/src/letter_evaluation.cpp
#)
#target_link_libraries(letter_evaluation
#	${catkin_LIBRARIES}
#)

add_executable(labelBox 
	ros/src/labelBox.cpp
)
target_link_libraries(labelBox
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(create_correlation 
	common/src/createCorrelation.cpp
)
target_link_libraries(create_correlation 
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(record_kinect_prosilica
	ros/src/recordKinPro.cpp
)
target_link_libraries(record_kinect_prosilica
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(script_for_imglist 
	common/src/convertXML.cpp
)
target_link_libraries(script_for_imglist 
	${catkin_LIBRARIES}
)

add_executable(showPCD 
	common/src/showPCD.cpp
)
target_link_libraries(showPCD 
	${catkin_LIBRARIES}
	${VTK_LIBRARIES}
	${Boost_LIBRARIES}
)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS 
		read_text		
		run_detect
		cob_read_text 
		read_evaluation  
#		letter_evaluation 
		labelBox 
		record_kinect_prosilica 
		script_for_imglist 
		showPCD 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
		common/include
		ros/include
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

