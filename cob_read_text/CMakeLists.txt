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
	read_text
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


####################################
## Declare a cpp libraries
####################################

## read_text
add_library(read_text
	common/src/text_detect.cpp
)
target_link_libraries(read_text
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(read_text ${catkin_EXPORTED_TARGETS})

####################################
## Declare a cpp executable
####################################

## read_text_run_detect
add_executable(read_text_run_detect
	common/src/run_detection.cpp common/src/text_detect.cpp
)
target_link_libraries(read_text_run_detect 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(read_text_run_detect ${catkin_EXPORTED_TARGETS})

## cob_read_text 
add_executable(cob_read_text 
	ros/src/cob_read_text.cpp
)
target_link_libraries(cob_read_text 
	read_text
	${catkin_LIBRARIES}
)
add_dependencies(cob_read_text ${catkin_EXPORTED_TARGETS})

## read_text_read_evaluation 
add_executable(read_text_read_evaluation 
	ros/src/read_evaluation.cpp
)
target_link_libraries(read_text_read_evaluation 
	read_text
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(read_text_read_evaluation ${catkin_EXPORTED_TARGETS})

## read_text_letter_evaluation
#add_executable(read_text_letter_evaluation
#	ros/src/letter_evaluation.cpp
#)
#target_link_libraries(read_text_letter_evaluation
#	${catkin_LIBRARIES}
#)
#add_dependencies(read_text_letter_evaluation  ${catkin_EXPORTED_TARGETS})

## read_text_labelBox 
add_executable(read_text_labelBox 
	ros/src/labelBox.cpp
)
target_link_libraries(read_text_labelBox
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(read_text_labelBox  ${catkin_EXPORTED_TARGETS})

## read_text_create_correlation
add_executable(read_text_create_correlation 
	common/src/createCorrelation.cpp
)
target_link_libraries(read_text_create_correlation 
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(read_text_create_correlation  ${catkin_EXPORTED_TARGETS})

## read_text_record_kinect_prosilica
add_executable(read_text_record_kinect_prosilica
	ros/src/recordKinPro.cpp
)
target_link_libraries(read_text_record_kinect_prosilica
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(read_text_record_kinect_prosilica  ${catkin_EXPORTED_TARGETS})

## read_text_script_for_imglist 
add_executable(read_text_script_for_imglist 
	common/src/convertXML.cpp
)
target_link_libraries(read_text_script_for_imglist 
	${catkin_LIBRARIES}
)
add_dependencies(read_text_script_for_imglist  ${catkin_EXPORTED_TARGETS})

## read_text_showPCD
add_executable(read_text_showPCD 
	common/src/showPCD.cpp
)
target_link_libraries(read_text_showPCD 
	${catkin_LIBRARIES}
	${VTK_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(read_text_showPCD ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS 
		cob_read_text 
		read_text
		read_text_create_correlation		
		read_text_run_detect
		read_text_read_evaluation  
#		read_text_letter_evaluation 
		read_text_labelBox 
		read_text_record_kinect_prosilica 
		read_text_script_for_imglist 
		read_text_showPCD 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
		common/include/${PROJECT_NAME}/
#		ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

