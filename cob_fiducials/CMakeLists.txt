cmake_minimum_required(VERSION 2.4.6)
project(cob_fiducials)

set(catkin_RUN_PACKAGES 
        roscpp
        sensor_msgs
        tf
        image_transport
        cv_bridge
        visualization_msgs
        cob_vision_utils
        cob_object_detection_msgs
        std_srvs
#	pcl_ros
#	pcl_conversions
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

find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
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
	${catkin_RUN_PACKAGES}
DEPENDS
	OpenCV
	TinyXML
	Boost
#	PCL
)


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	common/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${TinyXML_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
)

## Declare a cpp library

set(project_CPP_FILES
	common/src/pi/FiducialModelPi.cpp
        common/src/aruco/arucofidmarkers.cpp
        common/src/aruco/board.cpp
        common/src/aruco/boarddetector.cpp
        common/src/aruco/cameraparameters.cpp
        common/src/aruco/cvdrawingutils.cpp
        common/src/aruco/FiducialModelAruco.cpp
        common/src/aruco/marker.cpp
        common/src/aruco/markerdetector.cpp
)

## Declares a shared library. Resulting name will be lib<name>.so
add_library(${PROJECT_NAME}
	${project_CPP_FILES}
)

## Declare a cpp executable
add_executable(fiducials
	ros/src/fiducials.cpp
	${project_CPP_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${TinyXML_LIBRARIES}
	${Boost_LIBRARIES}
#	${PCL_LIBRARIES}
)

target_link_libraries(fiducials
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${TinyXML_LIBRARIES}
	${Boost_LIBRARIES}
#	${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(fiducials ${catkin_EXPORTED_TARGETS})

# set build flags for targets
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DBOOST_FILESYSTEM_VERSION=2")
set_target_properties(fiducials PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DBOOST_FILESYSTEM_VERSION=2")


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} fiducials
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

install(DIRECTORY common/files
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
	PATTERN ".svn" EXCLUDE
)

