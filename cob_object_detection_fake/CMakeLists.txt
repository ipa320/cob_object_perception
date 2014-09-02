cmake_minimum_required(VERSION 2.6)
project(cob_object_detection_fake)

set(catkin_PACKAGES 
	rospy
	cob_object_detection_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Creates a bunch of environment variables that may be used later in the script
## e.g. catkin_INCLUDE_DIRS

find_package(catkin REQUIRED COMPONENTS
	${catkin_PACKAGES}
)

## Uncomment if the package has a setup.py
catkin_python_setup()

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
LIBRARIES
CATKIN_DEPENDS
	${catkin_PACKAGES}
DEPENDS
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS ros/scripts/object_detection_fake
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

