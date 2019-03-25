cmake_minimum_required(VERSION 2.8.3)
project(cob_object_detection_fake)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#############
## Install ##
#############
install(PROGRAMS ros/scripts/object_detection_fake.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
)
