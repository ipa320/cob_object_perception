cmake_minimum_required(VERSION 2.8.3)
project(cob_marker)

# To find FindDMTX.cmake for its later use in find_package
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
#message(STATUS "${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  cob_object_detection_msgs
  cv_bridge
  image_transport
  libzxing
  message_filters
  nodelet
  pcl_ros
  roscpp
  tf
  tf_conversions
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS signals)
find_package(DMTX REQUIRED)
find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

catkin_package()

###########
## Build ##
###########
include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${DMTX_INCLUDE_DIRS}
  ${ImageMagick_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(marker
  ros/src/marker_action.cpp
  common/src/MagickBitmapSource.cpp
)
add_dependencies(marker ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ImageMagick_LIBRARIES}
  ${DMTX_LIBRARIES}
#  ${PCL_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS marker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/marker_action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
)
