cmake_minimum_required(VERSION 2.8.3)
project(cob_object_categorization)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  cob_object_detection_msgs
  cob_perception_msgs
  cv_bridge
  image_transport
  message_filters
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

catkin_package()


###########
## Build ##
###########
include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
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

add_executable(object_categorization
  ${project_CPP_FILES}
)
add_dependencies(object_categorization ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_categorization
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${VTK_LIBRARIES}
)
set_target_properties(object_categorization PROPERTIES COMPILE_FLAGS "-D__LINUX__")


add_executable(object_segmentation 
  ros/src/segmentation_node.cpp
)
add_dependencies(object_segmentation ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_segmentation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
)
set_target_properties(object_segmentation PROPERTIES COMPILE_FLAGS "-D__LINUX__")


#############
## Install ##
#############
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
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
)
