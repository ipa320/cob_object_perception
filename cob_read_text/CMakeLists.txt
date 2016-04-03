cmake_minimum_required(VERSION 2.8.3)
project(cob_read_text)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread filesystem system)
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
  ${Boost_INCLUDE_DIRS}
)


## read_text
add_library(read_text
  common/src/text_detect.cpp
)
add_dependencies(read_text ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_text
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## read_text_run_detect
add_executable(read_text_run_detect
  common/src/run_detection.cpp
)
add_dependencies(read_text_run_detect ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_text_run_detect
  read_text
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## cob_read_text 
add_executable(cob_read_text 
  ros/src/cob_read_text.cpp
)
add_dependencies(cob_read_text ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_read_text 
  read_text
  ${catkin_LIBRARIES}
)

## read_text_read_evaluation 
add_executable(read_text_read_evaluation 
  ros/src/read_evaluation.cpp
)
add_dependencies(read_text_read_evaluation ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_text_read_evaluation 
  read_text
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## read_text_letter_evaluation
#add_executable(read_text_letter_evaluation
#  ros/src/letter_evaluation.cpp
#)
#add_dependencies(read_text_letter_evaluation  ${catkin_EXPORTED_TARGETS})
#target_link_libraries(read_text_letter_evaluation
#  ${catkin_LIBRARIES}
#)

## read_text_labelBox 
add_executable(read_text_labelBox 
  ros/src/labelBox.cpp
)
add_dependencies(read_text_labelBox  ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_text_labelBox
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## read_text_create_correlation
add_executable(read_text_create_correlation 
  common/src/createCorrelation.cpp
)
add_dependencies(read_text_create_correlation  ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_text_create_correlation 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## read_text_record_kinect_prosilica
add_executable(read_text_record_kinect_prosilica
  ros/src/recordKinPro.cpp
)
add_dependencies(read_text_record_kinect_prosilica  ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_text_record_kinect_prosilica
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## read_text_script_for_imglist 
add_executable(read_text_script_for_imglist 
  common/src/convertXML.cpp
)
add_dependencies(read_text_script_for_imglist  ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_text_script_for_imglist 
  ${catkin_LIBRARIES}
)

## read_text_showPCD
## cmake --help-policy CMP0003
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif() #(COMMAND cmake_policy)
add_executable(read_text_showPCD 
  common/src/showPCD.cpp
)
add_dependencies(read_text_showPCD ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_text_showPCD 
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS 
    read_text
    read_text_run_detect
    cob_read_text
    read_text_read_evaluation
#    read_text_letter_evaluation
    read_text_labelBox
    read_text_create_correlation
    read_text_record_kinect_prosilica
    read_text_script_for_imglist
    read_text_showPCD 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/dictionary.py scripts/filelist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
    common/include/${PROJECT_NAME}/
    ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
