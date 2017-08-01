cmake_minimum_required(VERSION 2.8.3)
project(cob_read_text)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread filesystem system)
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

## run_detect
add_executable(run_detect
  common/src/run_detection.cpp
)
add_dependencies(run_detect ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_detect
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

## read_evaluation 
add_executable(read_evaluation 
  ros/src/read_evaluation.cpp
)
add_dependencies(read_evaluation ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_evaluation 
  read_text
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## letter_evaluation
#add_executable(letter_evaluation
#  ros/src/letter_evaluation.cpp
#)
#add_dependencies(letter_evaluation  ${catkin_EXPORTED_TARGETS})
#target_link_libraries(letter_evaluation
#  ${catkin_LIBRARIES}
#)

## labelBox 
add_executable(labelBox 
  ros/src/labelBox.cpp
)
add_dependencies(labelBox  ${catkin_EXPORTED_TARGETS})
target_link_libraries(labelBox
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## create_correlation
add_executable(create_correlation 
  common/src/createCorrelation.cpp
)
add_dependencies(create_correlation  ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_correlation 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## record_kinect_prosilica
add_executable(record_kinect_prosilica
  ros/src/recordKinPro.cpp
)
add_dependencies(record_kinect_prosilica  ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_kinect_prosilica
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## script_for_imglist 
add_executable(script_for_imglist 
  common/src/convertXML.cpp
)
add_dependencies(script_for_imglist  ${catkin_EXPORTED_TARGETS})
target_link_libraries(script_for_imglist 
  ${catkin_LIBRARIES}
)

## showPCD
## cmake --help-policy CMP0003
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif() #(COMMAND cmake_policy)
add_executable(showPCD 
  common/src/showPCD.cpp
)
add_dependencies(showPCD ${catkin_EXPORTED_TARGETS})
target_link_libraries(showPCD 
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS 
    read_text
    run_detect
    cob_read_text
    read_evaluation
#    letter_evaluation
    labelBox
    create_correlation
    record_kinect_prosilica
    script_for_imglist
    showPCD 
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
