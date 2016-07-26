cmake_minimum_required(VERSION 2.8.12)
project(cob_surface_classification)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release) # RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

set(catkin_RUN_PACKAGES
  cob_3d_features
  cob_3d_mapping_common
  cob_3d_segmentation
  cob_object_detection_msgs
  cv_bridge
  dynamic_reconfigure
  gazebo_msgs
  image_transport
  message_filters
  roscpp
  sensor_msgs
  std_msgs
)

set(catkin_BUILD_PACKAGES 
  ${catkin_RUN_PACKAGES}
  cmake_modules
  message_generation
)

find_package(catkin REQUIRED COMPONENTS
  ${catkin_BUILD_PACKAGES}
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
find_package(PCL 1.8 EXACT REQUIRED)# PATHS /home/rmb/.pcl-1.8 NO_DEFAULT_PATH)
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenMP)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  #${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()


#############################
## Message, CFG generation ##
#############################
add_message_files(
  FILES
  Int32Array.msg
  SegmentedPointCloud2.msg
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/NormalEstimation.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    ${catkin_RUN_PACKAGES}
    message_runtime
  DEPENDS
    Boost
    OpenCV
    OpenMP
    PCL
)


###########
## Build ##
###########
include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(cob_3d_curvature_segmentation
  common/src/refine_segmentation.cpp
)
add_dependencies(cob_3d_curvature_segmentation ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_3d_curvature_segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(surface_classification
  ros/src/surface_classification_node.cpp
  common/src/surface_classification.cpp
  ros/src/scene_recording.cpp
  ros/src/evaluation.cpp
  ros/src/simple_object_classification.cpp
)
target_compile_options(surface_classification PRIVATE -std=c++11 ${OpenMP_FLAGS})
add_dependencies(surface_classification ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(surface_classification
  cob_3d_curvature_segmentation
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenMP_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(simulation_scene_generator  common/src/simulation_scene_generator.cpp)
add_dependencies(simulation_scene_generator ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulation_scene_generator ${catkin_LIBRARIES})

add_executable(simulation_scene_reset ros/src/simulation_scene_reset.cpp)
add_dependencies(simulation_scene_reset ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulation_scene_reset ${catkin_LIBRARIES})


#############
## Install ##
#############
install(TARGETS cob_3d_curvature_segmentation surface_classification 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/sim_robot_config_back_extreme.py scripts/sim_robot_config_back.py scripts/sim_robot_config_home.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/ ros/include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY common/files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
)
