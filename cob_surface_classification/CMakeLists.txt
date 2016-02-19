cmake_minimum_required(VERSION 2.8.3)
project(cob_surface_classification)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release) # RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  cmake_modules
  # cob_3d_features
  # cob_3d_mapping_common
  # cob_3d_segmentation
  cob_object_detection_msgs
  cv_bridge
  dynamic_reconfigure
  gazebo_msgs
  image_transport
  message_filters
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(VTK REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
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

catkin_package()


###########
## Build ##
###########
# include_directories(
  # common/include
  # ros/include
  # ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  # ${OpenMP_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
  # ${VTK_INCLUDE_DIRS}
# )
# 
# cmake_policy(PUSH)
# cmake_policy(SET CMP0003 OLD) # use old-style link directories for now
# add_library(cob_3d_curvatureSegmentation
  # common/src/refine_segmentation.cpp
# )
# cmake_policy(POP)
# add_dependencies(cob_3d_curvatureSegmentation ${catkin_EXPORTED_TARGETS})
# target_link_libraries(cob_3d_curvatureSegmentation
  # ${catkin_LIBRARIES}
  # ${VTK_LIBRARIES}
# )
# 
# cmake_policy(PUSH)
# cmake_policy(SET CMP0003 OLD) # use old-style link directories for now
# add_executable(surface_classification
  # ros/src/surface_classification_node.cpp
  # common/src/surface_classification.cpp
  # ros/src/scene_recording.cpp
  # ros/src/evaluation.cpp
  # ros/src/simple_object_classification.cpp
# )
# set_target_properties(surface_classification PROPERTIES COMPILE_FLAGS "-std=c++11")
# cmake_policy(POP)
# add_dependencies(surface_classification ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(surface_classification
  # cob_3d_curvatureSegmentation
  # ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
# )
# 
# add_executable(simulation_scene_generator  common/src/simulation_scene_generator.cpp)
# add_dependencies(simulation_scene_generator ${catkin_EXPORTED_TARGETS})
# target_link_libraries(simulation_scene_generator ${catkin_LIBRARIES})
# 
# add_executable(simulation_scene_reset ros/src/simulation_scene_reset.cpp)
# add_dependencies(simulation_scene_reset ${catkin_EXPORTED_TARGETS})
# target_link_libraries(simulation_scene_reset ${catkin_LIBRARIES})


#############
## Install ##
#############
# install(TARGETS cob_3d_curvatureSegmentation surface_classification 
  # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# 
# install(PROGRAMS scripts/sim_robot_config_back_extreme.py scripts/sim_robot_config_back.py scripts/sim_robot_config_home.py
  # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# 
# install(DIRECTORY common/include/${PROJECT_NAME}/ ros/include/${PROJECT_NAME}
  # DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
# )
# 
# install(DIRECTORY ros/launch
  # DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
# )
# 
# install(DIRECTORY models
  # DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# 
# install(DIRECTORY common/files
  # DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
# )
