cmake_minimum_required(VERSION 2.8.3)
project(cob_texture_categorization)

set(catkin_RUN_PACKAGES
	cob_surface_classification
	cv_bridge
	image_transport
	libvlfeat
	message_filters
	pcl_ros
	roscpp
	sensor_msgs
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
)

find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS
		common/include
		ros/include
	LIBRARIES
		texture_categorization
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		OpenCV

)


###########
## Build ##
###########
include_directories(
	common/include
	ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

### library
add_library(texture_categorization
	common/src/create_lbp.cpp
	common/src/get_mapping.cpp
	common/src/lbp.cpp
	common/src/splitandmerge.cpp
	common/src/texture_features.cpp
	common/src/color_parameter.cpp
	common/src/amadasun.cpp
	common/src/compute_textures.cpp
	common/src/write_xml.cpp
	common/src/meanshift_3d.cpp
	common/src/run_meanshift_test.cpp
	common/src/meanshift.cpp
	common/src/depth_image.cpp
	common/src/segment_trans.cpp
	common/src/perspective_transformation.cpp
	common/src/create_train_data.cpp
	common/src/train_svm.cpp
	common/src/predict_svm.cpp
	common/src/train_ml.cpp
	common/src/attribute_learning.cpp
	common/src/ifv_features.cpp
)
add_dependencies(texture_categorization ${catkin_EXPORTED_TARGETS})
target_link_libraries(texture_categorization
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

### executable
add_executable(texture_categorization_node
	ros/src/texture_categorization.cpp
	common/src/create_lbp.cpp
	common/src/get_mapping.cpp
	common/src/lbp.cpp
	common/src/splitandmerge.cpp
	common/src/texture_features.cpp
	common/src/color_parameter.cpp
	common/src/amadasun.cpp
	common/src/compute_textures.cpp
	common/src/write_xml.cpp
	common/src/meanshift_3d.cpp
	common/src/run_meanshift_test.cpp
	common/src/meanshift.cpp
	common/src/depth_image.cpp
	common/src/segment_trans.cpp
	common/src/perspective_transformation.cpp
	common/src/create_train_data.cpp
	common/src/train_svm.cpp
	common/src/predict_svm.cpp
	common/src/train_ml.cpp
	common/src/attribute_learning.cpp
	common/src/ifv_features.cpp
)
add_dependencies(texture_categorization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(texture_categorization_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)


add_executable(texture_generator
	common/src/texture_generator.cpp
	common/src/color_parameter.cpp
	common/src/amadasun.cpp
)
add_dependencies(texture_generator ${catkin_EXPORTED_TARGETS})
target_link_libraries(texture_generator
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)


#############
## Install ##
#############
install(TARGETS texture_categorization_node texture_generator
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME} ros/include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
)
