cmake_minimum_required(VERSION 2.4.6)
project(libzxing)

find_package(catkin REQUIRED)

add_custom_target(
	build_zxing ALL
	COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball)



add_custom_target(
	copy_zxing ALL
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/common/lib/libzxing.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
)

message(STATUS "Copying ${PROJECT_SOURCE_DIR}/common/lib/libzxing.a to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}")

	
catkin_package(
	DEPENDS 
	CATKIN_DEPENDS
	INCLUDE_DIRS common/include 
	LIBRARIES zxing
)

include_directories(${catkin_INCLUDE_DIRS})
add_library(zxing STATIC)
set_target_properties(zxing PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(zxing copy_zxing)
add_dependencies(copy_zxing build_zxing)

install(TARGETS zxing
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY common/include/
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
