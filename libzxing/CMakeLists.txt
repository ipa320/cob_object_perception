cmake_minimum_required(VERSION 2.4.6)
project(libzxing)

find_package(catkin REQUIRED)

add_custom_target(
	build_zxing ALL
	COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball)

set(toon_libraries "zxing")
foreach(library ${toon_libraries})
	add_custom_target(
		copy_${library} ALL
		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/common/lib/lib${library}.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
		COMMENT "Copying ${PROJECT_SOURCE_DIR}/common/lib/lib${library}.a to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}"
	)
	add_library(${library} STATIC IMPORTED)
	# now you can use TARGET as if it were a regular cmake built target in your project
	add_dependencies(${library} copy_${library})
	add_dependencies(copy_${library} build_${library})
endforeach()

	
catkin_package(
	INCLUDE_DIRS
#		common/include 
	LIBRARIES
		zxing
	CATKIN_DEPENDS
	DEPENDS 
)

#install(TARGETS zxing
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

install(DIRECTORY common/include/
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
