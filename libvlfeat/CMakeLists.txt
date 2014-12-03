cmake_minimum_required(VERSION 2.8.3)
project(libvlfeat)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")
add_custom_target(
    build_vlfeat ALL
    COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball)


set(libraries "vl")
foreach(library ${libraries})
	add_custom_target(
		copy_${library} ALL
		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/common/lib/lib${library}.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
    	COMMENT "Copying ${PROJECT_SOURCE_DIR}/common/lib/lib${library}.so to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}"
	)
endforeach()

catkin_package(
	INCLUDE_DIRS
		common/include
	LIBRARIES
		vl
	CATKIN_DEPENDS
	DEPENDS
)

include_directories(
	common/include
)

foreach(library ${libraries})
	add_library(${library} SHARED)
	add_dependencies(${library} copy_${library})
	add_dependencies(copy_${library} build_vlfeat)
	set_target_properties(${library} PROPERTIES LINKER_LANGUAGE CXX)
endforeach()
set_target_properties(build_vlfeat PROPERTIES LINKER_LANGUAGE CXX)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS vl
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)