cmake_minimum_required(VERSION 2.8.3)
project(libzxing)

find_package(catkin REQUIRED)

catkin_destinations() # set-up destination variables
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include(ExternalProject)
ExternalProject_Add(EP_${PROJECT_NAME}
    URL https://github.com/ipa320/thirdparty/raw/master/zxing-2.1.tar.gz
    URL_MD5 00048357b97ca779dd91ad35c87e5058

    CONFIGURE_COMMAND cd ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/cpp && scons lib
    SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
    BUILD_COMMAND ""
        # copy headers to devel space (catkin does not like headers in source space)
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/cpp/build/core/src/zxing ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/libzxing/zxing
        # copy libs, set-up soname chain
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/cpp/build/libzxing.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
    INSTALL_COMMAND ""
)

catkin_package(
    CFG_EXTRAS libzxing-extras.cmake # catkin_package does not support artifacts in devel space
    EXPORTED_TARGETS EP_${PROJECT_NAME}
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/libzxing/zxing/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/libzxing/zxing
    FILES_MATCHING PATTERN "*.h"
)

install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libzxing.a
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
