# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(point-cloud-read)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
#find_package(PCL REQUIRED)

# add PCL library
#include_directories(${PCL_INCLUDE_DIRS} )
#link_directories(${PCL_LIBRARY_DIRS} )

# Fix some PCL bug related to Ubuntu
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpIDL)
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

yarp_add_idl(IDL_GEN_FILES src/idl.thrift)
source_group("IDL Files" FILES src/idl.thrift)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} src/main.cpp src/idl.thrift ${IDL_GEN_FILES})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
