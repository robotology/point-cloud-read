# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(point-cloud-read)

# mandatory use of these packages
find_package(YARP 3.0.101 REQUIRED COMPONENTS math OS sig pcl)
# finding the yarp pcl component already implies finding pcl. 
# pcl cmake is broken and the linking crashes if find_package is called multiple times.
#find_package(PCL 1.2 REQUIRED)
find_package(ICUBcontrib REQUIRED)

# add PCL library
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Fix some PCL bug related to Ubuntu
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

yarp_idl_to_dir(idl.thrift ${CMAKE_SOURCE_DIR})

include(idl_thrift.cmake)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES}
                                      ${PCL_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
