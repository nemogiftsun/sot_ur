cmake_minimum_required(VERSION 2.4.6)



#project(sot_ur)

#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs geometry_msgs sensor_msgs tf )

#catkin_package()



## LAAS cmake submodule part
set(PROJECT_DESCRIPTION "sot controller for Universal  Robot")
set(PROJECT_NAME sot_ur)
set(PROJECT_URL "")

set(CXX_DISABLE_WERROR False)
include(cmake/base.cmake)
include(cmake/python.cmake)


# Search for dependencies.
add_required_dependency("jrl-mathtools")
add_required_dependency("jrl-mal")
add_required_dependency("dynamic-graph")
add_required_dependency("dynamic-graph-python")
add_required_dependency("sot-core")

# This is required by the Python prologue.
add_required_dependency("sot-dynamic")

# Python
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

#set(SOURCES
#    src/ur_sot_controller.cpp
#    src/ur_device.cpp)

#set(HEADERS
#    include/ur_sot_controller.h
#    include/ur_device.h)

#add_library(sot_ur ${HEADERS} ${SOURCES})

#target_link_libraries(sot_ur dynamic-graph)
#target_link_libraries(sot_ur sot-core)
#target_link_libraries(sot_ur dynamic-graph-python)
#PKG_CONFIG_USE_DEPENDENCY(sot_ur "dynamic-graph")
#PKG_CONFIG_USE_DEPENDENCY(sot_ur "sot-core")
#PKG_CONFIG_USE_DEPENDENCY(sot_ur "dynamic-graph-python")

ADD_SUBDIRECTORY(src)
