# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(collvoid_local_planner)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roslib roscpp collvoid_msgs visualization_msgs laser_geometry tf angles base_local_planner pluginlib nav_core actionlib move_base_msgs std_srvs dynamic_reconfigure amcl)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
 include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   SYSTEM
)
 

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


generate_dynamic_reconfigure_options(cfg/Collvoid.cfg)
# make sure configure headers are built before any node using them

# add dynamic reconfigure api
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

#uncomment if you have defined messages
#add_message_files(
#  FILES
  # TODO: List your msg files here
#)
#uncomment if you have defined services
#add_service_files(
#  FILES
  # TODO: List your msg files here
#)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS signals signal)
include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

catkin_package(
    DEPENDS roscpp collvoid_msgs visualization_msgs laser_geometry tf angles base_local_planner pluginlib nav_core actionlib move_base_msgs std_srvs dynamic_reconfigure amcl
    CATKIN_DEPENDS # TODO
    roslib
    roscpp_serialization
    rosconsole
    roscpp
    dynamic_reconfigure
    message_generation
    tf
    pluginlib
    costmap_2d
    nav_core

    INCLUDE_DIRS include
    LIBRARIES collvoid_local_planner
)

add_library(collvoid_local_planner src/collvoid_local_planner.cpp src/Agent.cpp src/ROSAgent.cpp src/orca.cpp src/collvoid_publishers.cpp src/clearpath.cpp)
add_dependencies(collvoid_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(collvoid_local_planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )



add_executable(ROSAgent src/collvoid_local_planner.cpp src/Agent.cpp src/ROSAgent.cpp src/orca.cpp src/collvoid_publishers.cpp src/clearpath.cpp)
target_link_libraries(ROSAgent
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})
add_dependencies(ROSAgent ${${PROJECT_NAME}_EXPORTED_TARGETS})


#add_executable(Helper src/helper.cpp src/clearpath.cpp)
#target_link_libraries(Helper ${Boost_LIBRARIES})

## Generate added messages and services with any dependencies listed here
# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
