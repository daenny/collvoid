# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(collvoid_local_planner)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
roslib roscpp collvoid_msgs collvoid_srvs visualization_msgs laser_geometry tf angles base_local_planner pluginlib nav_core actionlib move_base_msgs std_srvs dynamic_reconfigure amcl)

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
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

generate_dynamic_reconfigure_options(cfg/Collvoid.cfg)

#uncomment if you have defined messages
#add_message_files(
#  FILES
#)
#uncomment if you have defined services
add_service_files(
  FILES
  GetCollvoidTwist.srv
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
 )

#find_package(Boost REQUIRED COMPONENTS signals signal)

include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    roscpp collvoid_msgs visualization_msgs laser_geometry tf angles base_local_planner pluginlib nav_core actionlib move_base_msgs std_srvs dynamic_reconfigure amcl
      INCLUDE_DIRS include
    LIBRARIES collvoid_local_planner collvoid_scoring_function
)


add_library(collvoid_scoring_function src/Agent.cpp src/publisher_helpers.cpp src/collvoid_scoring_function.cpp src/orca.cpp src/clearpath.cpp)
 add_dependencies(collvoid_scoring_function ${${PROJECT_NAME}_EXPORTED_TARGETS})
 target_link_libraries(collvoid_scoring_function
 ${catkin_LIBRARIES}
 ${Boost_LIBRARIES}
 )


add_library(collvoid_local_planner src/collvoid_local_planner.cpp src/Agent.cpp src/ROSAgent.cpp src/orca.cpp src/collvoid_publishers.cpp src/clearpath.cpp)
add_dependencies(collvoid_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(collvoid_local_planner collvoid_msgs_generate_messages_cpp)

target_link_libraries(collvoid_local_planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )


add_executable(MePublisher src/me_publisher.cpp src/clearpath.cpp src/publisher_helpers.cpp)
target_link_libraries(MePublisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})
add_dependencies(MePublisher ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(MePublisher collvoid_msgs_generate_messages_cpp)



add_executable(ROSAgent src/collvoid_local_planner.cpp src/Agent.cpp src/ROSAgent.cpp src/orca.cpp src/collvoid_publishers.cpp src/clearpath.cpp)
target_link_libraries(ROSAgent
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})
add_dependencies(ROSAgent ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(ROSAgent collvoid_msgs_generate_messages_cpp)

