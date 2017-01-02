# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(collvoid_simple_global_planner)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib nav_core costmap_2d base_local_planner tf angles)


 include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   SYSTEM
)

add_library(${PROJECT_NAME} src/collvoid_simple_global_planner.cpp)

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

catkin_package(
    DEPENDS roscpp pluginlib nav_core costmap_2d base_local_planner tf angles
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)