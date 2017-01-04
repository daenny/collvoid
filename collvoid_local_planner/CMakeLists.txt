# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(collvoid_local_planner)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        roslib roscpp collvoid_msgs collvoid_srvs visualization_msgs laser_geometry tf angles base_local_planner pluginlib nav_core actionlib move_base_msgs std_srvs dynamic_reconfigure amcl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


generate_dynamic_reconfigure_options(cfg/Collvoid.cfg)

add_service_files(
        FILES
        GetCollvoidTwist.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
)

include_directories(${Boost_INCLUDE_DIRS})


catkin_package(
        DEPENDS
        CATKIN_DEPENDS
        roscpp collvoid_msgs visualization_msgs laser_geometry tf angles base_local_planner pluginlib nav_core actionlib move_base_msgs std_srvs dynamic_reconfigure amcl
        INCLUDE_DIRS include
        LIBRARIES collvoid_local_planner collvoid_scoring_function
)


add_library(collvoid_scoring_function
        src/Agent.cpp
        src/collvoid_scoring_function.cpp
        src/orca.cpp
        src/clearpath.cpp
        src/collvoid_publishers.cpp
        include/collvoid_local_planner/)
add_dependencies(collvoid_scoring_function ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(collvoid_scoring_function ${catkin_EXPORTED_TARGETS})
target_link_libraries(collvoid_scoring_function
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )


add_library(collvoid_local_planner src/collvoid_local_planner.cpp src/Agent.cpp src/ROSAgent.cpp src/orca.cpp src/collvoid_publishers.cpp src/clearpath.cpp)
add_dependencies(collvoid_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(collvoid_local_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(collvoid_local_planner
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )


add_executable(MePublisher src/me_publisher.cpp src/clearpath.cpp src/collvoid_publishers.cpp)
target_link_libraries(MePublisher
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES})
add_dependencies(MePublisher ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(MePublisher ${catkin_EXPORTED_TARGETS})


add_executable(ROSAgent src/collvoid_local_planner.cpp src/Agent.cpp src/ROSAgent.cpp src/orca.cpp src/collvoid_publishers.cpp src/clearpath.cpp)
target_link_libraries(ROSAgent
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES})
add_dependencies(ROSAgent ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(ROSAgent ${catkin_EXPORTED_TARGETS})

