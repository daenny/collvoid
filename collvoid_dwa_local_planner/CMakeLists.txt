cmake_minimum_required(VERSION 2.8.3)
project(collvoid_dwa_local_planner)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED
        COMPONENTS
        base_local_planner
        collvoid_local_planner
        collvoid_srvs
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        nav_core
        nav_msgs
        pluginlib
        pcl_conversions
        roscpp
        tf
        )

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
        cfg/DWAPlanner.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES collvoid_dwa_local_planner
        CATKIN_DEPENDS
        dynamic_reconfigure
        pluginlib
        roscpp
)

add_library(collvoid_dwa_local_planner
        src/dwa_planner.cpp
        src/dwa_planner_ros.cpp
        src/goal_alignment_cost_function.cpp
        src/path_alignment_cost_function.cpp
        include/collvoid_dwa_local_planner/
        )
target_link_libraries(collvoid_dwa_local_planner base_local_planner ${catkin_LIBRARIES})
add_dependencies(collvoid_dwa_local_planner ${catkin_EXPORTED_TARGETS})
add_dependencies(collvoid_dwa_local_planner collvoid_local_planner)
add_dependencies(collvoid_dwa_local_planner collvoid_scoring_function)


install(TARGETS collvoid_dwa_local_planner
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES blp_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
