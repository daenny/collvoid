cmake_minimum_required(VERSION 2.8.3)
project(collvoid_stage)

find_package(catkin REQUIRED COMPONENTS collvoid_local_planner)
#catkin_python_setup()

catkin_package(
    DEPENDS collvoid_local_planner
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)