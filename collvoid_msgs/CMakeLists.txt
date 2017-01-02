# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
cmake_minimum_required(VERSION 2.8.3)
project(collvoid_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs)

add_message_files(
  FILES
  AggregatedPoseTwist.msg
  collvoid_debug.msg
  PoseTwistWithCovariance.msg
  PoseArrayWeighted.msg
)

generate_messages(
    DEPENDENCIES geometry_msgs nav_msgs
)

catkin_package(
    DEPENDS geometry_msgs nav_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)