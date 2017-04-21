cmake_minimum_required(VERSION 2.8.3)
project(collvoid_amcl)

find_package(catkin REQUIRED
  COMPONENTS
  collvoid_msgs
            rosbag
            roscpp
            tf
            dynamic_reconfigure
            nav_msgs
        )

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
  CATKIN_DEPENDS
  collvoid_msgs
        rosbag
        roscpp
        dynamic_reconfigure
        tf
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
)

include_directories(include/amcl include/amcl/map include/amcl/sensors include/amcl/pf)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(collvoid_amcl_pf
                    src/amcl/pf/pf.c
                    src/amcl/pf/pf_kdtree.c
                    src/amcl/pf/pf_pdf.c
                    src/amcl/pf/pf_vector.c
                    src/amcl/pf/eig3.c
                    src/amcl/pf/pf_draw.c)

add_library(collvoid_amcl_map
                    src/amcl/map/map.c
                    src/amcl/map/map_cspace.cpp
                    src/amcl/map/map_range.c
                    src/amcl/map/map_store.c
                    src/amcl/map/map_draw.c)

add_library(collvoid_amcl_sensors
                    src/amcl/sensors/amcl_sensor.cpp
                    src/amcl/sensors/amcl_odom.cpp
                    src/amcl/sensors/amcl_laser.cpp)
target_link_libraries(collvoid_amcl_sensors collvoid_amcl_map collvoid_amcl_pf)


add_executable(collvoid_amcl
                       src/amcl_node.cpp)
add_dependencies(collvoid_amcl collvoid_amcl_gencfg)
add_dependencies(collvoid_amcl collvoid_msgs_generate_messages_cpp)

target_link_libraries(collvoid_amcl
    collvoid_amcl_sensors collvoid_amcl_map collvoid_amcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install( TARGETS
    collvoid_amcl collvoid_amcl_sensors collvoid_amcl_map collvoid_amcl_pf
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/amcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)
