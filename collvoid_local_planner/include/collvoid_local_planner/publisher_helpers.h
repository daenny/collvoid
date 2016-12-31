//
// Created by danielclaes on 23/06/15.
//

#ifndef COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H
#define COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include "collvoid_local_planner/ROSAgent.h"

using namespace collvoid;

namespace collvoid {

    void publishObstacleLines(const std::vector<Obstacle>& obstacles_lines, std::string base_frame, std::string name_space, ros::Publisher line_pub);

    void publishMePosition(ROSAgent* me, std::string base_frame, std::string name_space, ros::Publisher me_pub);

    void fillMarkerWithROSAgent(visualization_msgs::MarkerArray& marker, ROSAgent* agent, std::string base_frame, std::string name_space);

    void publishNeighborPositions(std::vector<AgentPtr> &neighbors, std::string frame, std::string name_space,
                                  ros::Publisher neighbors_pub);

}



#endif //COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H
