//
// Created by danielclaes on 23/06/15.
//

#ifndef COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H
#define COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include "collvoid_local_planner/Vector2.h"
#include "collvoid_local_planner/Agent.h"

using namespace collvoid;

namespace collvoid_scoring_function {
    void publishVOs(Vector2 &pos, const std::vector<VO> &truncated_vos, bool use_truncation, std::string base_frame,
                    std::string name_space, ros::Publisher vo_pub);

    void fillMarkerWithParams(visualization_msgs::MarkerArray &marker, double radius, Vector2 position, double yaw,
                              std::string base_frame,
                              std::string name_space);

    void publishMePosition(double radius, tf::Stamped<tf::Pose> global_pose, std::string base_frame,
                           std::string name_space, ros::Publisher me_pub);

    void publishNeighborPositions(std::vector<AgentPtr> &neighbors, std::string frame, std::string name_space,
                                  ros::Publisher neighbors_pub);

}



#endif //COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H
