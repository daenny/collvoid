//
// Created by danielclaes on 23/06/15.
//

#ifndef COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H
#define COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include "collvoid_local_planner/Vector2.h"


namespace collvoid {
    void fillMarkerWithParams(visualization_msgs::MarkerArray &marker, double radius, Vector2 position, double yaw, std::string base_frame,
                                std::string name_space);
    void publishMePosition(double radius, tf::Stamped <tf::Pose> global_pose, std::string base_frame,
                           std::string name_space, ros::Publisher me_pub);


    void publishMePosition(double radius, tf::Stamped <tf::Pose> global_pose, std::string base_frame,
                           std::string name_space, ros::Publisher me_pub) {
        visualization_msgs::MarkerArray sphere_list;
        sphere_list.markers.clear();
        Vector2 position = Vector2(global_pose.getOrigin().x(), global_pose.getOrigin().y());
        double yaw = tf::getYaw(global_pose.getRotation());
        fillMarkerWithParams(sphere_list, radius, position, yaw, base_frame, name_space);
        me_pub.publish(sphere_list);
    }

    void fillMarkerWithParams(visualization_msgs::MarkerArray &marker, double radius, Vector2 position, double yaw, std::string base_frame,
                                std::string name_space) {
        int id = (int) marker.markers.size();
        marker.markers.resize(marker.markers.size() + 2);
        ros::Time timestamp = ros::Time::now();

        marker.markers[id].header.frame_id = base_frame;
        marker.markers[id].header.stamp = ros::Time::now();
        marker.markers[id].ns = name_space;
        marker.markers[id].action = visualization_msgs::Marker::ADD;
        marker.markers[id].pose.orientation.w = 1.0;
        marker.markers[id].type = visualization_msgs::Marker::SPHERE;
        marker.markers[id].scale.x = 2.0 * radius;
        marker.markers[id].scale.y = 2.0 * radius;
        marker.markers[id].scale.z = 0.1;
        marker.markers[id].color.r = 1.0;
        marker.markers[id].color.a = 1.0;
        marker.markers[id].id = id;

        marker.markers[id].pose.position.x = position.x();
        marker.markers[id].pose.position.y = position.y();
        marker.markers[id].pose.position.z = 0.2;

        id = id + 1;
        marker.markers[id].header.frame_id = base_frame;
        marker.markers[id].header.stamp = ros::Time::now();
        marker.markers[id].ns = name_space;
        marker.markers[id].action = visualization_msgs::Marker::ADD;
        marker.markers[id].pose.orientation.w = 1.0;//tf::createQuaternionMsgFromYaw(yaw+th_dif);
        marker.markers[id].type = visualization_msgs::Marker::ARROW;
        marker.markers[id].scale.x = 0.1;
        marker.markers[id].scale.y = 0.2;
        marker.markers[id].scale.z = 0.1;
        marker.markers[id].color.r = 1.0;
        marker.markers[id].color.a = 1.0;
        marker.markers[id].id = id;

        geometry_msgs::Point p;
        p.x = position.x();
        p.y = position.y();
        p.z = 0.1;
        marker.markers[id].points.push_back(p);

        p.x += radius * 2.0 * cos(yaw);
        p.y += radius * 2.0 * sin(yaw);
        marker.markers[id].points.push_back(p);
    }
}



#endif //COLLVOID_LOCAL_PLANNER_PUBLISHER_HELPERS_H
