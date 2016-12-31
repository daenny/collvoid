//
// Created by danielclaes on 24/06/15.
//
#include "collvoid_local_planner/publisher_helpers.h"

using namespace collvoid;


namespace collvoid {

    void publishMePosition(ROSAgent *me, std::string base_frame, std::string name_space, ros::Publisher me_pub) {
        visualization_msgs::MarkerArray sphere_list;
        sphere_list.markers.clear();
        fillMarkerWithROSAgent(sphere_list, me, base_frame, name_space);
        for (size_t i=0; i< sphere_list.markers.size(); i++) {
            sphere_list.markers[i].color.b = 1;
            sphere_list.markers[i].color.r = 0;
        }
        me_pub.publish(sphere_list);
    }


    void publishNeighborPositions(std::vector<AgentPtr> &neighbors, std::string base_frame, std::string name_space,
                                  ros::Publisher neighbors_pub) {
        visualization_msgs::MarkerArray sphere_list;
        sphere_list.markers.clear();

        for (int i = 0; i < (int) neighbors.size(); i++) {
            ROSAgentPtr agent = boost::dynamic_pointer_cast<ROSAgent>(neighbors[i]);

            fillMarkerWithROSAgent(sphere_list, agent.get(), base_frame, name_space);
        }
//        for (size_t i=0; i< sphere_list.markers.size(); i+=2) {
//            sphere_list.markers[i].color.r = 0.2 + (i / float(sphere_list.markers.size()/2.)) * 0.8;
//            sphere_list.markers[i+1].color.r = 0.2 + (i / float(sphere_list.markers.size()/2.)) * 0.8;
//            //sphere_list.markers[i].color.b = 0.2 + (i /sphere_list.markers.size()) * 0.8;
//        }
        neighbors_pub.publish(sphere_list);
    }

    void fillMarkerWithROSAgent(visualization_msgs::MarkerArray &marker, ROSAgent *agent, std::string base_frame,
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
        marker.markers[id].scale.x = 2.0 * agent->getRadius();
        marker.markers[id].scale.y = 2.0 * agent->getRadius();
        marker.markers[id].scale.z = 0.1;
        marker.markers[id].color.r = 1.0;
        marker.markers[id].color.a = 1.0;
        marker.markers[id].id = id;

        double yaw, x_dif, y_dif, th_dif, time_dif;
        time_dif = (ros::Time::now() - agent->lastSeen()).toSec();
        yaw = tf::getYaw(agent->base_odom_.pose.pose.orientation);
        //ROS_ERROR("time_dif %f", time_dif);
        //time_dif = 0.0;
        //forward projection?
        th_dif = time_dif * agent->base_odom_.twist.twist.angular.z;
        if (agent->isHoloRobot()) {
            x_dif = time_dif * agent->base_odom_.twist.twist.linear.x;
            y_dif = time_dif * agent->base_odom_.twist.twist.linear.y;
        }
        else {
            x_dif = time_dif * agent->base_odom_.twist.twist.linear.x * cos(yaw + th_dif / 2.0);
            y_dif = time_dif * agent->base_odom_.twist.twist.linear.x * sin(yaw + th_dif / 2.0);
        }
        marker.markers[id].pose.position.x = agent->base_odom_.pose.pose.position.x + x_dif;
        marker.markers[id].pose.position.y = agent->base_odom_.pose.pose.position.y + y_dif;
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
        p.x = agent->base_odom_.pose.pose.position.x + x_dif;
        p.y = agent->base_odom_.pose.pose.position.y + y_dif;
        p.z = 0.1;
        marker.markers[id].points.push_back(p);

        p.x += agent->getRadius() * 2.0 * cos(yaw + th_dif);
        p.y += agent->getRadius() * 2.0 * sin(yaw + th_dif);
        marker.markers[id].points.push_back(p);
    }

    void publishObstacleLines(const std::vector<Obstacle> &obstacles_lines, std::string base_frame,
                              std::string name_space, ros::Publisher line_pub) {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = base_frame;
        line_list.header.stamp = ros::Time::now();
        line_list.ns = name_space;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.015;
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
        line_list.id = 1;
        geometry_msgs::Point p;
        for (int i = 0; i < (int) obstacles_lines.size(); i++) {
            p.x = obstacles_lines[i].points[0].x();
            p.y = obstacles_lines[i].points[0].y();
            line_list.points.push_back(p);

            p.x = obstacles_lines[i].points[1].x();
            p.y = obstacles_lines[i].points[1].y();
            line_list.points.push_back(p);

            p.x = obstacles_lines[i].points[1].x();
            p.y = obstacles_lines[i].points[1].y();
            line_list.points.push_back(p);

            p.x = obstacles_lines[i].points[2].x();
            p.y = obstacles_lines[i].points[2].y();
            line_list.points.push_back(p);

            p.x = obstacles_lines[i].points[2].x();
            p.y = obstacles_lines[i].points[2].y();
            line_list.points.push_back(p);

            p.x = obstacles_lines[i].points[3].x();
            p.y = obstacles_lines[i].points[3].y();
            line_list.points.push_back(p);

            p.x = obstacles_lines[i].points[3].x();
            p.y = obstacles_lines[i].points[3].y();
            line_list.points.push_back(p);

            p.x = obstacles_lines[i].points[0].x();
            p.y = obstacles_lines[i].points[0].y();
            line_list.points.push_back(p);
        }
        line_pub.publish(line_list);

    }

}