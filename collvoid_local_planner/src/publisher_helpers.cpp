//
// Created by danielclaes on 24/06/15.
//
#include "collvoid_local_planner/publisher_helpers.h"

using namespace collvoid;


namespace collvoid_scoring_function {

     void publishVOs(Vector2 &pos, const std::vector<VO> &truncated_vos, bool use_truncation, std::string base_frame,
                    std::string name_space, ros::Publisher vo_pub) {
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
        line_list.id = 0;
        if (!use_truncation) {
            for (int i = 0; i < (int) truncated_vos.size(); i++) {
                geometry_msgs::Point p;

                //Apex
                p.x = pos.x() + truncated_vos[i].point.x();
                p.y = pos.y() + truncated_vos[i].point.y();

                line_list.points.push_back(p);
                p.x = p.x + 3 * truncated_vos[i].left_leg_dir.x();
                p.y = p.y + 3 * truncated_vos[i].left_leg_dir.y();
                line_list.points.push_back(p);

                //Apex
                p.x = pos.x() + truncated_vos[i].point.x();
                p.y = pos.y() + truncated_vos[i].point.y();

                line_list.points.push_back(p);
                p.x = p.x + 3 * truncated_vos[i].right_leg_dir.x();
                p.y = p.y + 3 * truncated_vos[i].right_leg_dir.y();
                line_list.points.push_back(p);

            }
        }
        else {
            for (int i = 0; i < (int) truncated_vos.size(); i++) {
                geometry_msgs::Point p;

                p.x = pos.x() + truncated_vos[i].trunc_left.x();
                p.y = pos.y() + truncated_vos[i].trunc_left.y();

                line_list.points.push_back(p);
                p.x = p.x + 3 * truncated_vos[i].left_leg_dir.x();
                p.y = p.y + 3 * truncated_vos[i].left_leg_dir.y();
                line_list.points.push_back(p);

                p.x = pos.x() + truncated_vos[i].trunc_right.x();
                p.y = pos.y() + truncated_vos[i].trunc_right.y();

                line_list.points.push_back(p);
                p.x = p.x + 3 * truncated_vos[i].right_leg_dir.x();
                p.y = p.y + 3 * truncated_vos[i].right_leg_dir.y();
                line_list.points.push_back(p);


                p.x = pos.x() + truncated_vos[i].trunc_left.x();
                p.y = pos.y() + truncated_vos[i].trunc_left.y();
                line_list.points.push_back(p);

                p.x = pos.x() + truncated_vos[i].trunc_right.x();
                p.y = pos.y() + truncated_vos[i].trunc_right.y();
                line_list.points.push_back(p);


            }
        }
        vo_pub.publish(line_list);
    }

  void publishNeighborPositions(std::vector<AgentPtr> &neighbors, std::string frame, std::string name_space,
                                  ros::Publisher neighbors_pub) {
        visualization_msgs::MarkerArray sphere_list;
        sphere_list.markers.clear();

        BOOST_FOREACH(AgentPtr agent, neighbors){
            Vector2 position = agent->position_;
            double yaw = agent->heading_;
            double radius = agent->radius_;
            fillMarkerWithParams(sphere_list, radius, position, yaw, frame, name_space);
        }
        neighbors_pub.publish(sphere_list);
    }

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