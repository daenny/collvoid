/*
 * Copyright (c) 2012, Daniel Claes, Maastricht University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Maastricht University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



#include "collvoid_local_planner/collvoid_publishers.h"

namespace collvoid {

    void publishHoloSpeed(Vector2 pos, Vector2 vel, std::string target_frame, std::string name_space,
                          ros::Publisher speed_pub) {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = target_frame;
        line_list.header.stamp = ros::Time::now();
        line_list.ns = name_space;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.02;
        line_list.color.r = 0.0;
        line_list.color.b = 1.0;
        line_list.color.a = 1.0;
        line_list.id = 0;
        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = 0.2;
        line_list.points.push_back(p);
        p.x = p.x + vel.x();
        p.y = p.y + vel.y();
        line_list.points.push_back(p);

        speed_pub.publish(line_list);
    }


    void publishVOs(Vector2 &pos, const std::vector<VO> &truncated_vos, bool use_truncation, std::string target_frame,
                    std::string name_space, ros::Publisher vo_pub) {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = target_frame;
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


    void publishPoints(Vector2 &pos, const std::vector<VelocitySample> &points, std::string target_frame,
                       std::string name_space, ros::Publisher samples_pub) {
        visualization_msgs::MarkerArray point_array;
        // point_array.markers.resize(MAX_POINTS);
        // point_array.markers.clear();
        visualization_msgs::Marker sphere;

        sphere.header.frame_id = target_frame;
        sphere.header.stamp = ros::Time::now();
        sphere.ns = name_space;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose.orientation.w = 1.0;
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        sphere.scale.x = 0.1;
        sphere.scale.y = 0.1;
        sphere.scale.z = 0.1;
        sphere.color.r = 0.5;
        sphere.color.a = 1.0;
        sphere.id = 0;
        for (int i = 0; i < (int) points.size(); i++) {

            geometry_msgs::Point p;
            p.x = pos.x() + points[i].velocity.x();
            p.y = pos.y() + points[i].velocity.y();
            p.z = points[i].cost;
            //p.z = 0.1;
            sphere.points.push_back(p);
        }
        point_array.markers.push_back(sphere);


        samples_pub.publish(point_array);
    }


    void publishOrcaLines(const std::vector<Line> &orca_lines, Vector2 &position, std::string target_frame,
                          std::string name_space, ros::Publisher line_pub) {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = target_frame;
        line_list.header.stamp = ros::Time::now();
        line_list.ns = name_space;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.015;
        line_list.color.g = 1.0;
        line_list.color.a = 1.0;
        line_list.id = 1;
        line_list.lifetime = ros::Duration(1);
        geometry_msgs::Point p;
        for (int i = 0; i < (int) orca_lines.size(); i++) {
            p.x = position.x() + orca_lines[i].point.x() - orca_lines[i].dir.x();
            p.y = position.y() + orca_lines[i].point.y() - orca_lines[i].dir.y();

            line_list.points.push_back(p);
            p.x = p.x + 3 * orca_lines[i].dir.x();
            p.y = p.y + 3 * orca_lines[i].dir.y();
            line_list.points.push_back(p);

        }
        line_pub.publish(line_list);

    }


    void publishMePosition(double radius, tf::Stamped <tf::Pose> global_pose, std::string target_frame, std::string name_space, ros::Publisher me_pub){
        visualization_msgs::MarkerArray sphere_list;
        sphere_list.markers.clear();
        Vector2 position = Vector2(global_pose.getOrigin().x(), global_pose.getOrigin().y());
        double yaw = tf::getYaw(global_pose.getRotation());
        fillMarkerWithParams(sphere_list, radius, position, yaw, target_frame, name_space);
        me_pub.publish(sphere_list);
    }


    void publishNeighborPositionsBare(std::vector<AgentPtr> &neighbors, std::string target_frame, std::string name_space,
                                  ros::Publisher neighbors_pub) {
        visualization_msgs::MarkerArray sphere_list;
        sphere_list.markers.clear();

        for(AgentPtr agent: neighbors){
                        Vector2 position = agent->position_;
                        double yaw = agent->heading_;
                        double radius = agent->radius_;
                        fillMarkerWithParams(sphere_list, radius, position, yaw, target_frame, name_space);
                    }
        neighbors_pub.publish(sphere_list);
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
        marker.markers[id].lifetime = ros::Duration(1);

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
        marker.markers[id].lifetime = ros::Duration(1);

        geometry_msgs::Point p;
        p.x = position.x();
        p.y = position.y();
        p.z = 0.1;
        marker.markers[id].points.push_back(p);

        p.x += radius * 2.0 * cos(yaw);
        p.y += radius * 2.0 * sin(yaw);
        marker.markers[id].points.push_back(p);
    }


    void publishObstacleLines(const std::vector<Obstacle> &obstacles_lines, std::string target_frame,
                              std::string name_space, ros::Publisher line_pub) {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = target_frame;
        line_list.header.stamp = ros::Time::now();
        line_list.ns = name_space;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.015;
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
        line_list.id = 1;
        line_list.lifetime = ros::Duration(1);
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
