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

#ifndef COLLVOID_PUBLISHERS_H
#define COLLVOID_PUBLISHERS_H

/* #include <ros/ros.h> */
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include "collvoid_local_planner/Vector2.h"
#include "collvoid_local_planner/Agent.h"

namespace collvoid {

#define MAX_POINTS_ 400

    void publishHoloSpeed(Vector2 pos, Vector2 vel, std::string target_frame, std::string name_space, ros::Publisher speed_pub);

    void publishVOs(Vector2& pos, const std::vector<VO>& truncated_vos, bool use_truncation, std::string target_frame, std::string name_space, ros::Publisher vo_pub);

    void publishPoints(Vector2& pos, const std::vector<VelocitySample>& points, std::string target_frame, std::string name_space, ros::Publisher samples_pub);
    void publishOrcaLines(const std::vector<Line>& orca_lines, Vector2& position, std::string target_frame, std::string name_space, ros::Publisher line_pub);

    void publishNeighborPositionsBare(std::vector<AgentPtr>& neighbors, std::string target_frame, std::string name_space, ros::Publisher neighbors_pub);

    void publishMePosition(double radius, tf::Stamped <tf::Pose> global_pose, std::string target_frame, std::string name_space, ros::Publisher me_pub);

    void fillMarkerWithParams(visualization_msgs::MarkerArray &marker, double radius, Vector2 position, double yaw, std::string base_frame,
                              std::string name_space);
    void publishObstacleLines(const std::vector<Obstacle>& obstacles_lines, std::string target_frame, std::string name_space, ros::Publisher line_pub);

}

#endif
