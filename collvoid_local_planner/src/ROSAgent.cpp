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


#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <collvoid_srvs/GetObstacles.h>

#include "collvoid_local_planner/ROSAgent.h"

#include "collvoid_local_planner/orca.h"
#include "collvoid_local_planner/collvoid_publishers.h"


template<typename T>
void getParam(const ros::NodeHandle nh, const std::string &name, T *place) {
    bool found = nh.getParam(name, *place);
    ROS_ASSERT_MSG (found, "Could not find parameter %s", name.c_str());
    ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << *place);
}


template<class T>
T getParamDef(const ros::NodeHandle nh, const std::string &name, const T &default_val) {
    T val;
    nh.param(name, val, default_val);
    ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val <<
                                    "(default was " << default_val << ")");
    return val;
}

using namespace collvoid;

namespace collvoid {

    ROSAgent::ROSAgent() {
        initialized_ = false;
        cur_allowed_error_ = 0;
        min_dist_obst_ = DBL_MAX;
    }

    ROSAgent::~ROSAgent() {
        delete world_model_;
    }

    void ROSAgent::init(ros::NodeHandle private_nh, tf::TransformListener *tf, base_local_planner::LocalPlannerUtil *planner_util) {
        tf_ = tf;
        planner_util_ = planner_util;

        costmap_ = planner_util_->getCostmap();
        world_model_ = new base_local_planner::CostmapModel(*costmap_);

        //sim period
        std::string controller_frequency_param_name;
        if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
            sim_period_ = 0.05;
        else {
            double controller_frequency = 0;
            private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
            if (controller_frequency > 0)
                sim_period_ = 1.0 / controller_frequency;
            else {
                ROS_WARN(
                        "A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
                sim_period_ = 0.05;
            }
        }
        ROS_INFO("Sim period is set to %.2f", sim_period_);
        //holo_robot
        getParam(private_nh, "holo_robot", &holo_robot_);
        if (!holo_robot_)
            getParam(private_nh, "wheel_base", &wheel_base_);
        else
            wheel_base_ = 0.0;

        getParam(private_nh, "robot_radius", &fixed_robot_radius_);

        //other params agent
        time_to_holo_ = getParamDef(private_nh, "time_to_holo", 0.4);
        min_error_holo_ = getParamDef(private_nh, "min_error_holo", 0.01);
        max_error_holo_ = getParamDef(private_nh, "max_error_holo", 0.15);

        getParam(private_nh, "use_polygon_footprint", &use_polygon_footprint_);

        getParam(private_nh, "orca", &orca_);
        getParam(private_nh, "clearpath", &clearpath_);
        getParam(private_nh, "use_truncation", &use_truncation_);

        num_samples_ = getParamDef(private_nh, "num_samples", 400);
        type_vo_ = getParamDef(private_nh, "type_vo", 0); //HRVO

        trunc_time_ = getParamDef(private_nh, "trunc_time", 5.0);
        left_pref_ = getParamDef(private_nh, "left_pref", 0.1);

        private_nh.param("base_frame_id", base_frame_, std::string("base_link"));
        private_nh.param("global_frame_id", global_frame_, std::string("map"));

        getParam(private_nh, "use_obstacles", &use_obstacles_);
        controlled_ = getParamDef(private_nh, "controlled", true);
        
        ros::NodeHandle nh;
        //Publishers
        vo_pub_ = nh.advertise<visualization_msgs::Marker>("vo", 1);
        neighbors_pub_ = nh.advertise<visualization_msgs::MarkerArray>("neighbors", 1);

        lines_pub_ = nh.advertise<visualization_msgs::Marker>("orca_lines", 1);
        samples_pub_ = nh.advertise<visualization_msgs::MarkerArray>("samples", 1);
        speed_pub_ = nh.advertise<visualization_msgs::Marker>("speed", 1);
        obstacles_pub_ = nh.advertise<visualization_msgs::Marker>("obstacles", 1);

        //new obstacles from python service
        get_obstacles_srv_ = nh.serviceClient<collvoid_srvs::GetObstacles>("get_obstacles");

        //new neighbors and me services for easier handling
        get_me_srv_ = nh.serviceClient<collvoid_srvs::GetMe>("get_me");
        get_neighbors_srv_ = nh.serviceClient<collvoid_srvs::GetNeighbors>("get_neighbors");

        get_collvoid_twist_service_ = nh.advertiseService("get_collvoid_twist", &ROSAgent::getTwistServiceCB,this);
        ROS_INFO("New Agent as me initialized");
        name_space_ = std::string("collvoid");
        initialized_ = true;
    }

    bool ROSAgent::getMe() {
        collvoid_srvs::GetMe srv;
        if (get_me_srv_.call(srv)) {
            collvoid_msgs::PoseTwistWithCovariance msg = srv.response.me;
            this->radius_ = msg.radius;
            this->position_ = Vector2(msg.pose.pose.position.x, msg.pose.pose.position.y);
            this->heading_ = tf::getYaw(msg.pose.pose.orientation);

            std::vector<Vector2> minkowski_footprint;
            unrotated_footprint.clear();
            BOOST_FOREACH(geometry_msgs::Point32 p, msg.footprint.polygon.points) {
                            minkowski_footprint.push_back(Vector2(p.x, p.y));
                            geometry_msgs::Point px;
                            px.x = p.x;
                            px.y = p.y;
                            unrotated_footprint.push_back(px);
                        }
            footprint_lines_.clear();
            collvoid::Vector2 first = collvoid::Vector2(minkowski_footprint.at(0).x(), minkowski_footprint.at(0).y());
            collvoid::Vector2 old = collvoid::Vector2(minkowski_footprint.at(0).x(), minkowski_footprint.at(0).y());
            //add linesegments for footprint
            for (size_t i = 0; i < minkowski_footprint.size(); i++) {
                collvoid::Vector2 point = collvoid::Vector2(minkowski_footprint.at(i).x(), minkowski_footprint.at(i).y());
                footprint_lines_.push_back(std::make_pair(old, point));
                old = point;
            }
            //add last segment
            footprint_lines_.push_back(std::make_pair(old, first));

            this->footprint_ = rotateFootprint(minkowski_footprint, this->heading_);
            if (this->footprint_.size() < 3 && use_polygon_footprint_)
                ROS_FATAL("Me: %s, configured to use polygon footprint but size < 3, size is %d", msg.robot_id.c_str(), (int)this->footprint_.size());

            if (msg.holo_robot) {
                this->velocity_ = rotateVectorByAngle(msg.twist.twist.linear.x,
                                                      msg.twist.twist.linear.y, this->heading_);
            }
            else {
                double dif_x, dif_y, dif_ang, time_dif;
                time_dif = 0.1;
                dif_ang = time_dif * msg.twist.twist.angular.z;
                dif_x = msg.twist.twist.linear.x * cos(dif_ang / 2.0);
                dif_y = msg.twist.twist.linear.x * sin(dif_ang / 2.0);
                this->velocity_ = rotateVectorByAngle(dif_x, dif_y, this->heading_);
            }
            this->current_angular_speed_ = msg.twist.twist.angular.z;
            return true;
        }
        else {
            ROS_WARN("Could not get me");
            return false;
        }
    }

    bool ROSAgent::getNeighbors() {
        collvoid_srvs::GetNeighbors srv;
        this->agent_neighbors_.clear();
        if (get_neighbors_srv_.call(srv)) {
            BOOST_FOREACH(collvoid_msgs::PoseTwistWithCovariance msg, srv.response.neighbors) {
                            this->agent_neighbors_.push_back(createAgentFromMsg(msg));
                        }

            std::sort(this->agent_neighbors_.begin(), this->agent_neighbors_.end(),
                      boost::bind(&ROSAgent::compareNeighborsPositions, this, _1, _2));
            collvoid::publishNeighborPositionsBare(this->agent_neighbors_, "/map", "/map", neighbors_pub_);
            return true;
        }
        else {
            ROS_WARN("Could not get neighbors, is python node running?");
            return false;
        }
    }



    AgentPtr ROSAgent::createAgentFromMsg(collvoid_msgs::PoseTwistWithCovariance &msg) {
        AgentPtr agent = AgentPtr(new Agent());
        agent->radius_ = msg.radius;
        agent->controlled_ = msg.controlled;
        agent->position_ = Vector2(msg.pose.pose.position.x, msg.pose.pose.position.y);
        agent->heading_ = tf::getYaw(msg.pose.pose.orientation);

        std::vector<Vector2> minkowski_footprint;
        BOOST_FOREACH(geometry_msgs::Point32 p, msg.footprint.polygon.points) {
                        minkowski_footprint.push_back(Vector2(p.x, p.y));
                    }
        agent->footprint_ = rotateFootprint(minkowski_footprint, agent->heading_);

        if (agent->footprint_.size() < 3 && use_polygon_footprint_)
            ROS_FATAL("Agent %s, Configured to use polygon footprint but size < 3, size is %d", msg.robot_id.c_str(), (int)agent->footprint_.size());

        if (msg.holo_robot) {
            agent->velocity_ = rotateVectorByAngle(msg.twist.twist.linear.x,
                                                   msg.twist.twist.linear.y, agent->heading_);
        }
        else {
            double dif_x, dif_y, dif_ang, time_dif;
            time_dif = 0.1;
            dif_ang = time_dif * msg.twist.twist.angular.z;
            dif_x = msg.twist.twist.linear.x * cos(dif_ang / 2.0);
            dif_y = msg.twist.twist.linear.x * sin(dif_ang / 2.0);
            agent->velocity_ = rotateVectorByAngle(dif_x, dif_y, agent->heading_);
        }
        return agent;
    }

    bool ROSAgent::getTwistServiceCB(collvoid_local_planner::GetCollvoidTwist::Request &req, collvoid_local_planner::GetCollvoidTwist::Response &res) {
        tf::Stamped<tf::Pose> goal_pose, global_pose;
        tf::poseStampedMsgToTF(req.goal, goal_pose);

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.stamp_ = ros::Time();

        try{
            tf_->transformPose(global_frame_, goal_pose, global_pose);
        }
        catch(tf::TransformException& ex){
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                     goal_pose.frame_id_.c_str(), global_frame_.c_str(), ex.what());
            geometry_msgs::Twist msg;
            res.twist = msg;
            return false;
        }

        geometry_msgs::PoseStamped global_pose_msg;
        tf::poseStampedTFToMsg(global_pose, global_pose_msg);

        Vector2 goal = Vector2(global_pose_msg.pose.position.x, global_pose_msg.pose.position.y);
        double ang = tf::getYaw(global_pose_msg.pose.orientation);

        res.twist = computeVelocityCommand(goal, ang);
        return true;
    }


    geometry_msgs::Twist ROSAgent::computeVelocityCommand(Vector2 waypoint, double goal_ang) {
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return geometry_msgs::Twist();
        }
        tf::Stamped<tf::Pose> global_pose;
        //let's get the pose of the robot in the frame of the plan
        global_pose.setIdentity();
        global_pose.frame_id_ = base_frame_;
        global_pose.stamp_ = ros::Time();
        tf_->transformPose(global_frame_, global_pose, global_pose);

        Vector2 pos = Vector2(global_pose.getOrigin().x(), global_pose.getOrigin().y());
        Vector2 goal_dir = waypoint - pos;

        geometry_msgs::Twist cmd_vel;


        if (collvoid::abs(goal_dir) > planner_util_->getCurrentLimits().max_vel_x) {
            goal_dir = planner_util_->getCurrentLimits().max_vel_x * collvoid::normalize(goal_dir);
        }

        //double goal_dir_ang = atan2(goal_dir.y(), goal_dir.x());
        //ROS_INFO("Pose (%f, %f), goal (%f, %f), dir (%f, %f), ang %f", pos.x(), pos.y(), waypoint.x(), waypoint.y(), goal_dir.x(), goal_dir.y(), goal_dir_ang);


        computeNewVelocity(goal_dir, cmd_vel);
//
        if(std::abs(cmd_vel.angular.z)<planner_util_->getCurrentLimits().min_rot_vel)
            cmd_vel.angular.z = 0.0;
        if(std::abs(cmd_vel.linear.x)<planner_util_->getCurrentLimits().min_vel_x)
            cmd_vel.linear.x = 0.0;
        if(std::abs(cmd_vel.linear.y)<planner_util_->getCurrentLimits().min_vel_y)
            cmd_vel.linear.y = 0.0;

        return cmd_vel;
    }


    void ROSAgent::computeNewVelocity(Vector2 pref_velocity, geometry_msgs::Twist &cmd_vel) {
        getMe();
        getNeighbors();

        new_velocity_ = Vector2(0.0, 0.0);

        //reset
        additional_orca_lines_.clear();

        all_vos_.clear();
        static_vos_.clear();
        human_vos_.clear();
        agent_vos_.clear();

        computeObstacles();
        //get closest agent/obstacle
        double min_dist_neigh = DBL_MAX;
        if (agent_neighbors_.size() > 0) {
            min_dist_neigh = collvoid::abs(agent_neighbors_.at(0)->position_ - position_);
        }
        double min_dist = std::min(min_dist_neigh, min_dist_obst_);

        if (!holo_robot_) {
            addNHConstraints(min_dist, pref_velocity);
        }
        //add acceleration constraints
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        addAccelerationConstraintsXY(limits.max_vel_x, limits.acc_lim_x, limits.max_vel_y, limits.acc_lim_y, velocity_, heading_, sim_period_,
                                     holo_robot_, additional_orca_lines_);

        if (orca_) {
            computeOrcaVelocity(pref_velocity);
        }
        else {
            samples_.clear();
            if (clearpath_) {
                computeClearpathVelocity(pref_velocity);
            }
            else {
                computeSampledVelocity(pref_velocity);
            }
        }

        double speed_ang = atan2(new_velocity_.y(), new_velocity_.x());
        double dif_ang = angles::shortest_angular_distance(heading_, speed_ang);

        if (!holo_robot_) {
            double vel = collvoid::abs(new_velocity_);
            double vstar;

            if (std::abs(dif_ang) > EPSILON)
                vstar = calcVstar(vel, dif_ang);
            else
                vstar = limits.max_vel_x;

            cmd_vel.linear.x = std::min(vstar, vMaxAng());
            cmd_vel.linear.y = 0.0;

            //ROS_ERROR("dif_ang %f", dif_ang);
            if (std::abs(dif_ang) > 3.0 * M_PI / 4.0) {
                if (last_twist_ang_ != 0.0)
                {
                    cmd_vel.angular.z = sign(last_twist_ang_) *
                                        std::min(std::abs(dif_ang / time_to_holo_), limits.max_rot_vel);
                }
                else {
                    cmd_vel.angular.z = sign(dif_ang) *
                                        std::min(std::abs(dif_ang / time_to_holo_), limits.max_rot_vel);

                }
//                ROS_ERROR("dif_ang %f", dif_ang);
//                ROS_ERROR("base twist %f", base_odom_.twist.twist.angular.z);
//                ROS_ERROR("cmd_vel %f", cmd_vel.angular.z);
//                ROS_ERROR("last_cmd_vel %f", last_twist_ang_);
                last_twist_ang_ = cmd_vel.angular.z;
            }
            else {
                cmd_vel.angular.z = sign(dif_ang) * std::min(std::abs(dif_ang / time_to_holo_), limits.max_rot_vel);
                last_twist_ang_ = 0.;
            }
            //ROS_ERROR("vstar = %.3f", vstar);
            // if (std::abs(cmd_vel.angular.z) == max_vel_th_)
            // 	cmd_vel.linear.x = 0;


        }
        else {
            collvoid::Vector2 rotated_vel = rotateVectorByAngle(new_velocity_.x(), new_velocity_.y(), -heading_);

            cmd_vel.linear.x = rotated_vel.x();
            cmd_vel.linear.y = rotated_vel.y();
            // if (min_dist < 1.0/2.0 * radius_) {
            // 	if (min_dist == min_dist_neigh) {
            // 	  dif_ang = dif_ang;
            // 	}
            // 	else {
            // 	  double ang_obst = atan2(min_obst_vec.y(), min_obst_vec.x());
            // 	  double diff = angles::shortest_angular_distance(heading_, ang_obst);

            // 	  dif_ang = angles::shortest_angular_distance(0.0, diff - sign(diff) * M_PI / 2.0);
            // 	}
            // }
            //      if (std::abs(dif_ang) < M_PI/2.0)
            if (min_dist > 2 * fixed_robot_radius_)
                cmd_vel.angular.z = sign(dif_ang) * std::min(std::abs(dif_ang), limits.max_rot_vel);
        }

    }


    void ROSAgent::computeClearpathVelocity(Vector2 pref_velocity) {
        boost::mutex::scoped_lock lock(neighbors_lock_);

        //account for nh error
        radius_ += cur_allowed_error_;
        ((Agent *) this)->computeClearpathVelocity(pref_velocity);
        radius_ -= cur_allowed_error_;

        publishHoloSpeed(position_, new_velocity_, global_frame_, name_space_, speed_pub_);
        publishVOs(position_, all_vos_, use_truncation_, global_frame_, name_space_, vo_pub_);

        publishPoints(position_, samples_, global_frame_, name_space_, samples_pub_);
        publishOrcaLines(additional_orca_lines_, position_, global_frame_, name_space_, lines_pub_);

    }


    void ROSAgent::computeSampledVelocity(Vector2 pref_velocity) {
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        createSamplesWithinMovementConstraints(samples_, this->velocity_.x(),
                                               this->velocity_.y(), this->current_angular_speed_,
                                               limits,
                                               heading_, pref_velocity,
                                               sim_period_, num_samples_, holo_robot_);

        //account for nh error
        boost::mutex::scoped_lock lock(neighbors_lock_);

        //tf::Stamped<tf::Pose> global_pose;
        //odom_pose_ = Vector2(global_pose.getOrigin().x(), global_pose.getOrigin().y());
        radius_ += cur_allowed_error_;
        ((Agent *) this)->computeSampledVelocity(pref_velocity);
        radius_ -= cur_allowed_error_;

        //    Vector2 null_vec = Vector2(0,0);
        publishHoloSpeed(position_, new_velocity_, global_frame_, name_space_, speed_pub_);
        publishVOs(position_, all_vos_, use_truncation_, global_frame_, name_space_, vo_pub_);
        publishPoints(position_, samples_, global_frame_, name_space_, samples_pub_);
        publishOrcaLines(additional_orca_lines_, position_, global_frame_, name_space_, lines_pub_);

    }


    void ROSAgent::computeOrcaVelocity(Vector2 pref_velocity) {
        ((Agent *) this)->computeOrcaVelocity(pref_velocity, use_polygon_footprint_);

        // publish calcuated speed and orca_lines
        publishHoloSpeed(position_, new_velocity_, global_frame_, name_space_, speed_pub_);
        publishOrcaLines(orca_lines_, position_, global_frame_, name_space_, lines_pub_);

    }

    void ROSAgent::addNHConstraints(double min_dist, Vector2 pref_velocity) {
        double min_error = min_error_holo_;
        double max_error = max_error_holo_;
        double error = max_error;
        double v_max_ang = vMaxAng();

        //ROS_ERROR("v_max_ang %.2f", v_max_ang);

        if (min_dist < 2.0 * radius_) {
            error = (max_error - min_error) / (collvoid::sqr(2 * (radius_))) *
                    collvoid::sqr(min_dist) + min_error; // how much error do i allow?
            //ROS_DEBUG("Error = %f", error);
            if (min_dist < 0) {
                error = min_error;
                // ROS_DEBUG("%s I think I am in collision", me_->getId().c_str());
            }
        }
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        cur_allowed_error_ = 1.0 / 3.0 * cur_allowed_error_ + 2.0 / 3.0 * error;
        //ROS_ERROR("error = %f", cur_allowed_error_);
        double speed_ang = atan2(pref_velocity.y(), pref_velocity.x());
        double dif_ang = angles::shortest_angular_distance(heading_, speed_ang);
        if (std::abs(dif_ang) > M_PI / 2.0) { // || cur_allowed_error_ < 2.0 * min_error) {
            double max_track_speed = calculateMaxTrackSpeedAngle(time_to_holo_, M_PI / 2.0, cur_allowed_error_,
                                                                 limits.max_vel_x, limits.max_rot_vel, v_max_ang);
            if (max_track_speed <= 2 * min_error) {
                max_track_speed = 2 * min_error;
            }
            //ROS_INFO("Max Track speed %f", max_track_speed);
            addMovementConstraintsDiffSimple(max_track_speed, heading_, additional_orca_lines_);
        }
        else {
            addMovementConstraintsDiff(cur_allowed_error_, time_to_holo_, limits.max_vel_x, limits.max_rot_vel, heading_, v_max_ang,
                                       additional_orca_lines_);
        }
        max_speed_x_ = vMaxAng();

    }


    std::vector<Obstacle> ROSAgent::getObstacles() {
        std::vector<Obstacle> obstacles;
        collvoid_srvs::GetObstacles srv;
        if (get_obstacles_srv_.call(srv)) {
            sensor_msgs::PointCloud in, result;

            BOOST_FOREACH(geometry_msgs::PolygonStamped poly, srv.response.obstacles) {
                            Obstacle obst;
                            in.header.stamp = poly.header.stamp;
                            in.header.frame_id = poly.header.frame_id;
                            obst.last_seen = poly.header.stamp;
                            //geometry_msgs::PointStamped in;
                            in.points = poly.polygon.points;
                            try {
                                tf_->waitForTransform(global_frame_, in.header.frame_id, in.header.stamp,
                                                      ros::Duration(0.1));
                                tf_->transformPointCloud(global_frame_, in, result);
                            }
                            catch(tf::TransformException& ex) {
                                ROS_WARN(
                                        "Failed to transform the goal pose from %s into the %s frame: %s",
                                        in.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
                                continue;

                            }
                            BOOST_FOREACH (geometry_msgs::Point32 p, result.points) {
                                            Vector2 v = Vector2(p.x, p.y);
                                            obst.points.push_back(v);

                                        }
                            obstacles.push_back(obst);
                        }
        }
        else {
            ROS_WARN("Could not get Obstacles from service");
        }
        publishObstacleLines(obstacles, global_frame_, name_space_, obstacles_pub_);

        return obstacles;
    }

    void ROSAgent::computeObstacles() {
        if (use_obstacles_) {
            boost::mutex::scoped_lock lock(obstacle_lock_);

            std::vector<Obstacle> obstacles = getObstacles();
            min_dist_obst_ = DBL_MAX;
            BOOST_FOREACH(Obstacle obst, obstacles) {
                            //obst.points = rotateFootprint(obst.points, heading_);
                            Vector2 obst_center = (obst.points[0] + obst.points[2])/2.;
                            double dist = distSqPointLineSegment(obst.points[0], obst.points[1], position_);
                            dist = std::min(dist, distSqPointLineSegment(obst.points[1], obst.points[2], position_));
                            dist = std::min(dist, distSqPointLineSegment(obst.points[2], obst.points[3], position_));
                            dist = std::min(dist, distSqPointLineSegment(obst.points[0], obst.points[3], position_));

                            if (dist < min_dist_obst_) {
                                min_dist_obst_ = dist;
                            }
                            std::vector<Vector2> obst_footprint;
                            BOOST_FOREACH(Vector2 p, obst.points) {
                                            obst_footprint.push_back(p-obst_center);
                                        }
                            if (orca_) {

                                createObstacleLine(footprint_, obst.points[0], obst.points[1]);
                                createObstacleLine(footprint_, obst.points[1], obst.points[2]);
                                createObstacleLine(footprint_, obst.points[2], obst.points[3]);
                                createObstacleLine(footprint_, obst.points[3], obst.points[0]);
                            }
                            else {
                                Vector2 null = Vector2(0,0);
                                VO obstacle_vo = createVO(position_, footprint_, obst_center, obst_footprint, null);
                                obstacle_vo = createTruncVO(obstacle_vo, 2);
                                static_vos_.push_back(obstacle_vo);
                                all_vos_.push_back(obstacle_vo);
                            }
                        }
        }

    }


    bool ROSAgent::compareNeighborsPositions(const AgentPtr &agent1, const AgentPtr &agent2) {
        return compareVectorPosition(agent1->position_, agent2->position_);
    }

    bool ROSAgent::compareVectorPosition(const collvoid::Vector2 &v1, const collvoid::Vector2 &v2) {
        return collvoid::absSqr(position_ - v1) <= collvoid::absSqr(position_ - v2);
    }

    void ROSAgent::sortObstacleLines() {
        boost::mutex::scoped_lock lock(obstacle_lock_);
        std::sort(obstacle_points_.begin(), obstacle_points_.end(),
                  boost::bind(&ROSAgent::compareVectorPosition, this, _1, _2));
    }

    collvoid::Vector2 ROSAgent::LineSegmentToLineSegmentIntersection(double x1, double y1, double x2, double y2,
                                                                     double x3, double y3, double x4, double y4) {
        double r, s, d;
        collvoid::Vector2 res;
        //Make sure the lines aren't parallel
        if ((y2 - y1) / (x2 - x1) != (y4 - y3) / (x4 - x3)) {
            d = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));
            if (d != 0) {
                r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / d;
                s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / d;
                if (r >= 0 && r <= 1) {
                    if (s >= 0 && s <= 1) {
                        return collvoid::Vector2(x1 + r * (x2 - x1), y1 + r * (y2 - y1));
                    }
                }
            }
        }
        return res;
    }


    double ROSAgent::getDistToFootprint(collvoid::Vector2 &point) {
        collvoid::Vector2 result, null;
        for (size_t i = 0; i < footprint_lines_.size(); i++) {
            collvoid::Vector2 first = footprint_lines_[i].first;
            collvoid::Vector2 second = footprint_lines_[i].second;

            result = LineSegmentToLineSegmentIntersection(first.x(), first.y(), second.x(), second.y(), 0.0, 0.0,
                                                          point.x(), point.y());
            if (result != null) {
                //ROS_DEBUG("Result = %f, %f, dist %f", result.x(), result.y(), collvoid::abs(result));
                return collvoid::abs(result);
            }
        }
        ROS_DEBUG("Obstacle Point within Footprint. I am close to/in collision");
        return -1;
    }

    void ROSAgent::computeObstacleLine(Vector2 &obst) {
        Line line;
        Vector2 relative_position = obst - position_;
        double dist_to_footprint;
        double dist = collvoid::abs(position_ - obst);
        if (!use_polygon_footprint_)
            dist_to_footprint = fixed_robot_radius_;
        else {
            dist_to_footprint = getDistToFootprint(relative_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = fixed_robot_radius_;
            }
        }
        dist = dist - dist_to_footprint - 0.03;

        line.point = normalize(relative_position) * (dist);
        line.dir = Vector2(-normalize(relative_position).y(), normalize(relative_position).x());
        additional_orca_lines_.push_back(line);
    }

    void ROSAgent::createObstacleLine(std::vector<Vector2> &own_footprint, Vector2 &obst1, Vector2 &obst2) {
        Vector2 null = Vector2(0,0);
        double dist = distSqPointLineSegment(obst1, obst2, null);

        if (dist == absSqr(obst1)) {
            computeObstacleLine(obst1);
        }
        else if (dist == absSqr(obst2)) {
            computeObstacleLine(obst2);
        }
            // if (false) {
            // }
        else {
            Vector2 position_obst = projectPointOnLine(obst1, obst2 - obst1, null);
            Vector2 rel_position = position_obst;
            dist = std::sqrt(dist);
            double dist_to_footprint = getDistToFootprint(rel_position);
            if (dist_to_footprint == -1) {
                dist_to_footprint = fixed_robot_radius_;
            }
            dist = dist - dist_to_footprint - 0.03;

            if (dist < 0.0) {
                Line line;
                line.point = (dist - 0.02) * normalize(rel_position);
                line.dir = normalize(obst1 - obst2);
                additional_orca_lines_.push_back(line);
                return;
            }

            if (abs(position_ - obst1) > 2 * fixed_robot_radius_ && abs(position_ - obst2) > 2 * fixed_robot_radius_) {
                Line line;
                line.point = dist * normalize(rel_position);
                line.dir = -normalize(obst1 - obst2);
                additional_orca_lines_.push_back(line);
                return;

            }
            // return;
            rel_position = (abs(rel_position) - dist / 2.0) * normalize(rel_position);

            std::vector<Vector2> obst;
            obst.push_back(obst1 - position_obst);
            obst.push_back(obst2 - position_obst);
            std::vector<Vector2> mink_sum = minkowskiSum(own_footprint, obst);

            Vector2 min, max;
            double min_ang = 0.0;
            double max_ang = 0.0;

            for (int i = 0; i < (int) mink_sum.size(); i++) {
                double angle = angleBetween(rel_position, rel_position + mink_sum[i]);
                if (leftOf(Vector2(0.0, 0.0), rel_position, rel_position + mink_sum[i])) {
                    if (-angle < min_ang) {
                        min = rel_position + mink_sum[i];
                        min_ang = -angle;
                    }
                }
                else {
                    if (angle > max_ang) {
                        max = rel_position + mink_sum[i];
                        max_ang = angle;
                    }
                }
            }

            Line line;
            line.point = (dist / 2.0) * normalize(rel_position);
            if (absSqr(position_obst - obst1) > absSqr(position_obst - obst2)) {
                // ROS_ERROR("max_ang = %.2f", max_ang);
                line.dir = rotateVectorByAngle(normalize(max), 0.1);
            }
            else {
                // ROS_ERROR("min_ang = %.2f", min_ang);
                line.dir = rotateVectorByAngle(normalize(min), 0.1);;
            }
            additional_orca_lines_.push_back(line);

        }
    }

    void ROSAgent::reconfigure(collvoid_local_planner::CollvoidConfig &cfg){

        this->max_speed_x_ = planner_util_->getCurrentLimits().max_vel_x;
    }



    double ROSAgent::vMaxAng() {
        double theoretical_max_v = planner_util_->getCurrentLimits().max_rot_vel * wheel_base_ / 2.0;
        //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
        return planner_util_->getCurrentLimits().max_vel_x; //TODO: fixme!!
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ROSAgent");
    //ros::NodeHandle nh;
    ros::NodeHandle nh("~");

    ROSAgentPtr me(new ROSAgent);
    tf::TransformListener tf;

    //me->init(nh, &tf);
    ROS_INFO("ROSAgent initialized");
    ros::spin();

}


