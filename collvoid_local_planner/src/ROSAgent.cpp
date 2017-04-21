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
#include <boost/tuple/tuple.hpp>
#include <collvoid_srvs/GetObstacles.h>
#include <base_local_planner/trajectory.h>

#include "collvoid_local_planner/ROSAgent.h"

#include "collvoid_local_planner/orca.h"
#include "collvoid_local_planner/collvoid_publishers.h"


template<typename T>
void getParam(const ros::NodeHandle nh, const std::string &name, T *place) {
    bool found = nh.getParam(name, *place);
    ROS_ASSERT_MSG (found, "Could not find parameter %s", name.c_str());
    ROS_DEBUG_STREAM_NAMED ("init", std::string("Initialized ") << name << " to " << *place);
}


template<class T>
T getParamDef(const ros::NodeHandle nh, const std::string &name, const T &default_val) {
    T val;
    nh.param(name, val, default_val);
    ROS_DEBUG_STREAM_NAMED ("init", std::string("Initialized ") << name << " to " << val <<
                                                                "(default was " << default_val << ")");
    return val;
}

using namespace collvoid;

namespace collvoid {


    void ROSAgent::reconfigure(collvoid_local_planner::CollvoidConfig &config){
        boost::mutex::scoped_lock l(configuration_mutex_);

        generator_.setParameters(config.sim_time,
                                 config.sim_granularity,
                                 config.angular_sim_granularity,
                                 config.use_dwa,
                                 sim_period_);

        double resolution = planner_util_->getCostmap()->getResolution();
        pdist_scale_ = config.path_distance_bias;
        // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
        path_costs_->setScale(resolution * pdist_scale_ * 0.5);
        alignment_costs_->setScale(resolution * pdist_scale_ * 0.5);


        goal_costs_->setScale(resolution * config.goal_distance_bias * 0.5);
        goal_front_costs_->setScale(resolution * config.goal_distance_bias * 0.5);


        obstacle_costs_->setScale(resolution * config.occdist_scale);


        forward_point_distance_ = config.forward_point_distance;
        goal_front_costs_->setXShift(forward_point_distance_);
        alignment_costs_->setXShift(forward_point_distance_);
        goal_heading_sq_dist_ = config.goal_sq_dist;


        // obstacle costs can vary due to scaling footprint feature
        obstacle_costs_->setParams(config.max_trans_vel, 0, 0);

        //collvoid_
        orca_ = config.orca;
        use_polygon_footprint_ = config.convex;
        use_truncation_ = config.use_truncation;
        num_samples_ = config.num_samples;
        type_vo_ = config.type_vo;

        time_to_holo_ = config.time_to_holo;
        min_error_holo_ = config.min_error_holo;
        max_error_holo_ = config.max_error_holo;

        trunc_time_ = config.trunc_time;
        left_pref_ = config.left_pref;

        use_dwa_score_ = config.use_dwa_scoring;
    }


    ROSAgent::ROSAgent():
            initialized_ (false),
            min_dist_obst_ (DBL_MAX)
    {
        cur_allowed_error_ = 0;
    }

    ROSAgent::~ROSAgent() {
        delete obstacle_costs_;
        delete path_costs_;
        delete goal_costs_;
        delete goal_front_costs_;
        delete alignment_costs_;
    }

    void ROSAgent::init(ros::NodeHandle private_nh, tf::TransformListener *tf, base_local_planner::LocalPlannerUtil *planner_util,
                        costmap_2d::Costmap2DROS* costmap_ros) {
        tf_ = tf;
        planner_util_ = planner_util;
        costmap_ros_ = costmap_ros;

        bool sum_scores;
        private_nh.param("sum_scores", sum_scores, false);

        obstacle_costs_ = new base_local_planner::ObstacleCostFunction (planner_util->getCostmap());
        path_costs_ = new base_local_planner::MapGridCostFunction(planner_util->getCostmap(), 0.0, 0.0, false);
        goal_costs_ = new base_local_planner::MapGridCostFunction(planner_util->getCostmap(), 0.0, 0.0, true);
        goal_front_costs_ = new base_local_planner::MapGridCostFunction(planner_util->getCostmap(), 0.0, 0.0, true);
        alignment_costs_ = new base_local_planner::MapGridCostFunction(planner_util->getCostmap(), 0.0, 0.0, false);

        obstacle_costs_->setSumScores(sum_scores);

        //path_costs_->setScale(0);
        goal_costs_->setScale(0);
        goal_front_costs_->setScale(0);
        alignment_costs_->setScale(0);

        //obstacle check
        critics_.push_back((base_local_planner::TrajectoryCostFunction *&&) obstacle_costs_); // discards trajectories that move into obstacles
        //critics_.push_back((base_local_planner::TrajectoryCostFunction *&&) goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal

        //critics_.push_back((base_local_planner::TrajectoryCostFunction *&&) alignment_costs_); // prefers trajectories that keep the robot nose on nose path
        critics_.push_back((base_local_planner::TrajectoryCostFunction *&&) path_costs_); // prefers trajectories on global path
        //critics_.push_back((base_local_planner::TrajectoryCostFunction *&&) goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
        std::vector<base_local_planner::TrajectorySampleGenerator *> generator_list;
        generator_list.push_back(&generator_);
        scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics_);

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
        use_dwa_score_ = getParamDef(private_nh, "use_dwa_scoring", true);
        new_sampling_ = getParamDef(private_nh, "new_sampling", true);

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
        neighbors_pub_ = nh.advertise<visualization_msgs::MarkerArray>("neighbors", 1, true);

        lines_pub_ = nh.advertise<visualization_msgs::Marker>("orca_lines", 1, true);
        samples_pub_ = nh.advertise<visualization_msgs::MarkerArray>("samples", 1, true);
        speed_pub_ = nh.advertise<visualization_msgs::Marker>("speed", 1, true);
        obstacles_pub_ = nh.advertise<visualization_msgs::Marker>("obstacles", 1, true);

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
            for(geometry_msgs::Point32 p: msg.footprint.polygon.points) {
                minkowski_footprint.push_back(Vector2(p.x, p.y));
                geometry_msgs::Point px;

            }
            if (minkowski_footprint.size()<3 && this->use_polygon_footprint_) {
                ROS_FATAL("Me: %s, configured to use polygon footprint but size < 3, size is %d", msg.robot_id.c_str(), (int)minkowski_footprint.size());
                return false;
            }
            footprint_lines_.clear();
            collvoid::Vector2 first = collvoid::Vector2(minkowski_footprint.at(0).x(),
                                                        minkowski_footprint.at(0).y());
            collvoid::Vector2 old = collvoid::Vector2(minkowski_footprint.at(0).x(), minkowski_footprint.at(0).y());
            //add linesegments for footprint
            for (size_t i = 0; i < minkowski_footprint.size(); i++) {
                collvoid::Vector2 point = collvoid::Vector2(minkowski_footprint.at(i).x(),
                                                            minkowski_footprint.at(i).y());
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
            for(collvoid_msgs::PoseTwistWithCovariance msg: srv.response.neighbors) {
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
        for(geometry_msgs::Point32 p: msg.footprint.polygon.points) {
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
        if (!getMe()) {
            ROS_WARN("Could not get me from service");
            return;
        }
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
            boost::mutex::scoped_lock lock(computing_lock_);
            samples_.clear();
            safe_samples_.clear();
            obstacle_costs_->setFootprint(costmap_ros_->getRobotFootprint());

            if (use_dwa_score_) {
                for (std::vector<base_local_planner::TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
                    base_local_planner::TrajectoryCostFunction* loop_critic_p = *loop_critic;
                    if (loop_critic_p->prepare() == false) {
                        ROS_WARN("A scoring function failed to prepare");
                    }
                }
                geometry_msgs::PoseStamped goal_pose = global_plan_.back();
                Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
                Eigen::Vector3f vsamples (1,1,1);
                generator_.initialise(pos_,
                                      vel_,
                                      goal,
                                      &limits,
                                      vsamples);
            }

            //account for nh error
            radius_ += cur_allowed_error_;
            if (controlled_) {
                computeAgentVOs();
            }
            computeHumanVOs();

            if (clearpath_) {
                createClearpathSamples(samples_, all_vos_, human_vos_, agent_vos_, static_vos_, additional_orca_lines_,
                                       pref_velocity, velocity_, planner_util_->getCurrentLimits().max_vel_x, use_truncation_);
                computeClearpathVelocity(pref_velocity);
            }
            else {
                createSamplesWithinMovementConstraints(samples_, this->velocity_.x(),
                                                       this->velocity_.y(), this->current_angular_speed_,
                                                       limits,
                                                       heading_, pref_velocity,
                                                       sim_period_, num_samples_, holo_robot_);

                computeSampledVelocity(pref_velocity);
            }
            radius_ -= cur_allowed_error_;

            publishHoloSpeed(position_, new_velocity_, global_frame_, name_space_, speed_pub_);
            publishVOs(position_, all_vos_, use_truncation_, global_frame_, name_space_, vo_pub_);
            if (new_sampling_ && safe_samples_.size() > 0)
                publishPoints(position_, safe_samples_, global_frame_, name_space_, samples_pub_);
            else
                publishPoints(position_, samples_, global_frame_, name_space_, samples_pub_);
            publishOrcaLines(additional_orca_lines_, position_, global_frame_, name_space_, lines_pub_);

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

            cmd_vel.linear.x = std::max(std::min(vstar, vMaxAng()), limits.min_vel_x);
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
                if (std::abs(dif_ang) > 4.0 * M_PI / 4.0) {
                    cmd_vel.linear.x = 0;
                }
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

    void ROSAgent::computeSampledVelocity(Vector2 pref_vel) {

        // VelocitySample pref_vel_sample;
        // pref_vel_sample.velocity = pref_vel;
        // samples_.push_back(pref_vel_sample);

        // VelocitySample null_vel;
        // null_vel.velocity = Vector2(0,0);
        // null_vel.dist_to_pref_vel = absSqr(pref_vel);
        // samples_.push_back(null_vel);

        double min_cost = DBL_MAX;
        Vector2 best_vel;

        double vel_x, vel_y;
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

        for (int i = 0; i < (int) samples_.size(); i++) {

            VelocitySample cur = samples_[i];
            double cost = calculateVelCosts(cur.velocity, all_vos_, use_truncation_);
            cost += 2 * absSqr(cur.velocity - pref_vel);
            //cost += std::min(-minDistToVOs(truncated_vos, cur.velocity, use_truncation),0.1);
            cost += -minDistToVOs(all_vos_, cur.velocity, use_truncation_);
            cost += 2 * absSqr(cur.velocity - velocity_);
            double footprint_cost = 0;
            if (use_dwa_score_) {

                Eigen::Vector3f twist = createTwistFromVector(samples_[i].velocity, limits);
                footprint_cost = scoreTrajectory(twist);

                //ROS_ERROR("footprint_cost %f", footprint_cost);
                if (footprint_cost < 0.) {
                    samples_[i].cost = 100;
                }
                samples_[i].cost = cost + footprint_cost;
            }

            if (cost < min_cost) {
                min_cost = cost;
                best_vel = cur.velocity;
            }

        }
        //ROS_ERROR("min_cost %f", min_cost);
        new_velocity_ = best_vel;

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
    }


    std::vector<Obstacle> ROSAgent::getObstacles() {
        std::vector<Obstacle> obstacles;
        collvoid_srvs::GetObstacles srv;
        if (get_obstacles_srv_.call(srv)) {
            //sensor_msgs::PointCloud in, result;
            tf::StampedTransform obst_to_global_transform;
            if (srv.response.obstacles.size() > 0) {
                try {
                    tf_->waitForTransform(global_frame_, srv.response.obstacles.at(0).header.frame_id,
                                          srv.response.obstacles.at(0).header.stamp,
                                          ros::Duration(0.2));
                    tf_->lookupTransform(global_frame_, srv.response.obstacles.at(0).header.frame_id,
                                         srv.response.obstacles.at(0).header.stamp, obst_to_global_transform);
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN(
                            "Failed to transform the goal pose from %s into the %s frame: %s",
                            srv.response.obstacles.at(0).header.frame_id.c_str(), global_frame_.c_str(), ex.what());
                    return obstacles;
                }

                tf::Stamped<tf::Point> tf_pose;
                geometry_msgs::PoseStamped newer_pose;
                geometry_msgs::Point p;
                Vector2 v;
                for (geometry_msgs::PolygonStamped poly: srv.response.obstacles) {
                    Obstacle obst;
                    for (geometry_msgs::Point32 p32: poly.polygon.points) {
                        p.x = p32.x;
                        p.y = p32.y;

                        tf::pointMsgToTF(p, tf_pose);
                        tf_pose.setData(obst_to_global_transform * tf_pose);

                        v = Vector2(tf_pose.getX(), tf_pose.getY());
                        obst.points.push_back(v);

                    }
                    obstacles.push_back(obst);
                }
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
            std::vector<Vector2> obst_footprint;
            for(Obstacle obst: obstacles) {
                //obst.points = rotateFootprint(obst.points, heading_);
                Vector2 obst_center = (obst.points[0] + obst.points[2])/2.;
                double dist = distSqPointLineSegment(obst.points[0], obst.points[1], position_);
                dist = std::min(dist, distSqPointLineSegment(obst.points[1], obst.points[2], position_));
                dist = std::min(dist, distSqPointLineSegment(obst.points[2], obst.points[3], position_));
                dist = std::min(dist, distSqPointLineSegment(obst.points[0], obst.points[3], position_));

                if (dist < min_dist_obst_) {
                    min_dist_obst_ = dist;
                }

                if (dist > planner_util_->getCurrentLimits().max_trans_vel * trunc_time_)
                    continue;
                obst_footprint.clear();
                for(Vector2 p: obst.points) {
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
                    obstacle_vo = createTruncVO(obstacle_vo, 0.5);
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

    void ROSAgent::updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose,
                                           const std::vector<geometry_msgs::PoseStamped> &new_plan, const tf::Stamped<tf::Pose>& robot_vel)
    {
        global_plan_.resize(new_plan.size());

        for (unsigned int i = 0; i < new_plan.size(); ++i) {
            global_plan_[i] = new_plan[i];
        }
        // costs for going away from path
        path_costs_->setTargetPoses(global_plan_);

        // costs for not going towards the local goal as much as possible
        goal_costs_->setTargetPoses(global_plan_);

        // alignment costs
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();

        pos_ = Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
        vel_ = Eigen::Vector3f(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(),tf::getYaw(robot_vel.getRotation()));

        double sq_dist =
                (pos_[0] - goal_pose.pose.position.x) * (pos_[0] - goal_pose.pose.position.x) +
                (pos_[1] - goal_pose.pose.position.y) * (pos_[1] - goal_pose.pose.position.y);

        // we want the robot nose to be drawn to its final position
        // (before robot turns towards goal orientation), not the end of the
        // path for the robot center. Choosing the final position after
        // turning towards goal orientation causes instability when the
        // robot needs to make a 180 degree turn at the end
        std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
        double angle_to_goal = atan2(goal_pose.pose.position.y - pos_[1], goal_pose.pose.position.x - pos_[0]);
        front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
                                                   forward_point_distance_ * cos(angle_to_goal);
        front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
                                                                                              sin(angle_to_goal);

        goal_front_costs_->setTargetPoses(front_global_plan);


        // keeping the nose on the path
        if (sq_dist > forward_point_distance_ * forward_point_distance_ ) {
            double resolution = planner_util_->getCostmap()->getResolution();
            alignment_costs_->setScale(resolution * pdist_scale_ * 0.5);
            // costs for robot being aligned with path (nose on path, not ju
            alignment_costs_->setTargetPoses(global_plan_);
        } else {
            // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
            alignment_costs_->setScale(0.0);
        }
    }

    /**
 * This function is used when other strategies are to be applied,
 * but the cost functions for obstacles are to be reused.
 */
    double ROSAgent::scoreTrajectory(Eigen::Vector3f vel_samples)
    {
        base_local_planner::Trajectory traj;
        generator_.generateTrajectory(pos_, vel_, vel_samples, traj);
        double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
        //if the trajectory is a legal one... the check passes
        if(cost >= 0) {
            return cost;
        }
        //ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);
        //otherwise the check fails
        return cost;
    }

    /**
     * This function is used when other strategies are to be applied,
     * but the cost functions for obstacles are to be reused.
     */
    bool ROSAgent::checkTrajectory(Eigen::Vector3f pos,
                                   Eigen::Vector3f vel,
                                   Eigen::Vector3f vel_samples)
    {
        obstacle_costs_->setFootprint(costmap_ros_->getRobotFootprint());
        base_local_planner::Trajectory traj;
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();
        Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        Eigen::Vector3f vsamples (1,1,1);

        generator_.initialise(pos,
                              vel,
                              goal,
                              &limits,
                              vsamples);

        generator_.generateTrajectory(pos, vel, vel_samples, traj);

        double ps, gc, gfc, ac;
        ps = path_costs_->getScale();
        gc = goal_costs_->getScale();
        gfc = goal_front_costs_->getScale();
        ac = alignment_costs_->getScale();
        path_costs_->setScale(0);
        goal_costs_->setScale(0);
        goal_front_costs_->setScale(0);
        alignment_costs_->setScale(0);
        double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
        path_costs_->setScale(ps);
        goal_costs_->setScale(gc);
        goal_front_costs_->setScale(gfc);
        alignment_costs_->setScale(ac);

        //if the trajectory is a legal one... the check passes
        if(cost >= 0) {
            return true;
        }
        ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

        //otherwise the check fails
        return false;
    }

    Eigen::Vector3f ROSAgent::createTwistFromVector(Vector2 speed, base_local_planner::LocalPlannerLimits &limits) {
        double x, y, theta;

        double speed_ang = atan2(speed.y(), speed.x());
        double dif_ang = angles::shortest_angular_distance(heading_, speed_ang);

        if (!holo_robot_) {
            double vel = collvoid::abs(speed);
            double vstar;

            if (std::abs(dif_ang) > EPSILON)
                vstar = calcVstar(vel, dif_ang);
            else
                vstar = limits.max_vel_x;

            x = std::max(std::min(vstar, vMaxAng()), limits.min_vel_x);
            y = 0.0;

            if (std::abs(dif_ang) > 3.0 * M_PI / 4.0) {
                if (last_twist_ang_ != 0.0)
                {
                    theta = sign(last_twist_ang_) *
                            std::min(std::abs(dif_ang / time_to_holo_), limits.max_rot_vel);
                }
                else {
                    theta = sign(dif_ang) *
                            std::min(std::abs(dif_ang / time_to_holo_), limits.max_rot_vel);

                }
                if (std::abs(dif_ang) > 4.0 * M_PI / 4.0) {
                    x = 0;
                }

            }
            else {
                theta = sign(dif_ang) * std::min(std::abs(dif_ang / time_to_holo_), limits.max_rot_vel);
            }

        } // end non holo robot
        else {
            collvoid::Vector2 rotated_vel = rotateVectorByAngle(new_velocity_.x(), new_velocity_.y(), -heading_);

            x = rotated_vel.x();
            y = rotated_vel.y();
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
        }

        return Eigen::Vector3f(x, y, theta);
    }

    double ROSAgent::vMaxAng() {
        double theoretical_max_v = planner_util_->getCurrentLimits().max_rot_vel * wheel_base_ / 2.0;
        //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
        return planner_util_->getCurrentLimits().max_vel_x; //TODO: fixme!!
    }


    void ROSAgent::computeClearpathVelocity(Vector2 pref_vel) {

        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        std::sort(samples_.begin(), samples_.end(), compareVelocitySamples);
        //Vector2 new_vel = evaluateClearpathSamples(samples_, all_vos, agent_vos, human_vos, additional_orca_lines_, pref_vel, max_speed, position, heading, cur_vel, use_truncation, footprint_spec, costmap, world_model);
        Vector2 new_vel = Vector2(0, 0); // = pref_vel;

        bool valid = false;
        bool foundOutside = false;
        bool withinConstraints = true;
        int optimal = -1;
        for (int i = 0; i < (int) samples_.size(); i++) {
            withinConstraints = true;
            valid = true;
            if (!isWithinAdditionalConstraints(additional_orca_lines_, samples_[i].velocity)) {
                withinConstraints = false;
            }

            for (int j = 0; j < (int) all_vos_.size(); j++) {
                if (isInsideVO(all_vos_[j], samples_[i].velocity, use_truncation_)) {
                    valid = false;
                    if (j > optimal) {
                        optimal = j;
                        new_vel = samples_[i].velocity;
                    }
                    break;
                }
            }


            double footprint_cost = 0.;
            if (use_dwa_score_) {
                Eigen::Vector3f twist = createTwistFromVector(samples_[i].velocity, limits);
                footprint_cost = scoreTrajectory(twist);
                //ROS_ERROR("footprint_cost %f", footprint_cost);
            }


            if (valid && withinConstraints && footprint_cost >= 0.) {
                new_vel = samples_[i].velocity;
                safe_samples_.push_back(samples_[i]);
            }
            if (valid && !withinConstraints && !foundOutside) {
                optimal = all_vos_.size();
                new_vel = samples_[i].velocity;
                foundOutside = true;
                //TODO project on movement constraints
            }
        }

        if (safe_samples_.size()>0 && new_sampling_) {
            std::vector<VelocitySample> samples_around_opt;
            new_vel = safe_samples_.at(0).velocity;
            double d = minDistToVOs(agent_vos_, new_vel, use_truncation_);
            d = std::min(minDistToVOs(human_vos_, new_vel, use_truncation_), d);
            if (d < 0.1 && (agent_vos_.size() + human_vos_.size()) >0 && collvoid::abs(pref_vel)>0.1) {
                //if (collvoid::abs(pref_vel)>0.1) {
// sample around optimal vel to find safe vel:
                double max_speed = planner_util_->getCurrentLimits().max_vel_x/2;
                for (size_t i=0; i< (size_t)std::min((int)safe_samples_.size(), 3); i++) {
                    createSamplesAroundOptVel(samples_around_opt, 0.2, 0.2, -max_speed, max_speed, -max_speed, max_speed, safe_samples_.at(i).velocity,
                                              25);
                    samples_around_opt.push_back(safe_samples_.at(i));
                }
//    ROS_INFO("selected j %d, of size %d", optimal, (int) all_vos.size());

                double bestDist = DBL_MAX;
                double dist_to_pref, dist_vo;
                for(VelocitySample& cur: samples_around_opt) {
                    Vector2 vel = cur.velocity;
                    if (isWithinAdditionalConstraints(additional_orca_lines_, vel) &&
                        isSafeVelocity(all_vos_, vel, use_truncation_)) {

                        double footprint_cost = 0;
                        if (use_dwa_score_) {
                            Eigen::Vector3f twist = createTwistFromVector(cur.velocity, limits);
                            footprint_cost = scoreTrajectory(twist);
                            if (footprint_cost < 0.) {
                                footprint_cost = 100;
                            }
                        }
                        double vo_scale = 1;
                        double cost = footprint_cost;
                        dist_to_pref = sqrt(absSqr(cur.velocity - pref_vel));
                        cost += 24 * dist_to_pref;
                        dist_vo = std::max((vo_scale - sqrt(std::max(minDistToVOs(agent_vos_, vel, use_truncation_, true),0.)))/vo_scale, 0.);
                        cost += 24 * dist_vo;
                        cost += 36 * std::max(( vo_scale - sqrt(std::max(minDistToVOs(human_vos_, vel, use_truncation_, true), 0.)))/vo_scale, 0.);
                        cost += 16 * std::max(( vo_scale - sqrt(std::max(minDistToVOs(static_vos_, vel, use_truncation_, true),0.)))/vo_scale, 0.);

                        cost += 16 * sqrt(absSqr(cur.velocity - velocity_));
                        //ROS_ERROR("cost pref %f, cost vo %f", dist_to_pref, dist_vo);
                        cur.cost = cost/100.;
                        if (cost < bestDist) {
                            bestDist = cost;
//ROS_WARN("best dist = %f", bestDist);
                            new_vel = vel;
                        }
                    }
                    else {
                        cur.cost = -1;
                    }
                }
                safe_samples_.insert(safe_samples_.end(), samples_around_opt.begin(), samples_around_opt.end());
            }

        }
        else {
            if (safe_samples_.size()>0) {
                new_vel = safe_samples_[0].velocity;
            }
            else {
                new_vel = planner_util_->getCurrentLimits().min_trans_vel * normalize(new_vel);
                if (std::isnan(new_vel.x()) || std::isnan(new_vel.y())) {
                    ROS_WARN("Did not find safe velocity, chosing 0");
                    new_velocity_ = Vector2();
                    return;
                }

                ROS_WARN("Did not find safe velocity, chosing outside constraints, %f, %f", new_vel.x(), new_vel.y());
            }
        }
        new_velocity_ = new_vel;
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


