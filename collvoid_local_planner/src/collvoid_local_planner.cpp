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


#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <std_srvs/Empty.h>
#include <costmap_2d/obstacle_layer.h>

#include "collvoid_local_planner/collvoid_local_planner.h"

using namespace std;
using namespace costmap_2d;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(collvoid_local_planner, CollvoidLocalPlanner, collvoid_local_planner::CollvoidLocalPlanner,
                        nav_core::BaseLocalPlanner)


template<typename T>
void getParam(const ros::NodeHandle nh, const string &name, T *place) {
    bool found = nh.getParam(name, *place);
    ROS_ASSERT_MSG (found, "Could not find parameter %s", name.c_str());
    ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << *place);
}


template<class T>
T getParamDef(const ros::NodeHandle nh, const string &name, const T &default_val) {
    T val;
    nh.param(name, val, default_val);
    ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val <<
                                                   "(default was " << default_val << ")");
    return val;
}

namespace collvoid_local_planner {

    CollvoidLocalPlanner::CollvoidLocalPlanner() :
            odom_helper_("odom"), initialized_(false) {
    }


    CollvoidLocalPlanner::~CollvoidLocalPlanner() {
        delete dsrv_;
        delete obstacle_costs_;
    }


    void CollvoidLocalPlanner::initialize(std::string name, tf::TransformListener *tf,
                                          costmap_2d::Costmap2DROS *costmap_ros) {
        if (!isInitialized()) {

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);
            current_waypoint_ = 0;
            ros::NodeHandle private_nh("~/" + name);

            // make sure to update the costmap we'll use for this cycle
            costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
            planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());
            //ros::NodeHandle nh;
            me_ = ROSAgentPtr(new ROSAgent);
            me_->init(private_nh, tf, &planner_util_);

            skip_next_ = false;
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

            if (private_nh.getParam("odom_topic", odom_topic_)) {
                odom_helper_.setOdomTopic(odom_topic_);
            }

            setup_ = false;

            obstacle_costs_ = new base_local_planner::ObstacleCostFunction(planner_util_.getCostmap());
            bool sum_scores;
            private_nh.param("sum_scores", sum_scores, false);
            obstacle_costs_->setSumScores(sum_scores);

            std::vector<base_local_planner::TrajectoryCostFunction *> critics;
            critics.push_back(obstacle_costs_);
            std::vector<base_local_planner::TrajectorySampleGenerator *> generator_list;
            generator_list.push_back(&generator_);

            scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

            //intialize the collision planner
            //collision_planner_.initialize("collision_planner", tf_, costmap_ros_);
            //
            dsrv_ = new dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>(private_nh);

            dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>::CallbackType cb = boost::bind(
                    &CollvoidLocalPlanner::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            ros::NodeHandle nh("~/");
            clear_costmaps_srv_ = nh.advertiseService("clear_local_costmap", &CollvoidLocalPlanner::clearCostmapsService, this);


            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        //end if init
    } // end init

    bool CollvoidLocalPlanner::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
        //clear the costmaps
        std::vector<boost::shared_ptr<costmap_2d::Layer> > *plugins = costmap_ros_->getLayeredCostmap()->getPlugins();
        for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator layer = plugins->begin(); layer != plugins->end(); ++layer) {
            boost::shared_ptr<costmap_2d::ObstacleLayer> obstacle_layer = boost::dynamic_pointer_cast<costmap_2d::ObstacleLayer>(*layer);
            if (!obstacle_layer) {
                // ROS_INFO("NO Obstacle layer\n");
                continue;
            } else {
                //ROS_INFO("REMOVING Obstacle layer\n");
                (*layer)->reset();
            }

        }
        costmap_ros_->getLayeredCostmap()->updateMap(0, 0, 0);
        costmap_ros_->updateMap();
        //ros::Duration(0.5).sleep();
        return true;
    }
    /**
 * This function is used when other strategies are to be applied,
 * but the cost functions for obstacles are to be reused.
 */
    bool CollvoidLocalPlanner::checkTrajectory(Eigen::Vector3f pos,
                                               Eigen::Vector3f vel,
                                               Eigen::Vector3f vel_samples)
    {
        obstacle_costs_->setFootprint(costmap_ros_->getRobotFootprint());
        base_local_planner::Trajectory traj;
        geometry_msgs::PoseStamped goal_pose = transformed_plan_.back();
        Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
        base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
        Eigen::Vector3f vsamples (6,0,9);

        generator_.initialise(pos,
                              vel,
                              goal,
                              &limits,
                              vsamples);

        generator_.generateTrajectory(pos, vel, vel_samples, traj);
        double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
        //if the trajectory is a legal one... the check passes
        if(cost >= 0) {
            return true;
        }
        ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

        //otherwise the check fails
        return false;
    }



    void CollvoidLocalPlanner::reconfigureCB(collvoid_local_planner::CollvoidConfig &config, uint32_t level) {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        //The first time we're called, we just want to make sure we have the
        //original configuration
        if (setup_ && config.restore_defaults) {
            config = default_config_;
            config.restore_defaults = false;
        }

        if (! setup_) {
            default_config_ = config;
            setup_ = true;
        }

        // update generic local planner params
        base_local_planner::LocalPlannerLimits limits;
        limits.max_trans_vel = config.max_trans_vel;
        limits.min_trans_vel = config.min_trans_vel;
        limits.max_vel_x = config.max_vel_x;
        limits.min_vel_x = config.min_vel_x;
        limits.max_vel_y = config.max_vel_y;
        limits.min_vel_y = config.min_vel_y;
        limits.max_rot_vel = config.max_rot_vel;
        limits.min_rot_vel = config.min_rot_vel;
        limits.acc_lim_x = config.acc_lim_x;
        limits.acc_lim_y = config.acc_lim_y;
        limits.acc_lim_theta = config.acc_lim_theta;
        limits.acc_limit_trans = config.acc_limit_trans;
        limits.xy_goal_tolerance = config.xy_goal_tolerance;
        limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
        limits.prune_plan = config.prune_plan;
        limits.trans_stopped_vel = config.trans_stopped_vel;
        limits.rot_stopped_vel = config.rot_stopped_vel;

        generator_.setParameters(config.sim_time,
                                 config.sim_granularity,
                                 config.angular_sim_granularity,
                                 config.use_dwa,
                                 me_->sim_period_);
        planner_util_.reconfigureCB(limits, config.restore_defaults);

    }

    bool CollvoidLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //when we get a new plan, we also want to clear any latch we may have on goal tolerances
        latchedStopRotateController_.resetLatching();
        ROS_INFO("Got new plan");
        return planner_util_.setPlan(orig_global_plan);
    }

    bool CollvoidLocalPlanner::isGoalReached() {
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if (! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
            ROS_INFO("Goal reached");
            return true;
        } else {
            return false;
        }
    }


    bool CollvoidLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        if (!isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if (! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        transformed_plan_.clear();
        if (! planner_util_.getLocalPlan(current_pose_, transformed_plan_)) {
            ROS_ERROR("Could not get local plan");
            return false;
        }

        //if the global plan passed in is empty... we won't do anything
        if (transformed_plan_.empty()) {
            ROS_WARN_NAMED("collvoid_local_planner", "Received an empty transformed plan.");
            return false;
        }

        // Set current velocities from odometry
        tf::Stamped<tf::Pose> robot_vel;
        odom_helper_.getRobotVel(robot_vel);

        //check to see if we've reached the goal position
        if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
            //publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishGlobalPlan(transformed_plan);
            publishLocalPlan(local_plan);
            base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
            return latchedStopRotateController_.computeVelocityCommandsStopRotate(cmd_vel,
                                                                                  limits.getAccLimits(),
                                                                                  me_->sim_period_,
                                                                                  &planner_util_,
                                                                                  odom_helper_,
                                                                                  current_pose_,
                                                                                  boost::bind(&CollvoidLocalPlanner::checkTrajectory, this, _1, _2, _3));
        }

        tf::Stamped<tf::Pose> target_pose;
        target_pose.setIdentity();
        target_pose.frame_id_ = planner_util_.getGlobalFrame(); //TODO should be base frame
        if (!skip_next_) {

            geometry_msgs::PoseStamped target_pose_msg;
            findBestWaypoint(target_pose_msg, current_pose_);
        }
        tf::poseStampedMsgToTF(transformed_plan_.at(current_waypoint_), target_pose);


        tf_->transformPose(me_->global_frame_, current_pose_, current_pose_);
        tf_->transformPose(me_->global_frame_, target_pose, target_pose);

        geometry_msgs::Twist res;



        res.linear.x = target_pose.getOrigin().x() - current_pose_.getOrigin().x();
        res.linear.y = target_pose.getOrigin().y() - current_pose_.getOrigin().y();
        res.angular.z = angles::shortest_angular_distance(tf::getYaw(current_pose_.getRotation()),
                                                          atan2(res.linear.y, res.linear.x));


        collvoid::Vector2 goal_dir = collvoid::Vector2(res.linear.x, res.linear.y);
        // collvoid::Vector2 goal_dir = collvoid::Vector2(goal_x,goal_y);

        if (collvoid::abs(goal_dir) > planner_util_.getCurrentLimits().max_vel_x) {
            goal_dir = planner_util_.getCurrentLimits().max_vel_x * collvoid::normalize(goal_dir);
        }

        else if (collvoid::abs(goal_dir) < planner_util_.getCurrentLimits().min_vel_x) {
            goal_dir = planner_util_.getCurrentLimits().min_vel_x * 1.2 * collvoid::normalize(goal_dir);
        }

        collvoid::Vector2 pref_vel = collvoid::Vector2(goal_dir.x(), goal_dir.y());
        me_->computeNewVelocity(pref_vel, cmd_vel);


        if (std::abs(cmd_vel.angular.z) < planner_util_.getCurrentLimits().min_rot_vel)
            cmd_vel.angular.z = 0.0;
        if (std::abs(cmd_vel.linear.x) < planner_util_.getCurrentLimits().min_vel_x)
            cmd_vel.linear.x = 0.0;
        if (std::abs(cmd_vel.linear.y) < planner_util_.getCurrentLimits().min_vel_y)
            cmd_vel.linear.y = 0.0;


        //  if (!skip_next_ && current_waypoint_ < transformed_plan_.size()-1)
        //transformed_plan_.erase(transformed_plan_.begin()+current_waypoint_,transformed_plan_.end());
        //ROS_DEBUG("%s cmd_vel.x %6.4f, cmd_vel.y %6.4f, cmd_vel_z %6.4f", me_->getId().c_str(), cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        std::vector<geometry_msgs::PoseStamped> local_plan;
        geometry_msgs::PoseStamped pos;

        tf::poseStampedTFToMsg(current_pose_, pos);
        local_plan.push_back(pos);
        local_plan.push_back(transformed_plan_.at(current_waypoint_));
        base_local_planner::publishPlan(transformed_plan_, g_plan_pub_);
        base_local_planner::publishPlan(local_plan, l_plan_pub_);
        //me_->publishOrcaLines();
        return true;
    }

    void CollvoidLocalPlanner::findBestWaypoint(geometry_msgs::PoseStamped &target_pose,
                                                const tf::Stamped<tf::Pose> &global_pose) {
        current_waypoint_ = 0;
        double min_dist = DBL_MAX;
        for (size_t i = current_waypoint_; i < transformed_plan_.size(); i++) {
            //double y = global_pose.getOrigin().y();
            //double x = global_pose.getOrigin().x();
            double dist = base_local_planner::getGoalPositionDistance(global_pose, transformed_plan_.at(i).pose.position.x,
                                                                      transformed_plan_.at(i).pose.position.y);
            if (dist < me_->getRadius() || dist < min_dist) {
                min_dist = dist;
                target_pose = transformed_plan_.at(i);
                current_waypoint_ = i;

            }
        }

        //ROS_INFO("waypoint = %d, of %d", current_waypoint_, (int)transformed_plan_.size());

        if (current_waypoint_ == transformed_plan_.size() - 1) //I am at the end of the plan
            return;

        double dif_x = transformed_plan_.at(current_waypoint_ + 1).pose.position.x - target_pose.pose.position.x;
        double dif_y = transformed_plan_.at(current_waypoint_ + 1).pose.position.y - target_pose.pose.position.y;

        double plan_dir = atan2(dif_y, dif_x);

        double dif_ang = plan_dir;

        //ROS_DEBUG("dif = %f,%f of %f",dif_x,dif_y, dif_ang );

        //size_t look_ahead_ind = 0;
        //bool look_ahead = false;

        for (size_t i = current_waypoint_ + 1; i < transformed_plan_.size(); i++) {
            dif_x = transformed_plan_.at(i).pose.position.x - target_pose.pose.position.x;
            dif_y = transformed_plan_.at(i).pose.position.y - target_pose.pose.position.y;

            dif_ang = atan2(dif_y, dif_x);
            //target_pose = transformed_plan_[current_waypoint_];

            if (fabs(plan_dir - dif_ang) > 1.0 * planner_util_.getCurrentLimits().yaw_goal_tolerance) {
                target_pose = transformed_plan_.at(i - 1);
                current_waypoint_ = i - 1;
                //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

                return;
            }
        }
        target_pose = transformed_plan_.back();
        current_waypoint_ = transformed_plan_.size() - 1;


    }

    void CollvoidLocalPlanner::publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, l_plan_pub_);
    }


    void CollvoidLocalPlanner::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }



}; //end namespace


