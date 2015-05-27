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
            costmap_ros_(NULL), tf_(NULL), initialized_(false) {
    }

    CollvoidLocalPlanner::CollvoidLocalPlanner(std::string name, tf::TransformListener *tf, Costmap2DROS *costmap_ros) :
            costmap_ros_(NULL), tf_(NULL), initialized_(false) {

        //initialize the planner
        initialize(name, tf, costmap_ros);


    }

    CollvoidLocalPlanner::~CollvoidLocalPlanner() {
    }


    void CollvoidLocalPlanner::initialize(std::string name, tf::TransformListener *tf,
                                          costmap_2d::Costmap2DROS *costmap_ros) {
        if (!initialized_) {

            tf_ = tf;
            costmap_ros_ = costmap_ros;

            rot_stopped_velocity_ = 1e-2;
            trans_stopped_velocity_ = 1e-2;

            current_waypoint_ = 0;

            ros::NodeHandle private_nh("~/" + name);


            //base local planner params
            yaw_goal_tolerance_ = getParamDef(private_nh, "yaw_goal_tolerance", 0.05);
            xy_goal_tolerance_ = getParamDef(private_nh, "xy_goal_tolerance", 0.10);
            latch_xy_goal_tolerance_ = getParamDef(private_nh, "latch_xy_goal_tolerance", false);
            ignore_goal_yaw_ = getParamDef(private_nh, "ignore_goal_jaw", false);


            ros::NodeHandle nh;


            //set my id
            my_id_ = nh.getNamespace();
            if (strcmp(my_id_.c_str(), "/") == 0) {
                char hostname[1024];
                hostname[1023] = '\0';
                gethostname(hostname, 1023);
                my_id_ = std::string(hostname);
            }
            my_id_ = getParamDef<std::string>(private_nh, "name", my_id_);
            ROS_INFO("My name is: %s", my_id_.c_str());


            //init ros agent
            me_.reset(new ROSAgent());

            //acceleration limits
            getParam(private_nh, "acc_lim_x", &acc_lim_x_);
            getParam(private_nh, "acc_lim_y", &acc_lim_y_);
            getParam(private_nh, "acc_lim_th", &acc_lim_th_);

            me_->setAccelerationConstraints(acc_lim_x_, acc_lim_y_, acc_lim_th_);

            //holo_robot
            getParam(private_nh, "holo_robot", &holo_robot_);
            me_->setIsHoloRobot(holo_robot_);

            if (!holo_robot_)
                getParam(private_nh, "wheel_base", &wheel_base_);
            else
                wheel_base_ = 0.0;
            me_->setWheelBase(wheel_base_);


            //min max speeds
            getParam(private_nh, "max_vel_with_obstacles", &max_vel_with_obstacles_);
            me_->setMaxVelWithObstacles(max_vel_with_obstacles_);

            getParam(private_nh, "max_vel_x", &max_vel_x_);
            getParam(private_nh, "min_vel_x", &min_vel_x_);
            getParam(private_nh, "max_vel_y", &max_vel_y_);
            getParam(private_nh, "min_vel_y", &min_vel_y_);
            getParam(private_nh, "max_vel_th", &max_vel_th_);
            getParam(private_nh, "min_vel_th", &min_vel_th_);
            getParam(private_nh, "min_vel_th_inplace", &min_vel_th_inplace_);

            me_->setMinMaxSpeeds(min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, min_vel_th_, max_vel_th_,
                                 min_vel_th_inplace_);

            //set radius
            getParam(private_nh, "footprint_radius", &radius_);
            me_->setFootprintRadius(radius_);

            //set frames
            robot_base_frame_ = costmap_ros_->getBaseFrameID();
            global_frame_ = getParamDef<std::string>(private_nh, "global_frame", "/map");
            me_->setGlobalFrame(global_frame_);
            me_->setRobotBaseFrame(robot_base_frame_);

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
            me_->setSimPeriod(sim_period_);



            //other params agent
            time_horizon_obst_ = getParamDef(private_nh, "time_horizon_obst", 10.0);
            time_to_holo_ = getParamDef(private_nh, "time_to_holo", 0.4);
            min_error_holo_ = getParamDef(private_nh, "min_error_holo", 0.01);
            max_error_holo_ = getParamDef(private_nh, "max_error_holo", 0.15);
            delete_observations_ = getParamDef(private_nh, "delete_observations", true);
            threshold_last_seen_ = getParamDef(private_nh, "threshold_last_seen", 1.0);
            eps_ = getParamDef(private_nh, "eps", 0.1);

            bool orca, convex, clearpath, use_truncation;
            int num_samples, type_vo;
            getParam(private_nh, "orca", &orca);
            getParam(private_nh, "convex", &convex);
            getParam(private_nh, "clearpath", &clearpath);
            getParam(private_nh, "use_truncation", &use_truncation);

            num_samples = getParamDef(private_nh, "num_samples", 400);
            type_vo = getParamDef(private_nh, "type_vo", 0); //HRVO


            me_->setTimeHorizonObst(time_horizon_obst_);
            me_->setTimeToHolo(time_to_holo_);
            me_->setMinMaxErrorHolo(min_error_holo_, max_error_holo_);
            me_->setDeleteObservations(delete_observations_);
            me_->setThresholdLastSeen(threshold_last_seen_);
            me_->setLocalizationEps(eps_);

            me_->setOrca(orca);
            me_->setClearpath(clearpath);
            me_->setTypeVO(type_vo);
            me_->setConvex(convex);
            me_->setUseTruncation(use_truncation);
            me_->setNumSamples(num_samples);


            trunc_time_ = getParamDef(private_nh, "trunc_time", 5.0);
            left_pref_ = getParamDef(private_nh, "left_pref", 0.1);

            publish_positions_period_ = getParamDef(private_nh, "publish_positions_frequency", 10.0);
            publish_positions_period_ = 1.0 / publish_positions_period_;

            publish_me_period_ = getParamDef(private_nh, "publish_me_frequency", 10.0);
            publish_me_period_ = 1.0 / publish_me_period_;

            me_->setTruncTime(trunc_time_);
            me_->setLeftPref(left_pref_);
            me_->setPublishPositionsPeriod(publish_positions_period_);
            me_->setPublishMePeriod(publish_me_period_);

            been_in_obstacle_ = false;



            //set Footprint
            std::vector<geometry_msgs::Point> footprint_points;
            footprint_points = costmap_ros_->getRobotFootprint();

            geometry_msgs::PolygonStamped footprint;
            geometry_msgs::Point32 p;
            for (int i = 0; i < (int) footprint_points.size(); i++) {
                p.x = footprint_points[i].x;
                p.y = footprint_points[i].y;
                footprint.polygon.points.push_back(p);
            }
            if (footprint.polygon.points.size() > 2)
                me_->setFootprint(footprint);
            else {
                double angle = 0;
                double step = 2 * M_PI / 72;
                while (angle < 2 * M_PI) {
                    geometry_msgs::Point32 pt;
                    pt.x = radius_ * cos(angle);
                    pt.y = radius_ * sin(angle);
                    pt.z = 0.0;
                    footprint.polygon.points.push_back(pt);
                    angle += step;
                }
                me_->setFootprint(footprint);
            }

            me_->initAsMe(tf_);
            me_->setId(my_id_);


            skip_next_ = false;
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
            std::string move_base_name = ros::this_node::getName();
            //ROS_ERROR("%s name of node", thisname.c_str());
            obstacles_sub_ = nh.subscribe(move_base_name + "/local_costmap/obstacles", 1,
                                          &CollvoidLocalPlanner::obstaclesCallback, this);

            setup_ = false;
            dsrv_ = new dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>(private_nh);
            dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>::CallbackType cb = boost::bind(
                    &CollvoidLocalPlanner::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);


            //intialize the collision planner
            //collision_planner_.initialize("collision_planner", tf_, costmap_ros_);
            //ROS_DEBUG("collision_planner initialized...");
            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        //end if init
    } // end init


    void CollvoidLocalPlanner::reconfigureCB(collvoid_local_planner::CollvoidConfig &config, uint32_t level) {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        //The first time we're called, we just want to make sure we have the
        //original configuration
        if (!setup_) {
            last_config_ = config;
            default_config_ = config;
            setup_ = true;
            return;
        }
        if (config.restore_defaults) {
            config = default_config_;
            //if someone sets restore defaults on the parameter server, prevent looping
            config.restore_defaults = false;
        }

        acc_lim_x_ = config.acc_lim_x;
        acc_lim_y_ = config.acc_lim_y;
        acc_lim_th_ = config.acc_lim_th;

        max_vel_with_obstacles_ = config.max_vel_with_obstacles;
        max_vel_x_ = config.max_vel_x;
        min_vel_x_ = config.min_vel_x;
        max_vel_y_ = config.max_vel_y;
        min_vel_y_ = config.min_vel_y;
        max_vel_th_ = config.max_vel_th;
        min_vel_th_ = config.min_vel_th;
        min_vel_th_inplace_ = config.min_vel_th_inplace;

        radius_ = config.footprint_radius;

        time_horizon_obst_ = config.time_horizon_obst;
        time_to_holo_ = config.time_to_holo;
        min_error_holo_ = config.min_error_holo;
        max_error_holo_ = config.max_error_holo;
        delete_observations_ = config.delete_observations;
        threshold_last_seen_ = config.threshold_last_seen;

        eps_ = config.eps;


        trunc_time_ = config.trunc_time;
        left_pref_ = config.left_pref;
        publish_positions_period_ = config.publish_positions_frequency;
        publish_positions_period_ = 1.0 / publish_positions_period_;
        publish_me_period_ = config.publish_me_frequency;
        publish_me_period_ = 1.0 / publish_me_period_;


        me_->setAccelerationConstraints(acc_lim_x_, acc_lim_y_, acc_lim_th_);

        me_->setMaxVelWithObstacles(max_vel_with_obstacles_);

        me_->setMinMaxSpeeds(min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, min_vel_th_, max_vel_th_,
                             min_vel_th_inplace_);

        me_->setFootprintRadius(radius_);

        me_->setTimeHorizonObst(time_horizon_obst_);
        me_->setTimeToHolo(time_to_holo_);
        me_->setMinMaxErrorHolo(min_error_holo_, max_error_holo_);

        me_->setDeleteObservations(delete_observations_);
        me_->setThresholdLastSeen(threshold_last_seen_);
        me_->setLocalizationEps(eps_);

        me_->setTruncTime(trunc_time_);
        me_->setLeftPref(left_pref_);
        me_->setPublishPositionsPeriod(publish_positions_period_);
        me_->setPublishMePeriod(publish_me_period_);


        me_->setOrca(config.orca);
        me_->setClearpath(config.clearpath);
        me_->setTypeVO(config.type_vo);
        me_->setConvex(config.convex);
        me_->setUseTruncation(config.use_truncation);
        me_->setNumSamples(config.num_samples);


        last_config_ = config;
    }


    bool CollvoidLocalPlanner::rotateToGoal(const tf::Stamped<tf::Pose> &global_pose,
                                            const tf::Stamped<tf::Pose> &robot_vel, double goal_th,
                                            geometry_msgs::Twist &cmd_vel) {
        if (ignore_goal_yaw_) {
            cmd_vel.angular.z = 0.0;
            return true;
        }
        double yaw = tf::getYaw(global_pose.getRotation());
        double vel_yaw = tf::getYaw(robot_vel.getRotation());
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

        double v_th_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
                                                     std::max(min_vel_th_inplace_, ang_diff)) : std::max(
                -1.0 * max_vel_th_,
                std::min(-1.0 * min_vel_th_inplace_, ang_diff));

        //take the acceleration limits of the robot into account
        double max_acc_vel = fabs(vel_yaw) + acc_lim_th_ * sim_period_;
        double min_acc_vel = fabs(vel_yaw) - acc_lim_th_ * sim_period_;

        v_th_samp = sign(v_th_samp) * std::min(std::max(fabs(v_th_samp), min_acc_vel), max_acc_vel);

        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = sqrt(2 * acc_lim_th_ * fabs(ang_diff));

        v_th_samp = sign(v_th_samp) * std::min(max_speed_to_stop, fabs(v_th_samp));
        if (fabs(v_th_samp) <= 0.0 * min_vel_th_inplace_)
            v_th_samp = 0.0;
        else if (fabs(v_th_samp) < min_vel_th_inplace_)
            v_th_samp = sign(v_th_samp) * max(min_vel_th_inplace_, fabs(v_th_samp));
        //we still want to lay down the footprint of the robot and check if the action is legal
        bool valid_cmd = true;// collision_planner_.checkTrajectory(0.0, 0.0, v_th_samp,true);

        ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f", v_th_samp);

        if (valid_cmd) {
            cmd_vel.angular.z = v_th_samp;
            return true;
        }

        cmd_vel.angular.z = 0.0;
        return false;
    }

    bool CollvoidLocalPlanner::stopWithAccLimits(const tf::Stamped<tf::Pose> &global_pose,
                                                 const tf::Stamped<tf::Pose> &robot_vel,
                                                 geometry_msgs::Twist &cmd_vel) {
        //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
        //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
        double vx = sign(robot_vel.getOrigin().x()) *
                    std::max(0.0, (fabs(robot_vel.getOrigin().x()) - acc_lim_x_ * sim_period_));
        double vy = sign(robot_vel.getOrigin().y()) *
                    std::max(0.0, (fabs(robot_vel.getOrigin().y()) - acc_lim_y_ * sim_period_));

        double vel_yaw = tf::getYaw(robot_vel.getRotation());
        double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_th_ * sim_period_));

        //we do want to check whether or not the command is valid
        //    double yaw = tf::getYaw(global_pose.getRotation());
        bool valid_cmd = true;//collision_planner_.checkTrajectory(vx, vy, vth, true);
        //ROS_INFO("checkTrajectory (1): %d", valid_cmd);
        //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
        if (valid_cmd) {
            //ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
            cmd_vel.linear.x = vx;
            cmd_vel.linear.y = vy;
            cmd_vel.angular.z = vth;
            return true;
        }

        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }


    // void CollvoidLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //   //we assume that the odometry is published in the frame of the base
    //   boost::mutex::scoped_lock(me_->me_lock_);
    //   me_->base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    //   me_->base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    //   me_->base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    //   ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
    // 	      me_->base_odom_.twist.twist.linear.x, me_->base_odom_.twist.twist.linear.y, me_->base_odom_.twist.twist.angular.z);
    // }

    bool CollvoidLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
        if (!initialized_) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //reset the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        current_waypoint_ = 0;
        xy_tolerance_latch_ = false;
        //transformed_plan = global_plan;
        //get the global plan in our frame
        if (!transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, global_frame_, transformed_plan_)) {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }

        return true;
    }

    bool CollvoidLocalPlanner::isGoalReached() {
        if (!initialized_) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //copy over the odometry information
        nav_msgs::Odometry base_odom;
        {
            boost::mutex::scoped_lock(me_->me_lock_);
            base_odom = me_->base_odom_;
        }
        tf::Stamped<tf::Pose> global_pose;
        costmap_ros_->getRobotPose(global_pose);

        costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();


        return base_local_planner::isGoalReached(*tf_,
                                                 global_plan_,
                                                 *costmap,
                                                 global_frame_,
                                                 global_pose,
                                                 base_odom,
                                                 rot_stopped_velocity_,
                                                 trans_stopped_velocity_,
                                                 xy_goal_tolerance_, yaw_goal_tolerance_);
    }


    bool CollvoidLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        if (!initialized_) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        tf::Stamped<tf::Pose> global_pose;
        //let's get the pose of the robot in the frame of the plan
        global_pose.setIdentity();
        global_pose.frame_id_ = robot_base_frame_;
        global_pose.stamp_ = ros::Time();
        tf_->transformPose(global_frame_, global_pose, global_pose);

        // Set current velocities from odometry
        geometry_msgs::Twist global_vel;
        me_->me_lock_.lock();
        global_vel.linear.x = me_->base_odom_.twist.twist.linear.x;
        global_vel.linear.y = me_->base_odom_.twist.twist.linear.y;
        global_vel.angular.z = me_->base_odom_.twist.twist.angular.z;
        me_->me_lock_.unlock();

        tf::Stamped<tf::Pose> robot_vel;
        robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z),
                                        tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
        robot_vel.frame_id_ = robot_base_frame_;
        robot_vel.stamp_ = ros::Time();

        tf::Stamped<tf::Pose> goal_point;
        tf::poseStampedMsgToTF(global_plan_.back(), goal_point);

        //we assume the global goal is the last point in the global plan
        double goal_x = goal_point.getOrigin().getX();
        double goal_y = goal_point.getOrigin().getY();
        double yaw = tf::getYaw(goal_point.getRotation());
        double goal_th = yaw;

        //check to see if we've reached the goal position
        if (xy_tolerance_latch_ ||
            (base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

            //if(base_local_planner::goalPositionReached(global_pose, goal_x, goal_y, xy_goal_tolerance_) || xy_tolerance_latch_){

            //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
            //just rotate in place
            if (latch_xy_goal_tolerance_)
                xy_tolerance_latch_ = true;

            //check to see if the goal orientation has been reached
            double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
            //check to see if the goal orientation has been reached
            if (fabs(angle) <= yaw_goal_tolerance_) {

                //if(base_local_planner::goalOrientationReached(global_pose, goal_th, yaw_goal_tolerance_)){
                //set the velocity command to zero
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                rotating_to_goal_ = false;
                xy_tolerance_latch_ = false;
            }
            else {
                //copy over the odometry information
                nav_msgs::Odometry base_odom;
                {
                    boost::mutex::scoped_lock(me_->me_lock_);
                    base_odom = me_->base_odom_;
                }

                //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
                if (!rotating_to_goal_ &&
                    !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
                    //ROS_DEBUG("Not stopped yet. base_odom: x=%6.4f,y=%6.4f,z=%6.4f", base_odom.twist.twist.linear.x,base_odom.twist.twist.linear.y,base_odom.twist.twist.angular.z);
                    if (!stopWithAccLimits(global_pose, robot_vel, cmd_vel))
                        return false;
                }
                    //if we're stopped... then we want to rotate to goal
                else {
                    //set this so that we know its OK to be moving
                    rotating_to_goal_ = true;
                    if (!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel))
                        return false;
                }
            }

            //publish an empty plan because we've reached our goal position
            transformed_plan_.clear();
            base_local_planner::publishPlan(transformed_plan_, g_plan_pub_);
            base_local_planner::publishPlan(transformed_plan_, l_plan_pub_);
            //we don't actually want to run the controller when we're just rotating to goal
            return true;
        }

        tf::Stamped<tf::Pose> target_pose;
        target_pose.setIdentity();
        target_pose.frame_id_ = robot_base_frame_;

        if (!skip_next_) {
            if (!transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, global_frame_, transformed_plan_)) {
                ROS_WARN("Could not transform the global plan to the frame of the controller");
                return false;
            }
            geometry_msgs::PoseStamped target_pose_msg;
            findBestWaypoint(target_pose_msg, global_pose);
        }
        tf::poseStampedMsgToTF(transformed_plan_[current_waypoint_], target_pose);


        geometry_msgs::Twist res;

        res.linear.x = target_pose.getOrigin().x() - global_pose.getOrigin().x();
        res.linear.y = target_pose.getOrigin().y() - global_pose.getOrigin().y();
        res.angular.z = angles::shortest_angular_distance(tf::getYaw(global_pose.getRotation()),
                                                          atan2(res.linear.y, res.linear.x));


        collvoid::Vector2 goal_dir = collvoid::Vector2(res.linear.x, res.linear.y);
        // collvoid::Vector2 goal_dir = collvoid::Vector2(goal_x,goal_y);
        if (collvoid::abs(goal_dir) > max_vel_x_) {
            goal_dir = max_vel_x_ * collvoid::normalize(goal_dir);
        }
        else if (collvoid::abs(goal_dir) < min_vel_x_) {
            goal_dir = min_vel_x_ * 1.2 * collvoid::normalize(goal_dir);
        }


        collvoid::Vector2 pref_vel = collvoid::Vector2(goal_dir.x(), goal_dir.y());

        //TODO collvoid added

        me_->computeNewVelocity(pref_vel, cmd_vel);


        if (std::abs(cmd_vel.angular.z) < min_vel_th_)
            cmd_vel.angular.z = 0.0;
        if (std::abs(cmd_vel.linear.x) < min_vel_x_)
            cmd_vel.linear.x = 0.0;
        if (std::abs(cmd_vel.linear.y) < min_vel_y_)
            cmd_vel.linear.y = 0.0;


        bool in_obstacle = me_->isInStaticObstacle();//collision_planner_.checkTrajectory(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,true);

        //state machine to avoid static obstacles
        if (in_obstacle && been_in_obstacle_) { //moving backwards to procced avoiding the obstacle
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = -0.1;
            cmd_vel.linear.y = 0.0;
        }
        else if (!in_obstacle && been_in_obstacle_) { // ask for a new plan

            transformed_plan_.clear();
            base_local_planner::publishPlan(transformed_plan_, g_plan_pub_);
            base_local_planner::publishPlan(transformed_plan_, l_plan_pub_);

            been_in_obstacle_ = false;

            return false;
        }
        else if (in_obstacle) {    //in the obstacle
            been_in_obstacle_ = true;
        }
        else { //valid command and not in obstacle
            skip_next_ = false;
        }


        //  if (!skip_next_ && current_waypoint_ < transformed_plan_.size()-1)
        //transformed_plan_.erase(transformed_plan_.begin()+current_waypoint_,transformed_plan_.end());
        //ROS_DEBUG("%s cmd_vel.x %6.4f, cmd_vel.y %6.4f, cmd_vel_z %6.4f", me_->getId().c_str(), cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        std::vector<geometry_msgs::PoseStamped> local_plan;
        geometry_msgs::PoseStamped pos;
        //pos.header.frame_id = robot_base_frame_;

        tf::poseStampedTFToMsg(global_pose, pos);
        local_plan.push_back(pos);
        local_plan.push_back(transformed_plan_[current_waypoint_]);
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
            double dist = base_local_planner::getGoalPositionDistance(global_pose, transformed_plan_[i].pose.position.x,
                                                                      transformed_plan_[i].pose.position.y);
            if (dist < me_->getRadius() || dist < min_dist) {
                min_dist = dist;
                target_pose = transformed_plan_[i];
                current_waypoint_ = i;

            }
        }
        //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

        //if (min_dist > pt_agg_->getRadius() && transformed_plan_.size()>2) //lets first get to the begin pose of the plan
        //  return;

        if (current_waypoint_ == transformed_plan_.size() - 1) //I am at the end of the plan
            return;

        double dif_x = transformed_plan_[current_waypoint_ + 1].pose.position.x - target_pose.pose.position.x;
        double dif_y = transformed_plan_[current_waypoint_ + 1].pose.position.y - target_pose.pose.position.y;

        double plan_dir = atan2(dif_y, dif_x);

        double dif_ang = plan_dir;

        //ROS_DEBUG("dif = %f,%f of %f",dif_x,dif_y, dif_ang );

        //size_t look_ahead_ind = 0;
        //bool look_ahead = false;

        for (size_t i = current_waypoint_ + 1; i < transformed_plan_.size(); i++) {
            dif_x = transformed_plan_[i].pose.position.x - target_pose.pose.position.x;
            dif_y = transformed_plan_[i].pose.position.y - target_pose.pose.position.y;

            dif_ang = atan2(dif_y, dif_x);
            //target_pose = transformed_plan_[current_waypoint_];

            if (fabs(plan_dir - dif_ang) > 1.0 * yaw_goal_tolerance_) {
                target_pose = transformed_plan_[i - 1];
                current_waypoint_ = i - 1;
                //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

                return;
            }
        }
        target_pose = transformed_plan_.back();
        current_waypoint_ = transformed_plan_.size() - 1;


    }

    bool CollvoidLocalPlanner::transformGlobalPlan(const tf::TransformListener &tf,
                                                   const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                                   const costmap_2d::Costmap2DROS &costmap,
                                                   const std::string &global_frame,
                                                   std::vector<geometry_msgs::PoseStamped> &transformed_plan) {
        const geometry_msgs::PoseStamped &plan_pose = global_plan[0];

        transformed_plan.clear();

        try {
            if (!global_plan.size() > 0) {
                ROS_ERROR("Recieved plan with zero length");
                return false;
            }

            tf::StampedTransform transform;
            tf.lookupTransform(global_frame, ros::Time(),
                               plan_pose.header.frame_id, plan_pose.header.stamp,
                               plan_pose.header.frame_id, transform);

            //let's get the pose of the robot in the frame of the plan
            tf::Stamped<tf::Pose> robot_pose;
            robot_pose.setIdentity();
            robot_pose.frame_id_ = costmap_ros_->getBaseFrameID();
            robot_pose.stamp_ = ros::Time();
            tf.transformPose(plan_pose.header.frame_id, robot_pose, robot_pose);

            //we'll keep points on the plan that are within the window that we're looking at
            //double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

            //      double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist = DBL_MAX;

            unsigned int cur_waypoint = 0;
            for (size_t i = 0; i < global_plan_.size(); i++) {
                //double y = robot_pose.getOrigin().y();
                //double x = robot_pose.getOrigin().x();
                double dist = base_local_planner::getGoalPositionDistance(robot_pose, global_plan_[i].pose.position.x,
                                                                          global_plan_[i].pose.position.y);
                if (dist < sqrt(sq_dist)) {
                    sq_dist = dist * dist;
                    cur_waypoint = i;

                }
            }

            unsigned int i = cur_waypoint;

            tf::Stamped<tf::Pose> tf_pose;
            geometry_msgs::PoseStamped newer_pose;

            //now we'll transform until points are outside of our distance threshold
            while (i < (unsigned int) global_plan.size()) {// && sq_dist < sq_dist_threshold){
                double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
                double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                const geometry_msgs::PoseStamped &pose = global_plan[i];
                poseStampedMsgToTF(pose, tf_pose);
                tf_pose.setData(transform * tf_pose);
                tf_pose.stamp_ = transform.stamp_;
                tf_pose.frame_id_ = global_frame;
                poseStampedTFToMsg(tf_pose, newer_pose);

                transformed_plan.push_back(newer_pose);

                ++i;
            }
        }
        catch (tf::LookupException &ex) {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex) {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex) {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(),
                          (unsigned int) global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }

    void CollvoidLocalPlanner::obstaclesCallback(const nav_msgs::GridCells::ConstPtr &msg) {
        size_t num_obst = msg->cells.size();
        boost::mutex::scoped_lock lock(me_->obstacle_lock_);
        me_->obstacle_points_.clear();

        std::cout << "CollvoidLocalPlanner::obstaclesCallback" << std::endl;
        for (size_t i = 0; i < num_obst; i++) {
            geometry_msgs::PointStamped in, result;
            in.header = msg->header;
            in.point = msg->cells[i];
            //ROS_DEBUG("Robot %s found obstacle at %f %f", my_id_.c_str(), msg->cells[i].x,msg->cells[i].y);
            ROS_INFO("Robot %s found obstacle at %f %f", my_id_.c_str(), msg->cells[i].x, msg->cells[i].y);
            try {
                tf_->waitForTransform(global_frame_, robot_base_frame_, msg->header.stamp, ros::Duration(0.2));

                tf_->transformPoint(global_frame_, in, result);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return;
            };

            me_->obstacle_points_.push_back(collvoid::Vector2(result.point.x, result.point.y));
        }
    }


}; //end namespace


