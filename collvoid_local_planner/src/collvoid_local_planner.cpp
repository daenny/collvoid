/*
 *  
 *
 *
 *  Created by Daniel Claes on 14.06.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include "collvoid_local_planner/collvoid_local_planner.h"

using namespace std;
using namespace costmap_2d;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(collvoid_local_planner, CollvoidLocalPlanner, collvoid_local_planner::CollvoidLocalPlanner, nav_core::BaseLocalPlanner)

namespace collvoid_local_planner {
  
  CollvoidLocalPlanner::CollvoidLocalPlanner():
    costmap_ros_(NULL), tf_(NULL), initialized_(false){
  }

  CollvoidLocalPlanner::CollvoidLocalPlanner(std::string name, tf::TransformListener* tf, Costmap2DROS* costmap_ros): 
    costmap_ros_(NULL), tf_(NULL), initialized_(false){

    //initialize the planner
    initialize(name, tf, costmap_ros);
  }

  CollvoidLocalPlanner::~CollvoidLocalPlanner() {
  }

  
  void CollvoidLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if (!initialized_){
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
  

      ros::NodeHandle private_nh("~/" + name);
      

      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);

      private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
      private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
      private_nh.param("acc_lim_th", acc_lim_theta_, 3.2);

      private_nh.param("holo_robot", holo_robot_, false);
      
      private_nh.param("max_vel_x", max_vel_x_, 0.5);
      private_nh.param("min_vel_x", min_vel_x_, 0.1);

      private_nh.param("max_vel_y", max_vel_y_, 0.2);
      private_nh.param("min_vel_y", min_vel_y_, 0.0);

      private_nh.param("max_vel_theta", max_vel_th_, 1.5);
      private_nh.param("min_vel_theta", min_vel_th_, 0.3);


      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();

      std::string controller_frequency_param_name;
      if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
        sim_period_ = 0.05;
      else
      {
        double controller_frequency = 0;
        private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        if(controller_frequency > 0)
          sim_period_ = 1.0 / controller_frequency;
        else
        {
          ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
          sim_period_ = 0.05;
        }
      }
      ROS_INFO("Sim period is set to %.2f", sim_period_);



    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    //end if init
  } // end init


  bool CollvoidLocalPlanner::rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
    double yaw = tf::getYaw(global_pose.getRotation());
    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
						    std::max(min_vel_th_, ang_diff)) : std::max(-1.0 * max_vel_th_,
													 std::min(-1.0 * min_vel_th_, ang_diff));

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
    double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = true; //TODO tc_->checkTrajectory(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw, robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw, 0.0, 0.0, v_theta_samp);

    ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd){
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;

  }

  void CollvoidLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::recursive_mutex::scoped_lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
	      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  bool CollvoidLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    return true;
  }

  bool CollvoidLocalPlanner::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::recursive_mutex::scoped_lock(odom_lock_);
      base_odom = base_odom_;
    }

    return base_local_planner::isGoalReached(*tf_, global_plan_, *costmap_ros_, global_frame_, base_odom, 
					     rot_stopped_velocity_, trans_stopped_velocity_, xy_goal_tolerance_, yaw_goal_tolerance_);
  }



  bool CollvoidLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    return false;
  }




}; //end namespace
