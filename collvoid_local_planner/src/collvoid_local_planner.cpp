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
    if(initialized_) {
      delete me_;
      delete pt_agg_;
    }
  }

  
  void CollvoidLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if (!initialized_){
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
  
      current_waypoint_ = 0;

      ros::NodeHandle private_nh("~/" + name);

      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
      private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_,false);

      private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
      private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
      private_nh.param("acc_lim_th", acc_lim_theta_, 3.2);

      private_nh.param("holo_robot", holo_robot_, false);
      
      private_nh.param("max_vel_x", max_vel_x_, 0.5);
      private_nh.param("min_vel_x", min_vel_x_, 0.1);

      private_nh.param("max_vel_y", max_vel_y_, 0.2);
      private_nh.param("min_vel_y", min_vel_y_, 0.0);

      private_nh.param("max_vel_th", max_vel_th_, 1.5);
      private_nh.param("min_vel_th", min_vel_th_, 0.3);

      private_nh.param("wheel_base", wheel_base_, 0.25);
      
      double radius;
      private_nh.param("radius",radius, 0.5); //TODO make this safe such that it has to be set always
      circumscribed_radius_ = radius;
      private_nh.param("inscribed_radius",inscribed_radius_, 0.5); //TODO make this safe such that it has to be set always

      //global_frame_ = costmap_ros_->getGlobalFrameID();
      
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      private_nh.param<std::string>("global_frame", global_frame_ , "/map");
     
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


      nr_initial_guess_ = 0;

      private_nh.param("/use_ground_truth",use_ground_truth_,false);
      private_nh.param("/init_guess_noise_std",INIT_GUESS_NOISE_STD_, 0.0);
      private_nh.param("/scale_radius",scale_radius_,true);
  
      private_nh.param("max_neighbors",max_neighbors_,10);
      private_nh.param("neighbor_dist",neighbor_dist_,15.0);
      private_nh.param("time_horizon",time_horizon_,10.0);
      private_nh.param("time_horizon_obst",time_horizon_obst_,10.0);
	
      private_nh.param("threshold_last_seen",THRESHOLD_LAST_SEEN_,1.0);
      private_nh.param("max_initial_guess",MAX_INITIAL_GUESS_,20);
    


      ros::NodeHandle nh;

    
      std::string my_id = nh.getNamespace();
      if (strcmp(my_id.c_str(), "/") == 0) {
	char hostname[1024];
	hostname[1023] = '\0';
	gethostname(hostname,1023); 
	my_id = std::string(hostname);
      }
      ROS_INFO("My name is: %s",my_id.c_str());
      
      pt_agg_ = new PoseTwistAggregator();
      pt_agg_->initialize(nh,tf,use_ground_truth_, scale_radius_, radius, holo_robot_, robot_base_frame_, global_frame_, my_id);


      me_ = new ROSAgent();
      me_->setId(my_id);
      me_->setRadius(radius);
      me_->setMaxNeighbors(max_neighbors_);
      me_->setMaxSpeedLinear(max_vel_x_);
      me_->setNeighborDist(neighbor_dist_);
      me_->setIsHoloRobot(holo_robot_);
      me_->setTimeHorizon(time_horizon_);
      me_->setTimeHorizonObst(time_horizon_obst_);
      me_->setTimeStep(sim_period_);

      state_ = INIT;
      initialized_ = true;

      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);


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

    bool CollvoidLocalPlanner::stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel){
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    double vx = sign(robot_vel.getOrigin().x()) * std::max(0.0, (fabs(robot_vel.getOrigin().x()) - acc_lim_x_ * sim_period_));
    double vy = sign(robot_vel.getOrigin().y()) * std::max(0.0, (fabs(robot_vel.getOrigin().y()) - acc_lim_y_ * sim_period_));

    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

    //we do want to check whether or not the command is valid
    //    double yaw = tf::getYaw(global_pose.getRotation());
    bool valid_cmd = true; //TODO check if that workstc_->checkTrajectory(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw, robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw, vx, vy, vth);

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd){
      ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
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


  void CollvoidLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock(me_->odom_lock_);
    me_->base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    me_->base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    me_->base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
	      me_->base_odom_.twist.twist.linear.x, me_->base_odom_.twist.twist.linear.y, me_->base_odom_.twist.twist.angular.z);
  }

  bool CollvoidLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
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
      boost::mutex::scoped_lock(me_->odom_lock_);
      base_odom = me_->base_odom_;
    }

    return base_local_planner::isGoalReached(*tf_, global_plan_, *costmap_ros_, global_frame_, base_odom, 
					     rot_stopped_velocity_, trans_stopped_velocity_, xy_goal_tolerance_, yaw_goal_tolerance_);
  }



  bool CollvoidLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    if(!base_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, global_frame_, transformed_plan_)){
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //  tf::Stamped<tf::Pose> global_pose;
    //if(!costmap_ros_->getRobotPose(global_pose))
    //  return false;
    
    //interpolate own position
    collvoid_msgs::PoseTwistWithCovariance me_msg = pt_agg_->getLastMeMsg();
    updateROSAgentWithMsg(me_, &me_msg);

    
    tf::Stamped<tf::Pose> global_pose;
     //let's get the pose of the robot in the frame of the plan
    global_pose.setIdentity();
    global_pose.frame_id_ = robot_base_frame_;
    global_pose.stamp_ = ros::Time();
    tf_->transformPose(global_frame_, global_pose, global_pose);
    


    //we also want to clear the robot footprint from the costmap we're using
    costmap_ros_->clearRobotFootprint();

    //make sure to update the costmap we'll use for this cycle
    //costmap_ros_->getCostmapCopy(costmap_);

    // Set current velocities from odometry
    geometry_msgs::Twist global_vel;

    me_->odom_lock_.lock();
    global_vel.linear.x = me_->base_odom_.twist.twist.linear.x;
    global_vel.linear.y = me_->base_odom_.twist.twist.linear.y;
    global_vel.angular.z = me_->base_odom_.twist.twist.angular.z;
    me_->odom_lock_.unlock();

    tf::Stamped<tf::Pose> robot_vel;
    robot_vel.setData(btTransform(tf::createQuaternionFromYaw(global_vel.angular.z), btVector3(global_vel.linear.x, global_vel.linear.y, 0)));
    robot_vel.frame_id_ = robot_base_frame_;
    robot_vel.stamp_ = ros::Time();

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan_.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(global_plan_.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double yaw = tf::getYaw(goal_point.getRotation());

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if(base_local_planner::goalPositionReached(global_pose, goal_x, goal_y, xy_goal_tolerance_) || xy_tolerance_latch_){

      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if(latch_xy_goal_tolerance_)
        xy_tolerance_latch_ = true;

      //check to see if the goal orientation has been reached
      if(base_local_planner::goalOrientationReached(global_pose, goal_th, yaw_goal_tolerance_)){
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
      }
      else {
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids
        //tc_->updatePlan(transformed_plan);
	//        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        //map_viz_.publishCostCloud();

        //copy over the odometry information
        nav_msgs::Odometry base_odom;
        {
          boost::mutex::scoped_lock(me_->odom_lock_);
          base_odom = me_->base_odom_;
        }

        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if(!rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)){
          ROS_DEBUG("Not stopped yet. base_odom: x=%6.4f,y=%6.4f,z=%6.4f", base_odom.twist.twist.linear.x,base_odom.twist.twist.linear.y,base_odom.twist.twist.angular.z); 
	  if(!stopWithAccLimits(global_pose, robot_vel, cmd_vel))
            return false;
        }
        //if we're stopped... then we want to rotate to goal
        else{
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel))
            return false;
        }
      }

      //publish an empty plan because we've reached our goal position
      //publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      //publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    } 

    tf::Stamped<tf::Pose> target_pose;
    target_pose.setIdentity();
    target_pose.frame_id_ = robot_base_frame_;
    //target_pose.stamp_ = ros::Time();//transformed_plan_.back().header.stamp;

  
    //    tf::poseStampedMsgToTF(transformed_plan_[current_waypoint_], target_pose);
    geometry_msgs::PoseStamped target_pose_msg;
    findBestWaypoint(target_pose_msg, global_pose);
    //    current_waypoint_ = transformed_plan_.size()-1;
    ROS_DEBUG("Cur waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

    tf::poseStampedMsgToTF(transformed_plan_[current_waypoint_], target_pose);
    ROS_DEBUG("Collvoid: current robot pose %f %f ==> %f", global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()));
    ROS_DEBUG("Collvoid: target robot pose %f %f ==> %f", target_pose.getOrigin().x(), target_pose.getOrigin().y(), tf::getYaw(target_pose.getRotation()));
    //    global_pose.stamp_ = ros::Time();//transformed_plan_.back().header.stamp;
    //    target_pose.stamp_ = ros::Time();//transformed_plan_.back().header.stamp;

    //    tf::Pose diff = target_pose.inverse() * global_pose;
    
    // ROS_DEBUG("Collvoid: target robot pose translated %f %f ==> %f", -diff.getOrigin().x(), -diff.getOrigin().y(), tf::getYaw(diff.getRotation()));
    /*    while(fabs(diff.getOrigin().x()) <= xy_goal_tolerance_ &&
          fabs(diff.getOrigin().y()) <=  xy_goal_tolerance_)
	  //fabs(tf::getYaw(diff.getRotation())) <=  yaw_goal_tolerance_)
    {
      if(current_waypoint_ < global_plan_.size() - 1)
      {
	current_waypoint_++;
        tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
        diff = target_pose.inverse() * global_pose;
     }
      else
      {
	geometry_msgs::Twist empty_twist;
	cmd_vel = empty_twist;

        ROS_INFO("Reached goal: %d", current_waypoint_);
	return true;
      }
    }

    */
    geometry_msgs::Twist res;

    res.linear.x = target_pose.getOrigin().x() - global_pose.getOrigin().x();
    res.linear.y = target_pose.getOrigin().y() - global_pose.getOrigin().y();
    res.angular.z = angles::shortest_angular_distance(tf::getYaw(global_pose.getRotation()),atan2(res.linear.y, res.linear.x));
      //angles::shortest_angular_distance(tf::getYaw(global_pose.getRotation()),tf::getYaw(target_pose.getRotation()));
   
    RVO::Vector2 goal_dir = RVO::Vector2(res.linear.x,res.linear.y);
    // RVO::Vector2 goal_dir = RVO::Vector2(goal_x,goal_y);
    if (RVO::abs(goal_dir) > max_vel_x_) {
      goal_dir = max_vel_x_ * RVO::normalize(goal_dir);
    }

    me_->setPosition(global_pose.getOrigin().x(), global_pose.getOrigin().y());
    me_->setHeading(tf::getYaw(global_pose.getRotation()));
    me_->prefVelocity_ = RVO::Vector2(goal_dir.x(),goal_dir.y());

    addAllNeighbors();
    double T = 0.4; //in how much time I want to be on the holonomic track
    
    if (!me_->isHoloRobot()) {
    
      double min_error = 0.01;
      double max_error = 0.15;
  
      double speed = RVO::abs(me_->velocity_);
      double error = min_error + (max_error-min_error) * speed / vMaxAng(); // how much error do i allow?
  

      //  ROS_ERROR("error %6.4f", error);
      double min_theta = M_PI / 2.0;
      double max_track_speed = calculateMaxTrackSpeedAngle(T,min_theta, error);
      me_->setMaxTrackSpeed(max_track_speed);

      me_->setMaxSpeedLinear(vMaxAng());

    }
    else {
      me_->setMaxTrackSpeed(max_vel_y_);
      //TODO compute orca lines for max y speed...
    }
    me_->computeNewVelocity(); // compute ORCA velocity
    me_->setVelocity(me_->newVelocity_.x(), me_->newVelocity_.y());
    double speed_ang = atan2(me_->newVelocity_.y(), me_->newVelocity_.x());
    double dif_ang = angles::shortest_angular_distance(me_->getHeading(), speed_ang);

    if (!me_->isHoloRobot()){
      double vel = RVO::abs(me_->newVelocity_);
      double vstar;
 
      if (std::abs(dif_ang) > 0.01)
	vstar = calcVstar(vel,dif_ang);
      else
	vstar = max_vel_x_;

      //cmd.linear.x = (vel)*cos(difAng);
      //cmd.angular.z = (vel)*sin(difAng);
    
   
      cmd_vel.linear.x = std::min(vstar,vMaxAng());
      cmd_vel.linear.y = 0.0;
  
      cmd_vel.angular.z = sign(dif_ang) * std::min(std::abs(dif_ang/T),max_vel_th_);
   
      if (std::abs(cmd_vel.angular.z) == max_vel_th_)
	cmd_vel.linear.x = 0;
      //ROS_INFO("%s calc ang z=%6.4f vel = %6.4f",myId.c_str(),cmd.angular.z,vel);
  
    }
    else {
      RVO::Vector2 rotated_vel = rotateVectorByAngle(me_->newVelocity_.x(), me_->newVelocity_.y(), -me_->getHeading());

      cmd_vel.linear.x = rotated_vel.x();
      cmd_vel.linear.y = rotated_vel.y();
      cmd_vel.angular.z = sign(dif_ang) * std::min(std::abs(dif_ang),max_vel_th_);
    }

    if(std::abs(cmd_vel.angular.z)<min_vel_th_)
      cmd_vel.angular.z = 0.0;
    if(std::abs(cmd_vel.linear.x)<min_vel_x_)
      cmd_vel.linear.x = 0.0;
    if(std::abs(cmd_vel.linear.y)<min_vel_y_)
      cmd_vel.linear.y = 0.0;

    if (current_waypoint_ < transformed_plan_.size()-1)
      transformed_plan_.erase(transformed_plan_.begin()+current_waypoint_,transformed_plan_.end());
    base_local_planner::publishPlan(transformed_plan_, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
    return true;
  }
  
  void CollvoidLocalPlanner::findBestWaypoint(geometry_msgs::PoseStamped& target_pose, const tf::Stamped<tf::Pose>& global_pose){
    current_waypoint_ = 0;
    double min_dist = DBL_MAX;
    for (size_t i=0; i < transformed_plan_.size(); i++) 
      {
	double y = global_pose.getOrigin().y();
	double x = global_pose.getOrigin().x();
	double dist = base_local_planner::distance(x, y, transformed_plan_[i].pose.position.x, transformed_plan_[i].pose.position.y);
	if (dist < min_dist) {
	  min_dist = dist;
	  target_pose = transformed_plan_[i];
	  current_waypoint_ = i;

	}
      }
    ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

    if (min_dist > me_->getRadius(scale_radius_)) //lets first get to the begin pose of the plan
      return;
    
    if (current_waypoint_ == transformed_plan_.size()-1) //I am at the end of the plan
      return;

    double dif_x = transformed_plan_[current_waypoint_+1].pose.position.x - target_pose.pose.position.x;
    double dif_y = transformed_plan_[current_waypoint_+1].pose.position.y - target_pose.pose.position.y;
    
    double plan_dir = atan2(dif_y, dif_x);

    double dif_ang = plan_dir;

    ROS_DEBUG("dif = %f,%f of %f",dif_x,dif_y, dif_ang );
    
    size_t look_ahead_ind = 0;
    bool look_ahead = false;

    for (size_t i=current_waypoint_+1; i<transformed_plan_.size(); i++) {
      dif_x = transformed_plan_[i].pose.position.x - target_pose.pose.position.x;
      dif_y = transformed_plan_[i].pose.position.y - target_pose.pose.position.y;
    
      dif_ang = atan2(dif_y, dif_x);
      target_pose = transformed_plan_[current_waypoint_];

      if(fabs(plan_dir - dif_ang) > 2.0* yaw_goal_tolerance_) {
	  target_pose = transformed_plan_[i-1];
	  current_waypoint_ = i-1;
	  ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

	  return;
      }
    }
    target_pose = transformed_plan_.back();
    current_waypoint_ = transformed_plan_.size()-1;
    
    
  }


  void CollvoidLocalPlanner::addAllNeighbors(){
    me_->clearNeighbors();
    std::vector<collvoid_msgs::PoseTwistWithCovariance*> neighbor_msgs = pt_agg_->getNeighbors();
    size_t i;
    for(i=0; i < neighbor_msgs.size(); i++)
      {
	if (strcmp(neighbor_msgs[i]->robot_id.c_str(),me_->getId().c_str()) != 0) {
	  
	  float range_sq = RVO::sqr(neighbor_dist_);
	  double time_dif = (ros::Time::now()-neighbor_msgs[i]->header.stamp).toSec();
	  if (time_dif > THRESHOLD_LAST_SEEN_)
	    continue;
	  
	  //robots[i].agent->position_ = robots[i].agent->position_ + robots[i].agent->velocity_ * timeDif;
	  //ROS_INFO("Position of robot i =%d at: [%f, %f] with vel = [%f, %f], timeDif = %f", i, robots[i].agent->position_.x(), robots[i].agent->position_.y(),robots[i].agent->velocity_.x(),robots[i].agent->velocity_.y(),timeDif);
	  // if (neighbors_.count(neighbor_msgs[i]->robot_id) == 0) {
	  ROSAgent new_neighbor;
	  //   neighbors_[neighbor_msgs[i]->robot_id] = new_neighbor;
	  updateROSAgentWithMsg(&new_neighbor, neighbor_msgs[i]);
	  me_->insertAgentNeighbor(&new_neighbor,range_sq);
	}
      }
  }

  void CollvoidLocalPlanner::updateROSAgentWithMsg(ROSAgent* agent, collvoid_msgs::PoseTwistWithCovariance* msg){
    agent->setId(msg->robot_id);
    agent->setRadius(msg->radius);
    agent->setMaxNeighbors(max_neighbors_);
    agent->setMaxSpeedLinear(max_vel_x_);
    agent->setNeighborDist(neighbor_dist_);
    agent->setIsHoloRobot(msg->holo_robot);
    agent->setTimeHorizon(time_horizon_);
    agent->setTimeHorizonObst(time_horizon_obst_);
    agent->setLastSeen(msg->header.stamp);
    agent->base_odom_.twist = msg->twist;
    double time_dif = (ros::Time::now() - msg->header.stamp).toSec();
    ROS_DEBUG("time dif to last seen is: %6.4f", time_dif);
    double yaw, x_dif, y_dif, th_dif;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    th_dif =  time_dif * msg->twist.twist.angular.z;
    if (agent->isHoloRobot()) {
      x_dif = time_dif * msg->twist.twist.linear.x;
      y_dif = time_dif * msg->twist.twist.linear.y;
    }
    else {
      x_dif = time_dif * msg->twist.twist.linear.x * cos(yaw + th_dif/2.0);
      y_dif = time_dif * msg->twist.twist.linear.x * sin(yaw + th_dif/2.0);

    }
    agent->setHeading(yaw + th_dif);
    agent->setPosition(msg->pose.pose.position.x + x_dif, msg->pose.pose.position.y + y_dif);   

   
    RVO::Vector2 vel = rotateVectorByAngle(msg->twist.twist.linear.x, msg->twist.twist.linear.y, agent->getHeading());
    agent->setVelocity(vel.x(),vel.y());
    
  }

  double CollvoidLocalPlanner::vMaxAng(){
    return max_vel_th_ - std::abs(me_->base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
  }

  double CollvoidLocalPlanner::beta(double T, double theta){
    double v_max = vMaxAng();
    return - ((2.0 * RVO::sqr(T) * sin(theta)) / theta) * v_max;
  }

  double CollvoidLocalPlanner::gamma(double T, double theta, double error) {
    double v_max = vMaxAng();
    return (((2.0 * RVO::sqr(T) * (1.0 - cos(theta))) / RVO::sqr(theta)) * RVO::sqr(v_max) - RVO::sqr(error));
  }

  double CollvoidLocalPlanner::calcVstar(double vh, double theta){
    return vh * ((theta * sin(theta))/(2.0 * (1.0 - cos(theta))));
  }

  double CollvoidLocalPlanner::calcVstarError(double T,double theta, double error) {
  
    return calcVstar(error/T,theta) * sqrt((2.0*(1.0-cos(theta)))/(2.0*(1.0-cos(theta))-RVO::sqr(sin(theta))));
  }

  double CollvoidLocalPlanner::calculateMaxTrackSpeedAngle(double T, double theta, double error){
    if (theta/T <= max_vel_th_) {
      double vstar_error = calcVstarError(T,theta,error);
      if (vstar_error <=vMaxAng()) { 
	return std::min(vstar_error * (2.0 * (1.0 - cos(theta))) / (theta*sin(theta)),max_vel_x_);
      }
      else {
	double b, g;
	b = beta(T,theta);
	g = gamma(T,theta,error);
	return std::min((-b+sqrt(RVO::sqr(b)-4*RVO::sqr(T)*g))/ (2.0 * g), max_vel_x_);
      }
    }
    else
      return std::min(error*max_vel_th_/theta,max_vel_x_);
  }
  bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
      const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
      std::vector<geometry_msgs::PoseStamped>& transformed_plan){
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try{
      if (!global_plan.size() > 0)
      {
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
      robot_pose.frame_id_ = costmap.getBaseFrameID();
      robot_pose.stamp_ = ros::Time();
      tf.transformPose(plan_pose.header.frame_id, robot_pose, robot_pose);

      //we'll keep points on the plan that are within the window that we're looking at
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

      unsigned int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = DBL_MAX;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      while(i < (unsigned int)global_plan.size() && sq_dist > sq_dist_threshold){
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        ++i;
      }

      //make sure not to count the first point that is too far away
      if(i > 0)
        --i;

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      //now we'll transform until points are outside of our distance threshold
      while(i < (unsigned int)global_plan.size() && sq_dist < sq_dist_threshold){
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        const geometry_msgs::PoseStamped& pose = global_plan[i];
        poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(transform * tf_pose);
        tf_pose.stamp_ = transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        ++i;
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  RVO::Vector2 rotateVectorByAngle(double x, double y, double ang){
    double cos_a, sin_a;
    cos_a = cos(ang);
    sin_a = sin(ang);
    return RVO::Vector2(cos_a * x - sin_a * y, cos_a * y + sin_a * x);

  }

}; //end namespace


