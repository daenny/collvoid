/*
 *  
 *
 *
 *  Created by Daniel Claes on 14.06.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "collvoid_local_planner/pose_twist_aggregator.h"


PoseTwistAggregator::PoseTwistAggregator():
  initialized_(false){
}

PoseTwistAggregator::~PoseTwistAggregator(){
  if (neighbors_ != NULL) {
    neighbors.clear();
    delete neighbors;
  }
}

void initialize(){
  
}


void PoseTwistAggregator::publishPoseTwist(){
  collvoid_local_planner::PoseTwistWithCovariance msg;
  msg.pose.pose = me_->base_odom_.pose.pose;
  msg.header = me_->base_odom_.header;
  msg.twist.twist = me_->base_odom_.twist.twist;
      
  msg.holonomic_velocity.x = me_->getHoloVelocity().x();
  msg.holonomic_velocity.y = me_->getHoloVelocity().y();
   
  msg.radius = me_->getRadius();
  msg.id = me_->getId();

  position_share_pub_.publish(msg);
}


void PoseTwistAggregator::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  double cov_x, cov_y, cov_ang, rad_unc;
  cov_x = msg->pose.covariance[0];
  cov_y = msg->pose.covariance[7];
  cov_ang = msg->pose.covariance[35];
  if (!use_ground_truth_ && scale_radius_) {
    rad_unc = std::max(sqrt(covX),sqrt(covY));
    if (rad_unc == rad_unc){
      me_->setMaxRadiusCov(rad_unc);
      ROS_DEBUG("%s, max radius covariance: %6.4f", rad_unc);
    }
  }
  else
    radiusUncertainty = -1;
}


void PoseTwistAggregator::basePoseGroundTruthCallback(const nav_msgs::Odometry:ConstPtr& msg){
  double true_x,true_y,true_theta;
  true_x = -msg->pose.pose.position.y;
  true_y = msg->pose.pose.position.x;
  true_theta = tf::getYaw(msg->pose.pose.orientation)+M_PI/2.0;
  if (use_ground_truth_) {
    boost::mutex::scoped_lock lock(me_->odom_lock_);
    me_->setHeading(true_theta);
    me_->setPosition(true_x,true_y);
    me_->setVelocity(-msg->twist.twist.linear.y,msg->twist.twist.linear.x);
    me_->base_odom_.pose.pose.position.x = true_x;
    me_->base_odom_.pose.pose.position.y = true_y;
    me_->base_odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(true_theta);
  
    me_->base_odom_.twist.twist.linear.x = -msg->twist.twist.linear.y;
    me_->base_odom_.twist.twist.linear.y = msg->twist.twist.linear.x;
    me_->base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    me_->setLastSeen(msg->header.stamp);
    publishPoseTwist();
  }
  
  else { 
    if (state_ == INIT) {
      if (nr_initial_guess_ == 0)
	ROS_INFO("publishing initial guess ...");
      else if (nr_initial_guess_ == MAX_INITIAL_GUESS_)
	ROS_INFO("finished publishing initial guess!");
      else if (nr_initial_guess_ > MAX_INITIAL_GUESS_)
	return;
    
      geometry_msgs::PoseWithCovarianceStamped init_guess;
      //add initial gaussian noise
      init_guess.pose.pose.position.x = true_x + sampleNormal(0.0, NOISE_STD);
      init_guess.pose.pose.position.y =  true_y + sampleNormal(0.0, NOISE_STD);
      init_guess.pose.pose.orientation = tf::createQuaternionMsgFromYaw(true_theta);
      init_guess.header.frame_id = global_frame_;
    
      //set covariance to 0.2 in x and y and 0.068 in ang
      init_guess.pose.covariance[0] = 0.2;
      init_guess.pose.covariance[7] = 0.2;
      init_guess.pose.covariance[35] = 0.068;

      init_guess_pub_.publish(init_guess);
      nr_initial_guess_++;
  
    } else if (state_ != STOPPED){
      //lookup tf from map to baselink
      if (tf::frameExists(global_frame_) && tf::frameExists(robot_base_frame_)) {
	try {
	  tf::StampedTransform transform;
	  tfListener.waitForTransform(global_frame_, robot_base_frame_,
				      msg->header.stamp, ros::Duration(0.1));
	  tfListener.lookupTransform(global_frame_, robot_base_frame_,  
				     msg->header.stamp, transform);
	  geometry_msgs::Pose pose;
	  tf::poseTFToMsg(transform, pose);
	  loc_error_ = sqrt(RVO::sqr(true_x - pose.position.x)+RVO::sqr(true_y - pose.position.y));
 	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	}
      } //end if frame exits
    }// end else RUNNING
  } // end not use ground truth
}

void PoseTwistAggregator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  if (use_ground_truth_)
    return;
  if (tf::frameExists(global_frame_) && tf::frameExists(robot_base_frame_)) {
    try {
      boost::mutex::scoped_lock lock(odom_lock_);
      tf::StampedTransform transform;
      tfListener.waitForTransform(global_frame_, robot_base_frame_, msg->header.stamp, ros::Duration(0.1));
      tfListener.lookupTransform(global_frame_, robot_base_frame_, msg->header.stamp, transform);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(transform, pose);

      me_->setLastSeen(msg->header.stamp);
      me_->setHeading(tf::getYaw(pose.orientation));
      me_->setPosition(pose.position.x,pose.position.y);

      me_->setVelocity(msg->twist.twist.linear.x,msg->twist.twist.linear.y);

      me_->base_odom_.pose.pose.position.x = msg->pose.pose.position.x;
      me_->base_odom_.pose.pose.position.y =  msg->pose.pose.position.y;
      me_->base_odom_.pose.pose.orientation = tf::getQuaternionMsgFromYaw(me_->getHeading());
  
      me_->base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
      me_->base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
      me_->base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
      me_->base_odom_.header.stamp = msg->header.stamp;

      publishPoseTwist();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    };
  }

}


void PoseTwistAggregator::commonPositionsCallback(const collvoid_msgs::PoseTwistWithCovariaceConstPtr& msg) {
  int i;
  
  std::string cur_id = msg->robot_id;
  if (strcmp(cur_id.c_str(),me_->get_id.c_str()) == 0) {
    return;
  }

  for(i=0; i < (int)neighbors_.size(); i++)
    {
      if (strcmp(neighbors_[i]->getId().c_str(),cur_id.c_str()) == 0) {
	//I found the robot
	break;
      }
		
    }
  if (i>=(int)neighbors_.size()) { //Robot is new, so it will be added to the list
    ROSAgent* neighbor = new ROSAgent();
    neighbor->setId(cur_id);
    neighbor->setMaxNeighbors(max_neighbors_);
    neighbor->setMaxSpeed(max_speed_linear_);
    neighbor->setNeighborDist(neighbor_dist_);
    neighbor->setIsHoloRobot(msg->holo_robot);
    neighbor->setTimeHorizon(time_horizon_);
    neighbor->setTimeHorizonObst(time_horizon_obst_);
    neighbors_.push_back(neighbor);
  }
  neighbors_[i]->setLastSeen(msg->header.stamp);
  neighbors_[i]->setPosition(msg->pose.pose.position.x, msg->pose.pose.position.y);
  neighbors_[i]->setHeading(tf::getYaw(msg->pose.pose.orientation));
  neighbors_[i]->base_odom_.twist = msg->twist;
  neighbors_[i]->setRadius(msg->radius);

  if(use_ground_truth_)
    neighbors_[i].agent->velocity_ = RVO::Vector2(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  else {
    neighbors_[i].agent->velocity_ =  rotateVectorByAngle(msg->twist.twist.linear.x, msg->twist.twist.linear.y, neighbors[i]->getHeading());
  }
}

RVO::Vector2 rotateVectorByAngle(double x, double y, double ang){
  double cos_a, sin_a;
  cos_a = cos(ang);
  sin_a = sin(ang);
  return RVO::Vector2(cos_a * x - sin_a * y, cos_a * y + sin_a * x);

}
