/*
 *  
 *
 *
 *  Created by Daniel Claes on 14.06.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "collvoid_local_planner/pose_twist_aggregator.h"
#include <boost/random.hpp>

PoseTwistAggregator::PoseTwistAggregator():
  initialized_(false){
}

PoseTwistAggregator::~PoseTwistAggregator(){
  if (me_ != NULL) {
    delete me_;
  }
}

void PoseTwistAggregator::initialize(ros::NodeHandle private_nh, tf::TransformListener* tf){
  tf_ = tf;

  bool holo_robot;
  double radius;

  private_nh.param("use_ground_truth",use_ground_truth_,false);
  private_nh.param("max_neighbors",max_neighbors_,10);
  private_nh.param("neighbor_dist",neighbor_dist_,15.0);
  private_nh.param("time_horizon",time_horizon_,10.0);
  private_nh.param("time_horizon_obst",time_horizon_obst_,10.0);
	
  private_nh.param("threshold_last_seen",THRESHOLD_LAST_SEEN_,1.0);
  private_nh.param("max_initial_guess",MAX_INITIAL_GUESS_,20);
  private_nh.param("init_guess_noise_std",INIT_GUESS_NOISE_STD_, 0.0);
  private_nh.param("scale_radius",scale_radius_,true);
  
  private_nh.param("radius",radius, 0.5); //TODO make this safe such that it has to be set always

  private_nh.param("holo_robot", holo_robot, false);

  private_nh.param("max_speed_linear", max_speed_linear_, 0.5);
  private_nh.param<std::string>("global_frame", global_frame_, "/map");
  private_nh.param<std::string>("base_frame", robot_base_frame_, "/base_link");
  
  position_share_pub_ = private_nh.advertise<collvoid_msgs::PoseTwistWithCovariance>("/position_share",1);
  init_guess_pub_ = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);

  position_share_sub_ = private_nh.subscribe("/position_share",20, &PoseTwistAggregator::positionShareCallback, this);
  odom_sub_ = private_nh.subscribe("odom",1, &PoseTwistAggregator::odomCallback, this);
  amcl_pose_sub_ = private_nh.subscribe("amcl_pose",1, &PoseTwistAggregator::amclPoseCallback, this);
  base_pose_ground_truth_sub_ = private_nh.subscribe("base_pose_ground_truth",1,&PoseTwistAggregator::basePoseGroundTruthCallback,this);

  nr_initial_guess_ = 0;

  std::string my_id = private_nh.getNamespace();
  if (strcmp(my_id.c_str(), "/") == 0) {
    char hostname[1024];
    hostname[1023] = '\0';
    gethostname(hostname,1023); 
    my_id = std::string(hostname);
  }
  ROS_INFO("My name is: %s",my_id.c_str());


  me_ = new ROSAgent();
  me_->setId(my_id);
  me_->setRadius(radius);
  me_->setMaxNeighbors(max_neighbors_);
  me_->setMaxSpeedLinear(max_speed_linear_);
  me_->setNeighborDist(neighbor_dist_);
  me_->setIsHoloRobot(holo_robot);
  me_->setTimeHorizon(time_horizon_);
  me_->setTimeHorizonObst(time_horizon_obst_);


  initialized_ = true;
}


void PoseTwistAggregator::publishPoseTwist(){

  collvoid_msgs::PoseTwistWithCovariance msg;
  boost::mutex::scoped_lock lock(me_->odom_lock_);
  msg.pose.pose = me_->base_odom_.pose.pose;
  msg.header = me_->base_odom_.header;
  msg.twist.twist = me_->base_odom_.twist.twist;
  msg.header.frame_id = robot_base_frame_;
  msg.holonomic_velocity.x = me_->getHoloVelocity().x();
  msg.holonomic_velocity.y = me_->getHoloVelocity().y();
   
  msg.radius = me_->getRadius(scale_radius_);
  msg.robot_id = me_->getId();

  position_share_pub_.publish(msg);
}


void PoseTwistAggregator::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  double cov_x, cov_y, cov_ang, rad_unc;
  cov_x = msg->pose.covariance[0];
  cov_y = msg->pose.covariance[7];
  cov_ang = msg->pose.covariance[35];
  if (!use_ground_truth_ && scale_radius_) {
    rad_unc = std::max(sqrt(cov_x),sqrt(cov_y));
    if (rad_unc == rad_unc){
      me_->setMaxRadiusCov(rad_unc);
      ROS_DEBUG("%s, max radius covariance: %6.4f", me_->getId().c_str(),rad_unc);
    }
    else 
      me_->setMaxRadiusCov(0.0);
      
  }
  else
    me_->setMaxRadiusCov(-1.0);
}


void PoseTwistAggregator::basePoseGroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg){
  double true_x,true_y,true_theta;
  true_x = -msg->pose.pose.position.y;
  true_y = msg->pose.pose.position.x;
  true_theta = tf::getYaw(msg->pose.pose.orientation)+M_PI/2.0;
  if (use_ground_truth_) {
    me_->odom_lock_.lock();
    me_->setHeading(true_theta);
    me_->setPosition(true_x,true_y);
    me_->setVelocity(-msg->twist.twist.linear.y,msg->twist.twist.linear.x);
    me_->base_odom_.pose.pose.position.x = true_x;
    me_->base_odom_.pose.pose.position.y = true_y;
    me_->base_odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(true_theta);
  
    me_->base_odom_.twist.twist.linear.x = -msg->twist.twist.linear.y;
    me_->base_odom_.twist.twist.linear.y = msg->twist.twist.linear.x;
    me_->base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    me_->base_odom_.header.stamp = msg->header.stamp;

    me_->setLastSeen(msg->header.stamp);
    me_->odom_lock_.unlock();

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
      init_guess.pose.pose.position.x = true_x + sampleNormal(0.0, INIT_GUESS_NOISE_STD_);
      init_guess.pose.pose.position.y =  true_y + sampleNormal(0.0, INIT_GUESS_NOISE_STD_);
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
      if (tf_->frameExists(global_frame_) && tf_->frameExists(robot_base_frame_)) {
	try {
	  tf::StampedTransform transform;
	  tf_->waitForTransform(global_frame_, robot_base_frame_,
				      msg->header.stamp, ros::Duration(0.1));
	  tf_->lookupTransform(global_frame_, robot_base_frame_,  
				     msg->header.stamp, transform);
	  geometry_msgs::Pose pose;
	  tf::poseTFToMsg(transform, pose);
	  loc_error_ = sqrt(RVO::sqr(true_x - pose.position.x)+RVO::sqr(true_y - pose.position.y));
	  ROS_DEBUG("Localization error is: %6.4f", loc_error_);
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
  if (tf_->frameExists(global_frame_) && tf_->frameExists(robot_base_frame_)) {
    try {
      tf::StampedTransform transform;
      tf_->waitForTransform(global_frame_, robot_base_frame_, msg->header.stamp, ros::Duration(0.1));
      tf_->lookupTransform(global_frame_, robot_base_frame_, msg->header.stamp, transform);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(transform, pose);

      me_->setLastSeen(msg->header.stamp);
      me_->setHeading(tf::getYaw(pose.orientation));
      me_->setPosition(pose.position.x,pose.position.y);

      me_->setVelocity(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
      me_->odom_lock_.lock();

      me_->base_odom_.pose.pose.position.x = msg->pose.pose.position.x;
      me_->base_odom_.pose.pose.position.y =  msg->pose.pose.position.y;
      me_->base_odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(me_->getHeading());
  
      me_->base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
      me_->base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
      me_->base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
      me_->base_odom_.header.stamp = msg->header.stamp;

      me_->odom_lock_.unlock();

      publishPoseTwist();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    };
  }

}


void PoseTwistAggregator::positionShareCallback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& msg) {
  int i;
  
  std::string cur_id = msg->robot_id;
  if (strcmp(cur_id.c_str(),me_->getId().c_str()) == 0) {
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
    neighbor->setMaxSpeedLinear(max_speed_linear_);
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
    neighbors_[i]->setVelocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  else {
    RVO::Vector2 vel = rotateVectorByAngle(msg->twist.twist.linear.x, msg->twist.twist.linear.y, neighbors_[i]->getHeading());
    neighbors_[i]->setVelocity(vel.x(),vel.y());
  }
}

double sampleNormal(double mean, double sigma) {
  static boost::mt19937 rng(static_cast<unsigned> (std::time(0)));
  boost::normal_distribution<double> nd(mean, sigma);

  boost::variate_generator<boost::mt19937&, 
                           boost::normal_distribution<> > var_nor(rng, nd);

  return var_nor();
}

RVO::Vector2 rotateVectorByAngle(double x, double y, double ang){
  double cos_a, sin_a;
  cos_a = cos(ang);
  sin_a = sin(ang);
  return RVO::Vector2(cos_a * x - sin_a * y, cos_a * y + sin_a * x);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_twist_aggregator");
  ros::NodeHandle nh;
   
  PoseTwistAggregator pt_agg;
  tf::TransformListener tf;
  pt_agg.initialize(nh,&tf);
  ROS_INFO("PoseTwistAggregator initialized");
  ros::spin();
  
}
