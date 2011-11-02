/*
 *  
 *
 *
 *  Created by Daniel Claes on 14.10.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#include "collvoid_local_planner/pose_twist_aggregator.h"
#include <boost/random.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


PoseTwistAggregator::PoseTwistAggregator():
initialized_(false)
{
}

void PoseTwistAggregator::initialize(ros::NodeHandle private_nh, tf::TransformListener* tf, bool use_ground_truth, bool scale_radius, double radius, bool holo_robot, std::string robot_base_frame, std::string global_frame, std::string my_id ){
  tf_ = tf;
  use_ground_truth_ = use_ground_truth;
  scale_radius_ = scale_radius;
  radius_ = radius;
  holo_robot_= holo_robot;
  robot_base_frame_ = robot_base_frame;
  global_frame_ = global_frame;
  my_id_= my_id;

  initCommon(private_nh);
  ROS_INFO("My name is: %s",my_id_.c_str());

  initialized_ = true;
}

void PoseTwistAggregator::initialize(ros::NodeHandle private_nh, tf::TransformListener* tf){

  tf_ = tf;
  rad_unc_ = -1.0;

  private_nh.param("use_ground_truth",use_ground_truth_,false);
  private_nh.param("scale_radius",scale_radius_,true);
  private_nh.param("radius",radius_, 0.5); 
  private_nh.param("holo_robot", holo_robot_, false);
  private_nh.param<std::string>("base_frame", robot_base_frame_, "/base_link");
  private_nh.param<std::string>("global_frame", global_frame_, "/map");
  
  my_id_ = private_nh.getNamespace();
  if (strcmp(my_id_.c_str(), "/") == 0) {
    char hostname[1024];
    hostname[1023] = '\0';
    gethostname(hostname,1023); 
    my_id_ = std::string(hostname);
  }
  ROS_INFO("My name is: %s",my_id_.c_str());

  initCommon(private_nh);
  initialized_ = true;
}

void PoseTwistAggregator::initCommon(ros::NodeHandle private_nh) {
  rad_unc_ = -1.0;
  position_share_pub_ = private_nh.advertise<collvoid_msgs::PoseTwistWithCovariance>("/position_share",1);
  position_share_sub_ = private_nh.subscribe("/position_share",20, &PoseTwistAggregator::positionShareCallback, this);
  odom_sub_ = private_nh.subscribe("odom",1, &PoseTwistAggregator::odomCallback, this);
  base_pose_ground_truth_sub_ = private_nh.subscribe("base_pose_ground_truth",1,&PoseTwistAggregator::basePoseGroundTruthCallback,this);
  amcl_pose_sub_ = private_nh.subscribe("amcl_pose", 1, &PoseTwistAggregator::amclPoseCallback,this);
  neighbors_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("neighbors", 10);
  init_guess_pub_ = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);
  init_guess_srv_ = private_nh.advertiseService("init_guess_pub", &PoseTwistAggregator::initGuessCallback, this);
}


void PoseTwistAggregator::publishPoseTwist(){
  if (!initialized_) {
    ROS_DEBUG_NAMED("PT","not initialized yet");
    return;
  }
  boost::mutex::scoped_lock lock(odom_lock_);
  last_me_msg_.pose.pose = base_odom_.pose.pose;
  last_me_msg_.header = base_odom_.header;
  last_me_msg_.header.frame_id = robot_base_frame_;
  last_me_msg_.twist.twist = base_odom_.twist.twist;
  last_me_msg_.holonomic_velocity.x = holo_velocity_.x;
  last_me_msg_.holonomic_velocity.y = holo_velocity_.y;
  last_me_msg_.holo_robot= holo_robot_; 
  last_me_msg_.radius = this-> getRadius();
  last_me_msg_.robot_id = my_id_;
  position_share_pub_.publish(last_me_msg_);
}

void PoseTwistAggregator::publishInitialGuess(double noise_std){
  geometry_msgs::PoseWithCovarianceStamped init_guess;
  //add initial gaussian noise
  init_guess.pose.pose.position.x = ground_truth_.pose.pose.position.x + sampleNormal(0.0, noise_std);
  init_guess.pose.pose.position.y =  ground_truth_.pose.pose.position.y + sampleNormal(0.0, noise_std);
  init_guess.pose.pose.orientation = ground_truth_.pose.pose.orientation;
  init_guess.header.frame_id = global_frame_;
    
  //set covariance to 0.2 in x and y and 0.068 in ang
  init_guess.pose.covariance[0] = 0.2;
  init_guess.pose.covariance[7] = 0.2;
  init_guess.pose.covariance[35] = 0.068;

  init_guess_pub_.publish(init_guess);
 
}

bool PoseTwistAggregator::initGuessCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  ROS_INFO_NAMED("PoseTwistAggregator","Sending init guess from grount truth");
  publishInitialGuess(0.0); //TODO: add own srvs with noise as request
  return true;

}
 

void PoseTwistAggregator::basePoseGroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg){
  double true_x,true_y,true_theta;
  true_x = -msg->pose.pose.position.y;
  true_y = msg->pose.pose.position.x;
  true_theta = tf::getYaw(msg->pose.pose.orientation)+M_PI/2.0;
  if (use_ground_truth_) {
    odom_lock_.lock();
    base_odom_.pose.pose.position.x = true_x;
    base_odom_.pose.pose.position.y = true_y;
    base_odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(true_theta);
    double cos_a, sin_a;
    cos_a = cos(-true_theta);
    sin_a = sin(-true_theta);
    base_odom_.twist.twist.linear.x = -msg->twist.twist.linear.y * cos_a - msg->twist.twist.linear.x * sin_a;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.x * cos_a - msg->twist.twist.linear.y * sin_a;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    base_odom_.header.stamp = msg->header.stamp;
    odom_lock_.unlock();
    publishPoseTwist();
  }
  ground_truth_.pose.pose.position.x = true_x;
  ground_truth_.pose.pose.position.y =  true_y;
  ground_truth_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(true_theta);
 
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
      odom_lock_.lock();
      base_odom_.pose.pose = pose;
      base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
      base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
      base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
      base_odom_.header.stamp = msg->header.stamp;

      odom_lock_.unlock();
      publishPoseTwist();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    };
  }
}

void PoseTwistAggregator::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  double cov_x, cov_y, cov_ang, rad_unc;
  cov_x = msg->pose.covariance[0];
  cov_y = msg->pose.covariance[7];
  cov_ang = msg->pose.covariance[35];
  if (!use_ground_truth_ && scale_radius_) {
    rad_unc = std::max(sqrt(cov_x),sqrt(cov_y));
    if (rad_unc == rad_unc){
      this->setMaxRadiusCov(rad_unc); // TODO: Filter radius changes!
      ROS_DEBUG("%s, max radius covariance: %6.4f", my_id_.c_str(),rad_unc);
    }
    else 
      this->setMaxRadiusCov(0.0);
      
  }
  else
    this->setMaxRadiusCov(-1.0);
}

void PoseTwistAggregator::positionShareCallback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& msg) {
  if (!initialized_) {
    ROS_DEBUG_NAMED("PT","not initialized yet");
    return;
  }
  boost::mutex::scoped_lock lock(neighbors_lock_);
  std::string cur_id = msg->robot_id;
  if (strcmp(cur_id.c_str(),my_id_.c_str()) == 0) {
    return;
  }
  size_t i;
  for(i=0; i < neighbors_.size(); i++)
  {
    if (strcmp(neighbors_[i].robot_id.c_str(),cur_id.c_str()) == 0) {
      //I found the robot
      break;
    }
  }
  
  if (i>=neighbors_.size()) { //Robot is new, so it will be added to the list
    collvoid_msgs::PoseTwistWithCovariance msgNew;
    msgNew.robot_id = cur_id;
    msgNew.holo_robot = msg->holo_robot; 
    neighbors_.push_back(msgNew);
    ROS_DEBUG("I added a new neighbor with id %s and radius %f",cur_id.c_str(),msg->radius);
  }
  neighbors_[i].header = msg->header;
  neighbors_[i].pose.pose = msg->pose.pose;
  neighbors_[i].twist.twist = msg->twist.twist;
  neighbors_[i].holonomic_velocity.x = msg->holonomic_velocity.x;
  neighbors_[i].holonomic_velocity.y = msg->holonomic_velocity.y;
  neighbors_[i].radius = msg->radius;
  publishNeighborPositions();
  //  ROS_DEBUG("Neighbor %s updated with position %f, %f and speed x %f, y %f, z%f",cur_id.c_str(),neighbors_[i].pose.pose.position.x, neighbors_[i].pose.pose.position.y, neighbors_[i].twist.twist.linear.x, neighbors_[i].twist.twist.linear.y,neighbors_[i].twist.twist.angular.z);

}

void PoseTwistAggregator::publishNeighborPositions(){
  visualization_msgs::MarkerArray sphere_list;
  sphere_list.markers.resize(2*neighbors_.size());
  ros::Time timestamp = ros::Time::now();
  for (size_t i=0;i< neighbors_.size();i++) {

    //visualization_msgs::Marker neighbor;
   //sphere_list.header.frame_id = myId + "/base_link";
   sphere_list.markers[2*i].header.frame_id = global_frame_;
   sphere_list.markers[2*i].header.stamp = timestamp;
   sphere_list.markers[2*i].ns = robot_base_frame_;
   sphere_list.markers[2*i].action = visualization_msgs::Marker::ADD;
   sphere_list.markers[2*i].pose.orientation.w = 1.0;
   sphere_list.markers[2*i].type = visualization_msgs::Marker::SPHERE;
   sphere_list.markers[2*i].scale.x = neighbors_[i].radius;
   sphere_list.markers[2*i].scale.y = neighbors_[i].radius;
   sphere_list.markers[2*i].scale.z = 0.1;
   sphere_list.markers[2*i].color.r = 1.0;
   sphere_list.markers[2*i].color.a = 1.0;
   sphere_list.markers[2*i].id = 2*i; 

   double yaw, x_dif, y_dif, th_dif, time_dif;\
   time_dif = (ros::Time::now() - neighbors_[i].header.stamp).toSec();
   yaw = tf::getYaw(neighbors_[i].pose.pose.orientation);
   th_dif =  time_dif * neighbors_[i].twist.twist.angular.z;
   if (neighbors_[i].holo_robot) {
     x_dif = time_dif * neighbors_[i].twist.twist.linear.x;
     y_dif = time_dif * neighbors_[i].twist.twist.linear.y;
   }
   else {
     x_dif = time_dif * neighbors_[i].twist.twist.linear.x * cos(yaw + th_dif/2.0);
     y_dif = time_dif * neighbors_[i].twist.twist.linear.x * sin(yaw + th_dif/2.0);
   }
   sphere_list.markers[2*i].pose.position.x = neighbors_[i].pose.pose.position.x + x_dif;
   sphere_list.markers[2*i].pose.position.y = neighbors_[i].pose.pose.position.y + y_dif;
   sphere_list.markers[2*i].pose.position.z = 0.2;


   sphere_list.markers[2*i+1].header.frame_id = global_frame_;
   sphere_list.markers[2*i+1].header.stamp = timestamp;
   sphere_list.markers[2*i+1].ns = robot_base_frame_;
   sphere_list.markers[2*i+1].action = visualization_msgs::Marker::ADD;
   sphere_list.markers[2*i+1].pose.orientation.w = 1.0;//tf::createQuaternionMsgFromYaw(yaw+th_dif);
   sphere_list.markers[2*i+1].type = visualization_msgs::Marker::ARROW;
   sphere_list.markers[2*i+1].scale.x = 0.1;
   sphere_list.markers[2*i+1].scale.y = 0.2;
   sphere_list.markers[2*i+1].scale.z = 0.1;
   sphere_list.markers[2*i+1].color.r = 1.0;
   sphere_list.markers[2*i+1].color.a = 1.0;
   sphere_list.markers[2*i+1].id = 2*i+1; 
   //sphere_list.markers[2*i+1].pose.position.x = neighbors_[i].pose.pose.position.x + x_dif;
   //sphere_list.markers[2*i+1].pose.position.y = neighbors_[i].pose.pose.position.y + y_dif;
   //sphere_list.markers[2*i+1].pose.position.z = 0.2;
   
   geometry_msgs::Point p;
   p.x = neighbors_[i].pose.pose.position.x + x_dif;
   p.y = neighbors_[i].pose.pose.position.y + y_dif;
   p.z = 0.1;
   sphere_list.markers[2*i+1].points.push_back(p);
   
   p.x += neighbors_[i].radius * 1.1 * cos(yaw + th_dif); 
   p.y += neighbors_[i].radius * 1.1 * sin(yaw + th_dif);
   sphere_list.markers[2*i+1].points.push_back(p);
   



   //geometry_msgs::Point p;
   //p.x = agentNeighbors_[i].second->position_.x();
   //p.y = agentNeighbors_[i].second->position_.y();
   //p.z = 0.2;
   //sphere_list.markers[i].points.push_back(p);
   //sphere_list.markers[i] = neighbor;
 }
 neighbors_pub_.publish(sphere_list);
}


void PoseTwistAggregator::setRadius(double radius){
  radius_ = radius;
}

double PoseTwistAggregator::getRadius(){
  if (rad_unc_ != -1.0)
    return radius_ + rad_unc_;  //TODO maybe add scaling factor in front..
  else 
    return radius_;
}

void PoseTwistAggregator::setMaxRadiusCov(double rad_unc){
  rad_unc_ = rad_unc;
}

void PoseTwistAggregator::setHoloVelocity(double x, double y){
  holo_velocity_.x = x;
  holo_velocity_.y = y;
}

std::vector<collvoid_msgs::PoseTwistWithCovariance> PoseTwistAggregator::getNeighbors(){
  return neighbors_;
}


collvoid_msgs::PoseTwistWithCovariance PoseTwistAggregator::getLastMeMsg(){
  return last_me_msg_;
}


double sampleNormal(double mean, double sigma) {
  static boost::mt19937 rng(static_cast<unsigned> (std::time(0)));
  boost::normal_distribution<double> nd(mean, sigma);

  boost::variate_generator<boost::mt19937&, 
                           boost::normal_distribution<> > var_nor(rng, nd);

  return var_nor();
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
