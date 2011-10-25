/*
 *  
 *
 *
 *  Created by Daniel Claes on 31.08.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef POSE_TWIST_AGGREGATOR_H
#define POSE_TWIST_AGGREGATOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "collvoid_msgs/PoseTwistWithCovariance.h"

class PoseTwistAggregator{

 public:
  PoseTwistAggregator();
  void initialize(ros::NodeHandle private_nh, tf::TransformListener* tf, bool use_ground_truth, bool scale_radius, double radius, bool holo_robot, std::string robot_base_frame, std::string global_frame, std::string my_id );

  void initialize(ros::NodeHandle private_nh, tf::TransformListener* tf);
  std::vector<collvoid_msgs::PoseTwistWithCovariance*> getNeighbors();
  collvoid_msgs::PoseTwistWithCovariance getLastMeMsg();
  void setRadius(double radius);
  double getRadius();
  void setHoloVelocity(double x, double y);
  void publishInitialGuess(double noise_std);
  
 private:

  void basePoseGroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void positionShareCallback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& msg);
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void publishPoseTwist();
  void setMaxRadiusCov(double max_rad_unc);
  
  tf::TransformListener* tf_; 
  bool initialized_;

  bool use_ground_truth_, scale_radius_;

  std::string global_frame_; ///< @brief The frame in which the controller will run
  std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
  collvoid_msgs::PoseTwistWithCovariance last_me_msg_;
  std::vector<collvoid_msgs::PoseTwistWithCovariance*> neighbors_;
  std::string my_id_;
  double radius_, rad_unc_;
  bool holo_robot_;
  nav_msgs::Odometry base_odom_,ground_truth_;
  ros::Publisher position_share_pub_, init_guess_pub_;
  ros::Subscriber odom_sub_, position_share_sub_, base_pose_ground_truth_sub_;
  boost::mutex odom_lock_;
  geometry_msgs::Vector3 holo_velocity_;


};

double sampleNormal(double mean, double sigma);


#endif //end guard
