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
#include "collvoid_msgs/PoseTwistWithCovariance.msg"


enum State
  {
    INIT,
    RUNNING,
    STOPPED,
    AT_GOAL // implies stopped
  };


class PoseTwistAggregator{

 public:
  PoseTwistAggregator();
  ~PoseTwistAggregator();
  void initialize();
  bool getNeighborsAndMe(std::vector<ROSAgent>& neighbors, ROSAgent& me);
  
  State state_;
  
 private:

  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void basePoseGroundTruthCallback(const nav_msgs::Odometry:ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void positionShareCallback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& msg);

  void publishPoseTwist();



  tf::TransformListener* tf_; 
  bool initialized_;

  bool use_ground_truth_,scale_radius_;
  ROSAgent* me_;
  int nr_initial_guess_, MAX_INITIAL_GUESS_;
  double INIT_GUESS_NOISE_STD_,loc_error_;

  //params ORCA
  double max_speed_linear_, neighbor_dist_, time_horizon_,time_horizon_obst_;
  int max_neighbors_;

  std::string global_frame_; ///< @brief The frame in which the controller will run
  std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot

  std::vector<ROSAgent*> neighbors_;
  ros::Publisher position_share_pub_, init_guess_pub_;
  ros::Subscriber odom_sub_, position_share_sub_;
  

}
#endif //end guard
