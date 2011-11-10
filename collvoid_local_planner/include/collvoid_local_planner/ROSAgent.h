/*
 *  
 *
 *
 *  Created by Daniel Claes on 31.08.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ROSAGENT_H
#define ROSAGENT_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include "collvoid_local_planner/Agent.h"
#include "collvoid_local_planner/Obstacle.h"

class ROSAgent : public RVO::Agent {
 private:
  double beta(double T, double theta, double max_vel_x);
  double gamma(double T, double theta, double error, double max_vel_x);
  double sign(double x){
    return x < 0.0 ? -1.0 : 1.0;
  }

  void calculateObstacleLines();
  void insertStationaryAgent(const Agent*  agent);
  bool compareObstacles(const RVO::Vector2& v1, const RVO::Vector2& v2);
  bool pointInNeighbor(RVO::Vector2& point);


  float timestep_;
  float heading_;
  float max_track_speed_;
  float left_pref_;  
  float cur_allowed_error_;
  float max_radius_cov_,max_radius_uncertainty_;
  float wheel_base_;

  bool holo_robot_;
  bool delete_observations_;

  RVO::Vector2 holo_velocity_;
  ros::Time last_seen_;
  std::vector<RVO::Line> additional_orca_lines_;
  ros::Publisher line_pub_,neighbors_pub_;
  
 public:
  ROSAgent();
  ~ROSAgent();

  bool isHoloRobot();
  void setIsHoloRobot(bool holo_robot);

  void setWheelBase(float wheel_base);
  void setTimeStep(float timestep);
  
  void setHeading(float heading);
  float getHeading();
  
  void setHoloVelocity(float x, float y);
  RVO::Vector2 getHoloVelocity();

  void setId(std::string id);
  std::string getId();

  //  void setMaxRadiusCov(float max_rad_cov);
  void setLastSeen(ros::Time last_seen);
  void setMaxTrackSpeed(float max_track_speed);
  void setLeftPref(float left_pref);
  void setAdditionalOrcaLines(std::vector<RVO::Line> additional_orca_lines);
  void setRadius(float radius);
  float getRadius();
  //  void setCurAllowedError(float cur_allowed_error);
  void setPosition(float x, float y);
  void setVelocity(float x, float y);

  void setMaxSpeedLinear(float max_speed_linear);
  void setMaxNeighbors(int max_neighbors);
  void setNeighborDist(float neighbor_dist);
  void setTimeHorizon(float time_horizon);
  void setTimeHorizonObst(float time_horizon_obst);

  void setDeleteObservations(bool delete_observations);

  void clearNeighbors();

  void computeNewVelocity();
  void publishOrcaLines();
  void publishNeighborPositions();

  double calculateMaxTrackSpeedAngle(double T, double theta, double error, double max_vel_x, double max_vel_th);
  double calcVstarError(double T,double theta, double error);
  double vMaxAng(double max_vel_x);
  double calcVstar(double vh, double theta);

  void addAccelerationConstraintsXY(double max_vel_x, double acc_lim_x, double max_vel_y, double acc_lim_y, double sim_period);

  void addMovementConstraintsDiff(double error,double T,  double max_vel_x, double max_vel_th);
  void sortObstacleLines();
  boost::mutex odom_lock_, obstacle_lock_;
  nav_msgs::Odometry base_odom_; ///< @brief Used to get the velocity of the robot
  std::vector<RVO::Vector2> obstacle_points_;
};


#endif
