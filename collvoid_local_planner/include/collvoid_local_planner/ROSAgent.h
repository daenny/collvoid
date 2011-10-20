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
#include "collvoid_local_planner/Agent.h"
#include "collvoid_local_planner/Obstacle.h"

class ROSAgent : protected RVO::Agent {
 private:
  float timestep_;
  float heading_;
  float max_track_speed_;
  float left_pref_;  
  float max_radius_cov_;

  bool holo_robot_;

  RVO::Vector2 holo_velocity_;
  ros::Time last_seen;
  std::vector<RVO::Line> additional_orcaLines_;
    
  
 public:
  ROSAgent();
  ~ROSAgent();

  bool isHoloRobot();
  void setIsHoloRobot(bool holo_robot);
  
  void setTimeStep(float timestep);
  
  void setHeading(float heading);
  float getHeading();
  
  void setHoloVelocity(float x, float y);
  RVO::Vector2 getHoloVelocity();

  void setId(std::string id);
  std::string getId();

  void setMaxRadiusCov(float max_rad_cov);
  void setLastSeen(ros::Time last_seen);
  void setMaxTrackSpeed(float max_track_speed);
  void setLeftPref(float left_pref);
  void setAdditionalOrcaLines(std::vector<RVO::Line> additional_orca_lines);
  void setRadius(float radius);
  void setPosition(float x, float y);
  void setVelocity(float x, float y);

  void computeNewVelocity();

  boost::recursive_mutex odom_lock_;
  nav_msgs::Odometry base_odom_; ///< @brief Used to get the velocity of the robot

};


#endif
